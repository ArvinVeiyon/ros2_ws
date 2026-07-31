from rclpy.node import Node
from configparser import ConfigParser
import subprocess
import time
import os
import re

BY_ID_DIR = "/dev/v4l/by-id"
SYSFS_V4L = "/sys/class/video4linux"

# v2.1 stable id: usbcam-<vidpid>-<serial>-i<iface>[-<tag><n>] (see
# vision_config_manager — by-id index order is not boot-stable, so ids are
# built from USB descriptors and resolved here via sysfs)
USBCAM_ID_RE = re.compile(
    r'^usbcam-(?P<vidpid>[0-9a-f]{8})-(?P<serial>.+?)-i(?P<iface>\d+)(?:-\w+)?$')


def _read_sysfs(path):
    try:
        with open(path) as f:
            return f.read().strip()
    except OSError:
        return None


def resolve_usbcam_id(camera_id):
    """usbcam-* id -> /dev/videoN via sysfs USB descriptors, or None.

    On a multi-node interface the sibling with the lowest sysfs `index` wins.
    uvcvideo registers the capture node as index 0 and its metadata node as
    index 1, and `index` is intrinsic to the device -- unlike the videoN
    number, which is assigned from the lowest free slot and so is reshuffled
    by every reboot and USB rebind (this cam has been video8, video0 and
    video1 in one evening). Picking by lowest videoN, as this did before,
    would hand ffmpeg the *metadata* node whenever the numbering happened to
    land the other way round: ffmpeg opens it happily, receives no frames,
    and hangs alive forever with nothing to log."""
    m = USBCAM_ID_RE.match(camera_id)
    if not m:
        return None
    matches = []
    for node in os.listdir(SYSFS_V4L):
        if not node.startswith("video"):
            continue
        ifdir = os.path.realpath(os.path.join(SYSFS_V4L, node, "device"))
        iface = _read_sysfs(os.path.join(ifdir, "bInterfaceNumber"))
        usbdir = os.path.dirname(ifdir)
        vid = _read_sysfs(os.path.join(usbdir, "idVendor"))
        pid = _read_sysfs(os.path.join(usbdir, "idProduct"))
        if not (iface and vid and pid):
            continue
        serial = _read_sysfs(os.path.join(usbdir, "serial")) or \
            ("port" + os.path.basename(usbdir))
        serial = re.sub(r'[^A-Za-z0-9_.]', '_', serial)
        if (vid + pid == m.group('vidpid') and serial == m.group('serial')
                and iface == m.group('iface')):
            idx = _read_sysfs(os.path.join(SYSFS_V4L, node, "index"))
            matches.append((int(idx) if idx and idx.isdigit() else 1 << 30,
                            int(re.sub(r'\D', '', node) or 0), node))
    if not matches:
        return None
    # lowest index first; videoN only breaks ties / covers a missing index
    return "/dev/" + min(matches)[2]

# conf `format` -> ffmpeg -input_format
INPUT_FORMATS = {"MJPG": "mjpeg", "YUYV": "yuyv422"}

WATCHDOG_PERIOD_S = 2.0
BACKOFF_INITIAL_S = 2.0
BACKOFF_MAX_S = 30.0
STABLE_RUNTIME_S = 60.0   # runs longer than this reset the backoff
STALL_TIMEOUT_S = 10.0    # running ffmpeg, frame counter frozen -> hung
STARTUP_GRACE_S = 30.0    # opening the camera and decoding frame 1 can be slow
STATS_PERIOD_S = 1        # ffmpeg -stats_period: progress blocks per second
KILL_GRACE_S = 5.0        # SIGTERM -> SIGKILL escalation

# Throughput floor (see check_ffmpeg). The stall check above only catches a
# *frozen* counter. On 2026-07-31 ffmpeg lost a CPU race to the rover stack,
# fell permanently behind and delivered ~1/30th of its normal rate for over an
# hour while still ticking the counter over in bursts — so nothing fired and
# nothing was logged. Healthy here is ~60 fps encoded; the collapse sat near
# 2 fps, so the floor has a wide margin either side.
RATE_FLOOR_FPS = 5.0      # sustained encode rate below this = collapsed
RATE_WINDOW_S = 20.0      # averaging window; must exceed the ~14s burst cycle
RATE_GRACE_S = 30.0       # don't judge rate until the stream has settled


class VisionStreamingNode(Node):
    """
    Streams the camera(s) selected in /etc/vision_streaming.conf (written by
    vision_config_manager, driven from QGC) to RTP via ffmpeg.

    v2 behaviors:
      - camera_id (stable /dev/v4l/by-id identity) preferred over camera_name,
        re-resolved on every ffmpeg (re)start, so /dev/videoN renumbering
        across reboots or replugs cannot silently break the stream.
      - ffmpeg watchdog: child death is reaped, logged as ERROR, and the
        stream is restarted with backoff (2s..30s). A camera failure is
        therefore always visible in the journal and self-healing, never a
        silent zombie (incident 2026-07-19: depth node in conf -> black feed).
      - stall watchdog: liveness is judged by ffmpeg's own -progress frame
        counter, not by the process being alive. A UVC camera can stop
        feeding frames while ffmpeg keeps running with every thread parked,
        so the stream dies with no exit code for the death watchdog to catch
        (incident 2026-07-26: GS video cut off, service still "active", zero
        RTP for ~5 min until a manual restart). Frozen counter -> SIGTERM,
        SIGKILL if it is wedged in a v4l2 ioctl, then the normal restart path.
      - ffmpeg stderr goes to the journal (-loglevel error -nostats keeps it
        quiet in normal operation).
    """

    def __init__(self):
        super().__init__('vision_streaming_node')

        self.config_file = "/etc/vision_streaming.conf"
        self.config = self.load_config()

        self.rtp_ip = self.config.get("general", "rtp_ip")
        self.rtp_port = self.config.get("general", "rtp_port")

        self.primary_id = self.config.get("primary", "camera_id", fallback=None)
        self.primary_camera = self.config.get("primary", "camera_name")
        self.primary_resolution = self.config.get("primary", "resolution")
        self.primary_bitrate = self.config.get("primary", "bitrate")
        self.primary_format = self.config.get("primary", "format", fallback="MJPG")

        self.secondary_id = self.config.get("secondary", "camera_id", fallback=None)
        self.secondary_camera = self.config.get("secondary", "camera_name", fallback=None)
        self.secondary_resolution = self.config.get("secondary", "resolution", fallback=None)
        self.secondary_bitrate = self.config.get("secondary", "bitrate", fallback=None)
        self.secondary_format = self.config.get("secondary", "format", fallback="MJPG")
        self.pip_position = self.config.get("secondary", "pip_position", fallback="bottom-right")
        self.pip_size = self.config.get("secondary", "pip_size", fallback=None)

        self.ffmpeg_process = None
        self.started_at = None
        self.next_restart_at = 0.0
        self.backoff_s = BACKOFF_INITIAL_S

        # stall detection state (see check_ffmpeg)
        self.progress_fd = None
        self.progress_buf = b''
        self.last_frame = -1
        self.saw_frames = False
        self.last_progress_at = 0.0

        # throughput-floor state (see check_ffmpeg)
        self.rate_ref_frame = -1
        self.rate_ref_at = 0.0

        self.start_streaming()
        self.watchdog = self.create_timer(WATCHDOG_PERIOD_S, self.check_ffmpeg)

    def load_config(self):
        parser = ConfigParser()
        if not os.path.exists(self.config_file):
            self.get_logger().error(f"Config file {self.config_file} not found!")
            raise FileNotFoundError(f"Config file {self.config_file} not found!")
        parser.read(self.config_file)
        return parser

    def resolve_camera(self, camera_id, camera_name, label):
        """Stable camera_id wins; camera_name is the fallback."""
        if camera_id:
            if camera_id.startswith("usbcam-"):
                dev = resolve_usbcam_id(camera_id)
            else:
                # legacy pre-v2.1 conf: camera_id is a /dev/v4l/by-id basename
                by_id = os.path.join(BY_ID_DIR, camera_id)
                dev = os.path.realpath(by_id) if os.path.exists(by_id) else None
            if dev:
                if dev != camera_name:
                    self.get_logger().warning(
                        f"{label} camera {camera_id} moved: conf says "
                        f"{camera_name}, resolved to {dev} (using {dev})")
                return dev
            self.get_logger().error(
                f"{label} camera {camera_id} not found by USB identity "
                f"(unplugged?); falling back to {camera_name}")
        return camera_name

    def start_streaming(self):
        primary = self.resolve_camera(self.primary_id, self.primary_camera, "primary")
        if not os.path.exists(primary):
            self.get_logger().error(f"Primary camera {primary} does not exist; will retry.")
            self.schedule_restart()
            return

        # -progress feeds the stall watchdog; it is the only thing on stdout
        command = ['ffmpeg', '-loglevel', 'error', '-nostats',
                   '-progress', 'pipe:1', '-stats_period', str(STATS_PERIOD_S),
                   '-thread_queue_size', '4096']

        command += [
            '-f', 'v4l2',
            '-input_format', INPUT_FORMATS.get(self.primary_format.upper(), 'mjpeg'),
            '-video_size', self.primary_resolution,
            '-i', primary,
        ]

        if self.secondary_camera:
            secondary = self.resolve_camera(self.secondary_id, self.secondary_camera, "secondary")
            pip_x, pip_y = self.calculate_overlay_position()
            command += [
                '-f', 'v4l2',
                '-input_format', INPUT_FORMATS.get(self.secondary_format.upper(), 'mjpeg'),
                '-video_size', self.secondary_resolution,
                '-i', secondary,
                '-filter_complex', f'[0:v]scale={self.primary_resolution}[main];'
                                   f'[1:v]scale={self.pip_size}[pip];'
                                   f'[main][pip]overlay={pip_x}:{pip_y},format=yuv420p'
            ]
        else:
            command += ['-vf', 'format=yuv420p']

        command += [
            '-c:v', 'libx264',
            '-preset', 'ultrafast',
            '-b:v', self.primary_bitrate,
            '-f', 'rtp', f'rtp://{self.rtp_ip}:{self.rtp_port}'
        ]

        self.get_logger().info(f"Starting FFmpeg with command: {' '.join(command)}")
        self.ffmpeg_process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            # stderr inherited -> journald, so camera errors are visible
        )
        self.progress_fd = self.ffmpeg_process.stdout.fileno()
        os.set_blocking(self.progress_fd, False)
        self.progress_buf = b''
        self.last_frame = -1
        self.saw_frames = False
        self.started_at = time.monotonic()
        self.last_progress_at = self.started_at
        self.reset_rate_window(self.started_at)
        self.get_logger().info("FFmpeg process started.")

    def reset_rate_window(self, now):
        """Re-anchor the throughput window at the current frame count."""
        self.rate_ref_frame = self.last_frame
        self.rate_ref_at = now

    def schedule_restart(self):
        self.ffmpeg_process = None
        self.next_restart_at = time.monotonic() + self.backoff_s
        self.get_logger().warning(f"Retrying stream in {self.backoff_s:.0f}s.")
        self.backoff_s = min(self.backoff_s * 2, BACKOFF_MAX_S)

    def drain_progress(self):
        """Consume ffmpeg's -progress output; True if the frame counter moved.

        This pipe MUST be drained every tick: let it fill (64K) and ffmpeg
        blocks writing to it, manufacturing the very stall we are watching
        for.
        """
        if self.progress_fd is None:
            return False
        while True:
            try:
                chunk = os.read(self.progress_fd, 65536)
            except (BlockingIOError, OSError):
                break
            if not chunk:          # EOF: ffmpeg exited, poll() reports it below
                break
            self.progress_buf += chunk
        lines = self.progress_buf.split(b'\n')
        self.progress_buf = lines.pop()   # keep the partial trailing line
        advanced = False
        for line in lines:
            if not line.startswith(b'frame='):
                continue
            try:
                frame = int(line[len(b'frame='):])
            except ValueError:
                continue
            if frame > self.last_frame:
                self.last_frame = frame
                advanced = True
                if frame > 0:
                    self.saw_frames = True
        return advanced

    def close_progress(self):
        if self.ffmpeg_process is not None and self.ffmpeg_process.stdout:
            try:
                self.ffmpeg_process.stdout.close()
            except OSError:
                pass
        self.progress_fd = None
        self.progress_buf = b''

    def kill_ffmpeg(self):
        """SIGTERM, escalating to SIGKILL — ffmpeg wedged in a v4l2 ioctl
        does not act on SIGTERM."""
        proc = self.ffmpeg_process
        if proc is None:
            return
        proc.terminate()
        try:
            proc.wait(timeout=KILL_GRACE_S)
        except subprocess.TimeoutExpired:
            self.get_logger().warning("FFmpeg ignored SIGTERM; sending SIGKILL.")
            proc.kill()
            try:
                proc.wait(timeout=KILL_GRACE_S)
            except subprocess.TimeoutExpired:
                self.get_logger().error(
                    "FFmpeg survived SIGKILL (stuck in the kernel); the camera "
                    "likely needs a USB reset.")
        self.close_progress()

    def check_ffmpeg(self):
        """Watchdog: restart ffmpeg when it dies, and also when it hangs alive
        with its frame counter frozen (a dead stream has no exit code)."""
        if self.ffmpeg_process is None:
            if time.monotonic() >= self.next_restart_at:
                self.start_streaming()
            return

        now = time.monotonic()
        if self.drain_progress():
            self.last_progress_at = now
        runtime = now - (self.started_at or now)

        rc = self.ffmpeg_process.poll()
        if rc is not None:
            self.get_logger().error(
                f"FFmpeg exited with code {rc} after {runtime:.0f}s "
                f"(camera unplugged, wrong format, or busy device — see journal).")
            self.close_progress()
            if runtime > STABLE_RUNTIME_S:
                self.backoff_s = BACKOFF_INITIAL_S
            self.schedule_restart()
            return

        # Alive, but is it producing? Frame 1 can lag the camera opening.
        stalled_for = now - self.last_progress_at
        if stalled_for > (STALL_TIMEOUT_S if self.saw_frames else STARTUP_GRACE_S):
            self.get_logger().error(
                f"FFmpeg alive but no new frames for {stalled_for:.0f}s after "
                f"{runtime:.0f}s (camera stopped feeding); killing the stalled "
                f"stream and restarting.")
            self.kill_ffmpeg()
            if runtime > STABLE_RUNTIME_S:
                self.backoff_s = BACKOFF_INITIAL_S
            self.schedule_restart()
            return

        # Alive and advancing — but fast enough? A throughput collapse keeps
        # the counter moving (in bursts), so the stall check above never sees
        # it. Judge the *rate* over a window wider than the burst period.
        if not self.saw_frames or runtime < RATE_GRACE_S:
            self.reset_rate_window(now)
            return

        elapsed = now - self.rate_ref_at
        if elapsed < RATE_WINDOW_S:
            return

        fps = (self.last_frame - self.rate_ref_frame) / elapsed
        self.reset_rate_window(now)
        if fps >= RATE_FLOOR_FPS:
            return

        self.get_logger().error(
            f"FFmpeg alive but only {fps:.1f} fps over the last {elapsed:.0f}s "
            f"(floor {RATE_FLOOR_FPS:.0f} fps) after {runtime:.0f}s — throughput "
            f"collapse, not a stall. Usually means the encoder lost a CPU race "
            f"and never caught up; killing and restarting.")
        self.kill_ffmpeg()
        # Deliberately NOT resetting the backoff here, unlike the paths above:
        # a long run that ends degraded has not earned a fast retry, and under
        # sustained CPU pressure resetting would restart-loop every ~60s.
        self.schedule_restart()

    def calculate_overlay_position(self):
        if self.pip_position == "bottom-right":
            return "W-w-10", "H-h-10"
        elif self.pip_position == "bottom-left":
            return "10", "H-h-10"
        elif self.pip_position == "top-right":
            return "W-w-10", "10"
        elif self.pip_position == "top-left":
            return "10", "10"
        elif self.pip_position == "center":
            return "(W-w)/2", "(H-h)/2"
        else:
            return "10", "10"

    def stop_streaming(self):
        if self.ffmpeg_process is not None:
            self.get_logger().info("Stopping FFmpeg stream")
            # kill_ffmpeg, not a bare wait(): a hung ffmpeg ignores SIGTERM and
            # would block shutdown forever
            self.kill_ffmpeg()
            self.ffmpeg_process = None


def main(args=None):
    import rclpy
    rclpy.init(args=args)
    node = VisionStreamingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down VisionStreamingNode...")
    finally:
        node.stop_streaming()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
