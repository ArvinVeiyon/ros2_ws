from rclpy.node import Node
from configparser import ConfigParser
import subprocess
import time
import os

BY_ID_DIR = "/dev/v4l/by-id"

# conf `format` -> ffmpeg -input_format
INPUT_FORMATS = {"MJPG": "mjpeg", "YUYV": "yuyv422"}

WATCHDOG_PERIOD_S = 2.0
BACKOFF_INITIAL_S = 2.0
BACKOFF_MAX_S = 30.0
STABLE_RUNTIME_S = 60.0   # runs longer than this reset the backoff


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
        """Stable by-id identity wins; camera_name is the fallback."""
        if camera_id:
            by_id = os.path.join(BY_ID_DIR, camera_id)
            if os.path.exists(by_id):
                dev = os.path.realpath(by_id)
                if dev != camera_name:
                    self.get_logger().warning(
                        f"{label} camera {camera_id} moved: conf says "
                        f"{camera_name}, resolved to {dev} (using {dev})")
                return dev
            self.get_logger().error(
                f"{label} camera {camera_id} not present in {BY_ID_DIR} "
                f"(unplugged?); falling back to {camera_name}")
        return camera_name

    def start_streaming(self):
        primary = self.resolve_camera(self.primary_id, self.primary_camera, "primary")
        if not os.path.exists(primary):
            self.get_logger().error(f"Primary camera {primary} does not exist; will retry.")
            self.schedule_restart()
            return

        command = ['ffmpeg', '-loglevel', 'error', '-nostats',
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
            stdout=subprocess.DEVNULL,
            # stderr inherited -> journald, so camera errors are visible
        )
        self.started_at = time.monotonic()
        self.get_logger().info("FFmpeg process started.")

    def schedule_restart(self):
        self.ffmpeg_process = None
        self.next_restart_at = time.monotonic() + self.backoff_s
        self.get_logger().warning(f"Retrying stream in {self.backoff_s:.0f}s.")
        self.backoff_s = min(self.backoff_s * 2, BACKOFF_MAX_S)

    def check_ffmpeg(self):
        """Watchdog: reap a dead ffmpeg, log it loudly, restart with backoff."""
        if self.ffmpeg_process is None:
            if time.monotonic() >= self.next_restart_at:
                self.start_streaming()
            return
        rc = self.ffmpeg_process.poll()
        if rc is None:
            return
        runtime = time.monotonic() - (self.started_at or time.monotonic())
        self.get_logger().error(
            f"FFmpeg exited with code {rc} after {runtime:.0f}s "
            f"(camera unplugged, wrong format, or busy device — see journal).")
        if runtime > STABLE_RUNTIME_S:
            self.backoff_s = BACKOFF_INITIAL_S
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
            self.ffmpeg_process.terminate()
            self.ffmpeg_process.wait()
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
