from rclpy.node import Node
from configparser import ConfigParser
import subprocess
import os


class VisionStreamingNode(Node):
    def __init__(self):
        super().__init__('vision_streaming_node')

        # Load configuration
        self.config_file = "/etc/vision_streaming.conf"
        self.config = self.load_config()

        # General settings
        self.rtp_ip = self.config.get("general", "rtp_ip")
        self.rtp_port = self.config.get("general", "rtp_port")

        # Primary camera settings
        self.primary_camera = self.config.get("primary", "camera_name")
        self.primary_resolution = self.config.get("primary", "resolution")
        self.primary_bitrate = self.config.get("primary", "bitrate")

        # Secondary camera settings (optional)
        self.secondary_camera = self.config.get("secondary", "camera_name", fallback=None)
        self.secondary_resolution = self.config.get("secondary", "resolution", fallback=None)
        self.secondary_bitrate = self.config.get("secondary", "bitrate", fallback=None)
        self.pip_position = self.config.get("secondary", "pip_position", fallback="bottom-right")
        self.pip_size = self.config.get("secondary", "pip_size", fallback=None)

        self.ffmpeg_process = None
        self.start_streaming()

    def load_config(self):
        parser = ConfigParser()
        if not os.path.exists(self.config_file):
            self.get_logger().error(f"Config file {self.config_file} not found!")
            raise FileNotFoundError(f"Config file {self.config_file} not found!")
        parser.read(self.config_file)
        return parser

    def start_streaming(self):
        # Build FFmpeg command based on the availability of cameras
        command = ['ffmpeg', '-thread_queue_size', '4096']

        # Add primary camera to the FFmpeg command
        command += [
            '-f', 'v4l2',
            '-input_format', 'mjpeg',
            '-video_size', self.primary_resolution,
            '-i', self.primary_camera,
        ]

        # Add secondary camera (if available) to the FFmpeg command
        if self.secondary_camera:
            pip_x, pip_y = self.calculate_overlay_position()
            command += [
                '-f', 'v4l2',
                '-input_format', 'mjpeg',
                '-video_size', self.secondary_resolution,
                '-i', self.secondary_camera,
                '-filter_complex', f'[0:v]scale={self.primary_resolution}[main];'
                                   f'[1:v]scale={self.pip_size}[pip];'
                                   f'[main][pip]overlay={pip_x}:{pip_y},format=yuv420p'
            ]
        else:
            # If no secondary camera, use only the primary camera
            command += ['-vf', 'format=yuv420p']

        # Add encoder settings
        command += [
            '-c:v', 'libx264',
            '-preset', 'ultrafast',
            '-b:v', self.primary_bitrate,
            '-f', 'rtp', f'rtp://{self.rtp_ip}:{self.rtp_port}'
        ]

        # Start the FFmpeg process with output suppression
        self.get_logger().info(f"Starting FFmpeg with command: {' '.join(command)}")
        self.ffmpeg_process = subprocess.Popen(
            command,
            stdout=subprocess.DEVNULL,  # Suppress stdout
            stderr=subprocess.DEVNULL   # Suppress stderr
        )
        self.get_logger().info("FFmpeg process started.")

    def calculate_overlay_position(self):
        # Calculate position for Picture-in-Picture (PiP)
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
            return "10", "10"  # Default to top-left

    def stop_streaming(self):
        # Stop the FFmpeg process if running
        if self.ffmpeg_process is not None:
            self.get_logger().info("Stopping FFmpeg stream")
            self.ffmpeg_process.terminate()
            self.ffmpeg_process.wait()


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
