"""Mo camera that, publish sensor_msgs/Image. Buoc test dau tien cua cap_vision.

Chay:
    ros2 run cap_vision camera_test --ros-args -p show:=true
    ros2 run cap_vision camera_test --ros-args -p device_index:=1 -p snapshot_path:=/tmp/cap0.png
Topic mac dinh: /cap_vision/image_raw (xem config/camera.yaml).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraTestNode(Node):
    def __init__(self):
        super().__init__("camera_test")
        self.declare_parameter("device_index", 0)
        self.declare_parameter("width", 640)
        self.declare_parameter("height", 480)
        self.declare_parameter("fps", 30.0)
        self.declare_parameter("frame_id", "camera_frame")
        self.declare_parameter("image_topic", "/cap_vision/image_raw")
        self.declare_parameter("show", False)
        self.declare_parameter("snapshot_path", "")

        self.device = int(self.get_parameter("device_index").value)
        width = int(self.get_parameter("width").value)
        height = int(self.get_parameter("height").value)
        fps = float(self.get_parameter("fps").value)
        self.frame_id = str(self.get_parameter("frame_id").value)
        self.show = bool(self.get_parameter("show").value)
        self.snapshot_path = str(self.get_parameter("snapshot_path").value)

        topic = str(self.get_parameter("image_topic").value)
        self.pub = self.create_publisher(Image, topic, 10)
        self.bridge = CvBridge()
        self.cap = None
        self.snapshot_done = False
        self.warned = False

        period = 1.0 / max(fps, 1.0)
        self.create_timer(period, self.on_timer)
        self.get_logger().info(
            f"Camera /dev/video{self.device} {width}x{height}@{fps}fps"
            f" -> {topic} (show={self.show})")
        self._target_size = (width, height)

    def _open(self):
        self.cap = cv2.VideoCapture(self.device)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self._target_size[0])
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._target_size[1])
        if not self.cap.isOpened():
            self.cap = None
            return False
        return True

    def on_timer(self):
        if self.cap is None:
            if not self._open():
                if not self.warned:
                    self.get_logger().error(
                        f"Khong mo duoc camera index {self.device}."
                        " Kiem tra day USB / quyen /dev/video* (thu -p device_index:=1).")
                    self.warned = True
                return
            self.warned = False
        ok, frame = self.cap.read()
        if not ok or frame is None:
            self.get_logger().warn("Doc frame that bai, thu mo lai camera.")
            try:
                self.cap.release()
            except Exception:  # noqa: BLE001 - release best-effort
                pass
            self.cap = None
            return
        msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)
        if self.snapshot_path and not self.snapshot_done:
            cv2.imwrite(self.snapshot_path, frame)
            self.get_logger().info(f"Da luu snapshot: {self.snapshot_path}")
            self.snapshot_done = True
        if self.show:
            cv2.imshow("cap_vision/camera_test (nhan q de tat cua so)", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                cv2.destroyAllWindows()
                self.set_parameters([rclpy.Parameter("show", value=False)])
                self.show = False

    def destroy_node(self):
        if self.cap is not None:
            try:
                self.cap.release()
            except (Exception, KeyboardInterrupt):  # noqa: BLE001
                pass
        try:
            cv2.destroyAllWindows()
        except Exception:  # noqa: BLE001 - headless thi bo qua
            pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        try:
            rclpy.shutdown()
        except Exception:  # noqa: BLE001 - SIGINT doi khi shutdown context 2 lan
            pass


if __name__ == "__main__":
    main()
