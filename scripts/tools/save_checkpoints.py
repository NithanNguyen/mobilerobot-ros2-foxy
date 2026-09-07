# save_checkpoints.py — chạy trên Jetson Xavier
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
import json, math, sys
from datetime import datetime

class CheckpointSaver(Node):
    def __init__(self):
        super().__init__('checkpoint_saver')
        self.points = []
        self.sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose', self.cb, 10)
        self.get_logger().info(
            "Sẵn sàng. Nhấn ENTER để lưu checkpoint tại vị trí hiện tại, 'q' để thoát.")
        self.latest_pose = None

    def cb(self, msg):
        self.latest_pose = msg

    def quat_to_yaw(self, q):
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

def main():
    rclpy.init()
    node = CheckpointSaver()
    points = []
    idx = 1

    import threading
    spinner = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spinner.start()

    while True:
        cmd = input(f"\nCheckpoint {idx} — nhấn ENTER để lưu, 'q' để thoát: ")
        if cmd.lower() == 'q':
            break
        if node.latest_pose is None:
            print("  Chưa nhận được /amcl_pose, thử lại...")
            continue

        p = node.latest_pose.pose.pose
        yaw = node.quat_to_yaw(p.orientation)
        point = {
            "id": f"CP{idx:02d}",
            "x": round(p.position.x, 4),
            "y": round(p.position.y, 4),
            "yaw_rad": round(yaw, 4),
            "yaw_deg": round(math.degrees(yaw), 2),
            "timestamp": datetime.now().isoformat()
        }
        points.append(point)
        print(f"Saved CP{idx:02d}: x={point['x']}, y={point['y']}, yaw={point['yaw_deg']}°")
        idx += 1

    # Lưu ra file
    fname = f"checkpoints_{datetime.now().strftime('%Y%m%d_%H%M')}.json"
    with open(fname, 'w') as f:
        json.dump({"checkpoints": points, "total": len(points)}, f, indent=2)
    print(f"\nSave {len(points)} checkpoint → {fname}")

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()