import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from rclpy.time import Time
import math

class PolygonPassOppositeLineTimeMeasurer(Node):
    def __init__(self):
        super().__init__('polygon_pass_opposite_line_time_measurer')

        # --- パラメータの宣言 ---

        self.initial_side_sign = None # None: 未設定, 1.0: プラス側, -1.0: マイナス側
        # 車両の寸法
        self.declare_parameter('vehicle_length', 7.24) # 車両の長さ
        self.declare_parameter('vehicle_width', 2.30)  # 車両の幅
        self.declare_parameter('base_link_to_center_offset', 2.19) #base_linkと車両中心までの距離
        # スタートラインを定義する2点
        self.declare_parameter('start_line_p1_x', 0.0)
        self.declare_parameter('start_line_p1_y', 0.0)
        self.declare_parameter('start_line_p2_x', 0.0)
        self.declare_parameter('start_line_p2_y', 0.0)
        # フィニッシュラインを定義する2点
        self.declare_parameter('finish_line_p1_x', 0.0)
        self.declare_parameter('finish_line_p1_y', 0.0)
        self.declare_parameter('finish_line_p2_x', 0.0)
        self.declare_parameter('finish_line_p2_y', 0.0)

        # --- パラメータの取得 ---
        self.vehicle_length = self.get_parameter('vehicle_length').get_parameter_value().double_value
        self.vehicle_width = self.get_parameter('vehicle_width').get_parameter_value().double_value
        self.base_link_to_center_offset = self.get_parameter('base_link_to_center_offset').get_parameter_value().double_value
        self.start_line = {
            'p1_x': self.get_parameter('start_line_p1_x').get_parameter_value().double_value,
            'p1_y': self.get_parameter('start_line_p1_y').get_parameter_value().double_value,
            'p2_x': self.get_parameter('start_line_p2_x').get_parameter_value().double_value,
            'p2_y': self.get_parameter('start_line_p2_y').get_parameter_value().double_value,
        }
        self.finish_line = {
            'p1_x': self.get_parameter('finish_line_p1_x').get_parameter_value().double_value,
            'p1_y': self.get_parameter('finish_line_p1_y').get_parameter_value().double_value,
            'p2_x': self.get_parameter('finish_line_p2_x').get_parameter_value().double_value,
            'p2_y': self.get_parameter('finish_line_p2_y').get_parameter_value().double_value,
        }

        self.get_logger().info(f"Vehicle Size: length={self.vehicle_length}, width={self.vehicle_width}")
        self.get_logger().info(f"Measurement Start Line: ({self.start_line['p1_x']}, {self.start_line['p1_y']}) -> ({self.start_line['p2_x']}, {self.start_line['p2_y']})")
        self.get_logger().info(f"Measurement Finish Line: ({self.finish_line['p1_x']}, {self.finish_line['p1_y']}) -> ({self.finish_line['p2_x']}, {self.finish_line['p2_y']})")

        # --- Subscriberの作成 ---
        self.subscription = self.create_subscription(
            Odometry,
            '/localization/kinematic_state',
            self.odometry_callback,
            10)

        # --- 状態管理用の変数 ---
        self.state = "WAITING_FOR_START"  # WAITING_FOR_START -> TIMING -> FINISHED
        self.start_time = None
        # WAITING_FOR_START状態で、車両が完全にラインの手前にいるかを確認するためのフラグ
        self.is_initial_position_behind_line = False
        self.intermediate_side_sign_for_finish_line = None # 中間領域のゴールラインに対する符号

    def get_vehicle_corners(self, pose):
        """車両の中心位置と向きから四隅の座標を計算する"""
        x = pose.position.x
        y = pose.position.y
        q = pose.orientation
        
        # クォータニオンからYaw（Z軸周りの回転）を計算
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

        half_width = self.vehicle_width / 2.0

        front_offset = self.vehicle_length / 2.0 + self.base_link_to_center_offset
        rear_offset = self.vehicle_length / 2.0 - self.base_link_to_center_offset

        # 車両ローカル座標系での四隅の座標
        corners_local = [
            (front_offset, half_width),    # front-right
            (front_offset, -half_width),   # front-left
            (-rear_offset, -half_width),  # rear-left
            (-rear_offset, half_width)    # rear-right
        ]

        # ワールド座標系に変換
        corners_global = []
        for lx, ly in corners_local:
            gx = x + lx * math.cos(yaw) - ly * math.sin(yaw)
            gy = y + lx * math.sin(yaw) + ly * math.cos(yaw)
            corners_global.append({'x': gx, 'y': gy})
        
        return corners_global

    def get_point_side_of_line(self, point, line):
        """点がラインのどちら側にあるかを判定する (外積を利用)"""
        return (line['p2_x'] - line['p1_x']) * (point['y'] - line['p1_y']) - \
               (line['p2_y'] - line['p1_y']) * (point['x'] - line['p1_x'])

    def odometry_callback(self, msg: Odometry):
        if self.state == "FINISHED":
            return

        vehicle_corners = self.get_vehicle_corners(msg.pose.pose)

        if self.state == "WAITING_FOR_START":
            sides = [self.get_point_side_of_line(corner, self.start_line) for corner in vehicle_corners]
        
            if self.initial_side_sign is None:
                # 最初の点が0でないことを確認
                significant_sides = [s for s in sides if s != 0]
                if not significant_sides: return # 全点が線上なら何もしない

                # 全ての点が同じ側にあることを確認
                if all(math.copysign(1.0, s) == math.copysign(1.0, significant_sides[0]) for s in significant_sides):
                    self.initial_side_sign = math.copysign(1.0, significant_sides[0])
                    self.get_logger().info(f"Vehicle is positioned correctly. Ready to measure. (Initial side sign: {self.initial_side_sign})")
                else:
                    self.get_logger().warn("Vehicle started on the line. Please move completely to one side.", throttle_duration_sec=5)
                return

            is_crossing = any(math.copysign(1.0, s) != self.initial_side_sign for s in sides if s != 0)

            if is_crossing:
                self.start_time = self.get_clock().now()
                self.state = "TIMING"
                self.get_logger().info("Vehicle crossed the start line. Start measuring...")

                # ★変更点：TIMING状態になった瞬間のゴールラインとの位置関係を記録
                finish_line_sides = [self.get_point_side_of_line(corner, self.finish_line) for corner in vehicle_corners]
                first_significant_finish_side = next((s for s in finish_line_sides if s != 0), None)
                if first_significant_finish_side is not None:
                    self.intermediate_side_sign_for_finish_line = math.copysign(1.0, first_significant_finish_side)
                    self.get_logger().info(f"Entered intermediate area. Finish line sign relative to vehicle: {self.intermediate_side_sign_for_finish_line}")

        elif self.state == "TIMING":
            if self.intermediate_side_sign_for_finish_line is None:
                self.get_logger().error("Could not determine intermediate side for finish line. Aborting.", throttle_duration_sec=5)
                return
                
            sides = [self.get_point_side_of_line(corner, self.finish_line) for corner in vehicle_corners]

            # ★変更点：記録した中間領域の符号と逆側に全点が入ったらゴールとみなす
            finish_side_sign = -self.intermediate_side_sign_for_finish_line
            
            significant_sides = [s for s in sides if s != 0]
            if significant_sides and all(math.copysign(1.0, s) == finish_side_sign for s in significant_sides):
                end_time = self.get_clock().now()
                elapsed_time = (end_time - self.start_time).nanoseconds / 1e9
                self.state = "FINISHED"

                self.get_logger().info("Vehicle completely crossed the finish line. Stop measuring.")
                self.get_logger().info(f"--- Elapsed Time: {elapsed_time:.3f} sec ---")
                
                raise KeyboardInterrupt

def main(args=None):
    rclpy.init(args=args)
    time_measurer_node = PolygonPassOppositeLineTimeMeasurer()
    try:
        rclpy.spin(time_measurer_node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            time_measurer_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
