# ... (a ProportionalController osztály definíciója fentebb)
import rclpy                  # EZ A SOR HIÁNYZIK VAGY ROSSZUL VAN
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen, TeleportAbsolute
import math

class ProportionalController(Node):
    def __init__(self):
        super().__init__('proportional_controller')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10)
        self.subscription

        self.set_pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        while not self.set_pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set_pen service not available, waiting again...')

        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('teleport_absolute service not available, waiting again...')

        self.target_x = 0.0
        self.target_y = 0.0
        self.target_theta = None
        self.kp_linear = 1.5
        self.kp_angular = 5.0
        self.pose = Pose()
        self.has_target = False
        self.get_logger().info('ProportionalController node initialized.') # Debug üzenet

    def set_target(self, x, y, theta=None):
        self.target_x = float(x)
        self.target_y = float(y)
        self.target_theta = float(theta) if theta is not None else None
        self.has_target = True
        self.get_logger().info(f'Cél beállítva: ({self.target_x}, {self.target_y}), theta: {self.target_theta}')

    def pose_callback(self, msg):
        self.pose = msg
        if self.has_target:
            self.control_loop()
        # else: # Ideiglenesen kikapcsolva, ha sok outputot generálna
        #     self.get_logger().info('Nincs aktív cél, nem fut a control_loop.')

    def control_loop(self):
        twist_msg = Twist()

        distance = math.sqrt((self.target_x - self.pose.x)**2 + (self.target_y - self.pose.y)**2)
        angle_to_target = math.atan2(self.target_y - self.pose.y, self.target_x - self.pose.x)
        angle_diff = angle_to_target - self.pose.theta
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

        self.get_logger().info(f'Cél: ({self.target_x:.2f}, {self.target_y:.2f}) Aktuális: ({self.pose.x:.2f}, {self.pose.y:.2f}) Távolság: {distance:.2f}, Szögkülönbség: {math.degrees(angle_diff):.2f} fok')

        if distance > 0.1:
            twist_msg.linear.x = self.kp_linear * distance
            if twist_msg.linear.x > 2.0:
                twist_msg.linear.x = 2.0
            self.get_logger().info(f'Számított lineáris sebesség: {twist_msg.linear.x:.2f}')
        else:
            twist_msg.linear.x = 0.0
            if self.target_theta is None:
                self.has_target = False
                self.get_logger().info(f'Cél elérve: ({self.target_x}, {self.target_y})')
            else:
                self.get_logger().info(f'Pozíció elérve, forgatás a cél szögbe: {self.target_theta}')
                self.rotate_to_target_theta()
                return

        if abs(angle_diff) > 0.05:
            twist_msg.angular.z = self.kp_angular * angle_diff
            if twist_msg.angular.z > 2.0:
                twist_msg.angular.z = 2.0
            elif twist_msg.angular.z < -2.0:
                twist_msg.angular.z = -2.0
            self.get_logger().info(f'Számított szögsebesség: {twist_msg.angular.z:.2f}')
        else:
            twist_msg.angular.z = 0.0
            self.get_logger().info('Cél szög közelében.')

        self.publisher_.publish(twist_msg)
        self.get_logger().info(f'Közzétett sebesség: lin_x={twist_msg.linear.x:.2f}, ang_z={twist_msg.angular.z:.2f}')


    def rotate_to_target_theta(self):
        twist_msg = Twist()
        angle_diff = self.target_theta - self.pose.theta
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

        self.get_logger().info(f'Forgatás cél szögbe: {self.target_theta:.2f}, Aktuális: {self.pose.theta:.2f}, Diff: {math.degrees(angle_diff):.2f} fok')

        if abs(angle_diff) > 0.05:
            twist_msg.angular.z = self.kp_angular * angle_diff
            if twist_msg.angular.z > 2.0:
                twist_msg.angular.z = 2.0
            elif twist_msg.angular.z < -2.0:
                twist_msg.angular.z = -2.0
        else:
            twist_msg.angular.z = 0.0
            self.has_target = False
            self.get_logger().info(f'Cél szög elérve: {self.target_theta}')

        self.publisher_.publish(twist_msg)
        self.get_logger().info(f'Közzétett forgatási sebesség: ang_z={twist_msg.angular.z:.2f}')

    def set_pen(self, r, g, b, width, off):
        req = SetPen.Request()
        req.r = r
        req.g = g
        req.b = b
        req.width = width
        req.off = off
        future = self.set_pen_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def teleport(self, x, y, theta):
        req = TeleportAbsolute.Request()
        req.x = float(x)
        req.y = float(y)
        req.theta = float(theta)
        future = self.teleport_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()
        

def main(args=None):
    rclpy.init(args=args)
    controller = ProportionalController()

    # Teleportálás és toll beállítása
    controller.teleport(2.0, 8.0, 0.0)
    controller.set_pen(255, 255, 255, 2, False)

    controller.get_logger().info("Teleportálás és toll beállítva. Kezdődik a rajzolás.")

    text_waypoints = [
        (2.0, 8.0, True),
        (2.0, 4.0, True),
        (2.0, 6.0, False),
        (4.0, 6.0, True),
        (4.0, 8.0, False),
        (4.0, 4.0, True),
    ]

    for i, (x, y, pen_down) in enumerate(text_waypoints):
        controller.get_logger().info(f"Feldolgozandó waypoint {i}: ({x}, {y}), toll lent: {pen_down}")
        controller.set_pen(255, 255, 255, 2, not pen_down)
        controller.set_target(x, y, None)

        controller.get_logger().info(f"Cél beállítva. Várakozás, amíg eléri a célt (has_target: {controller.has_target})")

        while controller.has_target:
            rclpy.spin_once(controller, timeout_sec=0.1)

        controller.get_logger().info(f"Waypoint {i} elérve.")


    controller.destroy_node()
    rclpy.shutdown()

# EZ A RÉSZ KULCSFONTOSSÁGÚ:
if __name__ == '__main__':
    main()
