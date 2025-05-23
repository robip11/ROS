# feleves/feleves/controller.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import SetPen, TeleportAbsolute
import math
import time # Hozzáadva a kisebb szünetekhez a szebb rajzolás érdekében

class ProportionalController(Node):
    def __init__(self):
        super().__init__('proportional_controller')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.subscription = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.pose_callback,
            10)
        self.subscription  # prevent unused variable warning

        self.set_pen_client = self.create_client(SetPen, '/turtle1/set_pen')
        # A service kliensek blokkoló várakozása, mielőtt bármit tennénk
        while not self.set_pen_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('set_pen service not available, waiting again...')

        self.teleport_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        while not self.teleport_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('teleport_absolute service not available, waiting again...')


        self.target_x = 0.0
        self.target_y = 0.0
        self.target_theta = None # A szögorientáció opcionális cél
        self.kp_linear = 1.5  # Arányos erősítés a lineáris sebességhez
        self.kp_angular = 5.0 # Arányos erősítés a szögsebességhez
        self.pose = Pose()
        self.has_target = False
        self.get_logger().info('ProportionalController node initialized.')

    def set_target(self, x, y, theta=None):
        self.target_x = float(x)
        self.target_y = float(y)
        self.target_theta = float(theta) if theta is not None else None
        self.has_target = True
        # self.get_logger().info(f'Cél beállítva: ({self.target_x}, {self.target_y}), theta: {self.target_theta}')

    def pose_callback(self, msg):
        self.pose = msg
        if self.has_target:
            self.control_loop()

    def control_loop(self):
        twist_msg = Twist()

        # Távolság a céltól (distance) és szög a céltól (angle_diff) kiszámítása
        distance = math.sqrt((self.target_x - self.pose.x)**2 + (self.target_y - self.pose.y)**2)
        angle_to_target = math.atan2(self.target_y - self.pose.y, self.target_x - self.pose.x)
        angle_diff = angle_to_target - self.pose.theta
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff)) # Szög normalizálása -pi és pi közé

        #self.get_logger().info(f'Cél: ({self.target_x:.2f}, {self.target_y:.2f}) Aktuális: ({self.pose.x:.2f}, {self.pose.y:.2f}) Távolság: {distance:.2f}, Szögkülönbség: {math.degrees(angle_diff):.2f} fok')

        # ELŐSZÖR: Ha a teknősnek fordulnia kell (és még nincs a célon), akkor csak forogjon!
        # Itt a feltétel: van jelentős szögeltérés ÉS még nincs a célon
        if abs(angle_diff) > 0.05 and distance > 0.1:
            twist_msg.angular.z = self.kp_angular * angle_diff
            # Korlátozzuk a maximális szögsebességet
            if twist_msg.angular.z > 2.0:
                twist_msg.angular.z = 2.0
            elif twist_msg.angular.z < -2.0:
                twist_msg.angular.z = -2.0
            twist_msg.linear.x = 0.0 # Fontos: lineáris sebesség nulla, amíg forog!
            self.publisher_.publish(twist_msg) # Azonnal publikáljuk a forgási parancsot
            return # Kilépünk a függvényből, nem hajtunk végre lineáris mozgást ebben a körben

        # MÁSODSZOR: Ha a szög már nagyjából jó, de még messze van a céltól, akkor haladjon előre
        elif distance > 0.1:
            twist_msg.linear.x = self.kp_linear * distance
            if twist_msg.linear.x > 2.0:
                twist_msg.linear.x = 2.0
            twist_msg.angular.z = 0.0 # Fontos: szögsebesség nulla, amíg előre halad!

        # HARMADSZOR: Ha a célpozíció elérve
        else:
            twist_msg.linear.x = 0.0
            twist_msg.angular.z = 0.0
            if self.target_theta is None: # Ha csak pontot célzunk
                self.has_target = False # Elértük a célt, nincs több aktív cél
                # self.get_logger().info(f'Cél elérve: ({self.target_x:.2f}, {self.target_y:.2f})')
            else: # Ha szögorientációt is célzunk (pl. fordulj 90 fokot az aktuális helyen)
                # self.get_logger().info(f'Pozíció elérve, forgatás a cél szögbe: {self.target_theta}')
                self.rotate_to_target_theta() # Hívjuk meg a forgató függvényt
                return # Fontos: ne publikáljuk a nulla sebességet, hanem a rotate_to_target_theta tegye meg

        # Végül publikáljuk a Twist üzenetet (ha nem történt korábbi return)
        self.publisher_.publish(twist_msg)
        #self.get_logger().info(f'Közzétett sebesség: lin_x={twist_msg.linear.x:.2f}, ang_z={twist_msg.angular.z:.2f}')

    def rotate_to_target_theta(self):
        twist_msg = Twist()
        angle_diff = self.target_theta - self.pose.theta
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))

        #self.get_logger().info(f'Forgatás cél szögbe: {self.target_theta:.2f}, Aktuális: {self.pose.theta:.2f}, Diff: {math.degrees(angle_diff):.2f} fok')

        if abs(angle_diff) > 0.05: # Ha még van korrigálandó szög
            twist_msg.angular.z = self.kp_angular * angle_diff
            if twist_msg.angular.z > 2.0:
                twist_msg.angular.z = 2.0
            elif twist_msg.angular.z < -2.0:
                twist_msg.angular.z = -2.0
        else: # Ha a cél szög elérve
            twist_msg.angular.z = 0.0
            self.has_target = False # Nincs több aktív cél
            #self.get_logger().info(f'Cél szög elérve: {self.target_theta}')

        self.publisher_.publish(twist_msg)
        #self.get_logger().info(f'Közzétett forgatási sebesség: ang_z={twist_msg.angular.z:.2f}')

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

# --- Fraktál rajzoláshoz szükséges függvények ---

def koch_curve(controller_node, order, length):
    """
    Rekurzívan rajzol egy Koch-görbét.
    :param controller_node: A ProportionalController példány.
    :param order: A fraktál rekurziós mélysége (0-tól kezdve).
    :param length: Az aktuális szakasz hossza.
    """
    if order == 0:
        # Alapeset: rajzolj egy egyenes vonalat (nincs további rekurzió)
        end_x = controller_node.pose.x + length * math.cos(controller_node.pose.theta)
        end_y = controller_node.pose.y + length * math.sin(controller_node.pose.theta)
        controller_node.set_target(end_x, end_y, controller_node.pose.theta) # Céltávolúság és a jelenlegi szög beállítása
        while controller_node.has_target: # Várj, amíg a teknős eléri ezt a pontot!
            rclpy.spin_once(controller_node, timeout_sec=0.01)
        time.sleep(0.01) # Kis szünet, hogy a Turtlesim és a ROS lépést tartson
    else:
        # Rekurzív hívások (négy kisebb szakaszra bontás)
        new_length = length / 3.0 # Az új szakasz hossza az előző harmada

        # 1. szakasz
        koch_curve(controller_node, order - 1, new_length)

        # 2. szakasz (Fordulás +60 fok, rajzolás)
        # A set_target most a jelenlegi pozíciót célozza, de új szöggel.
        # A control_loop erre az új szögbe fog fordulni.
        controller_node.set_target(controller_node.pose.x, controller_node.pose.y, controller_node.pose.theta + math.pi / 3) # Fordulj 60 fokot (PI/3 radián)
        while controller_node.has_target: # Várj, amíg a fordulás befejeződik!
            rclpy.spin_once(controller_node, timeout_sec=0.01)
        koch_curve(controller_node, order - 1, new_length)

        # 3. szakasz (Fordulás -120 fok, rajzolás)
        controller_node.set_target(controller_node.pose.x, controller_node.pose.y, controller_node.pose.theta - 2 * math.pi / 3) # Fordulj -120 fokot (-2*PI/3 radián)
        while controller_node.has_target: # Várj, amíg a fordulás befejeződik!
            rclpy.spin_once(controller_node, timeout_sec=0.01)
        koch_curve(controller_node, order - 1, new_length)

        # 4. szakasz (Fordulás +60 fok, rajzolás)
        controller_node.set_target(controller_node.pose.x, controller_node.pose.y, controller_node.pose.theta + math.pi / 3) # Fordulj 60 fokot (PI/3 radián)
        while controller_node.has_target: # Várj, amíg a fordulás befejeződik!
            rclpy.spin_once(controller_node, timeout_sec=0.01)
        koch_curve(controller_node, order - 1, new_length)


def draw_koch_snowflake(controller_node, order, size, start_x, start_y):
    """
    Rajzol egy teljes Koch-pelyhet.
    :param controller_node: A ProportionalController példány.
    :param order: A fraktál rekurziós mélysége.
    :param size: A pehely oldalainak hossza.
    :param start_x: A kezdő X koordináta.
    :param start_y: A kezdő Y koordináta.
    """
    controller_node.get_logger().info(f"Koch-pehely rajzolása. Rend: {order}, Méret: {size}")

    # Kezdő teleportálás és toll beállítása
    # A PI / 6 (30 fok) azért kell, hogy az alsó él vízszintes legyen
    controller_node.teleport(start_x, start_y, math.pi / 6)
    controller_node.set_pen(255, 255, 255, 2, False) # Fehér toll, vastagság 2, toll lent

    # Rajzoljuk a három Koch-görbét a pehelyhez
    for _ in range(3):
        koch_curve(controller_node, order, size) # Rajzold az aktuális oldalt
        # Fordulj 120 fokot (2*PI/3 radián) a következő oldalhoz
        controller_node.set_target(controller_node.pose.x, controller_node.pose.y, controller_node.pose.theta - 2 * math.pi / 3)
        while controller_node.has_target: # Várj, amíg ez a nagy fordulás befejeződik!
            rclpy.spin_once(controller_node, timeout_sec=0.01)

# --- Fő függvény ---
def main(args=None):
    rclpy.init(args=args)
    controller = ProportionalController()

    # Ezeket a paramétereket változtathatod!
    fractal_order = 2 # A fraktál rekurziós mélysége (0-tól kezdve). 2 vagy 3 jó kezdetnek.
    fractal_size = 6.0 # A fraktál alapjának hossza (Turtlesim egységben)
    start_pos_x = 2.0 # A kezdő X koordináta
    start_pos_y = 8.0 # A kezdő Y koordináta

    draw_koch_snowflake(controller, fractal_order, fractal_size, start_pos_x, start_pos_y)

    controller.get_logger().info("Rajzolás befejezve. Várakozás...")
    # Addig futtatjuk a node-ot, amíg manuálisan le nem állítjuk (Ctrl+C)
    # Ez fontos, hogy a rajzot láthassuk a végén
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        controller.get_logger().info("Controller leállítva a felhasználó által.")
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
