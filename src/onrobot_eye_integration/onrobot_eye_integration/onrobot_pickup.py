import rclpy
import argparse
from rclpy.node import Node
from std_msgs.msg import Bool
from .onrobot import RG
import time

# Robot ID és modell
ROBOT_ID   = "dsr01"
ROBOT_MODEL= "m1013"

def get_options():
        parser = argparse.ArgumentParser(description='Set options.')
        parser.add_argument('--gripper', dest='gripper', type=str, default="rg2", choices=['rg2', 'rg6'], help='Gripper type')
        parser.add_argument('--ip', dest='ip', type=str, default="192.168.1.1", help='IP address')
        parser.add_argument('--port', dest='port', type=str, default="502", help='Port number')
        args, _ = parser.parse_known_args()
        return args

class OnRobotEyeNode(Node):

        def __init__(self, options, dsr_functions):
                super().__init__('onrobot_eye_node', namespace=ROBOT_ID)

                self.movej = dsr_functions['movej']
                self.movel = dsr_functions['movel']
                self.posj = dsr_functions['posj']
                self.set_velx = dsr_functions['set_velx']
                self.set_accx = dsr_functions['set_accx']
                self.set_robot_mode = dsr_functions['set_robot_mode']
                self.ROBOT_MODE_AUTONOMOUS = dsr_functions['ROBOT_MODE_AUTONOMOUS']

                self.rg = RG(options.gripper, options.ip, options.port)
                self.subscription = self.create_subscription(Bool, '/onrobot_eye/object_detected', self.object_detected_callback, 10)
                self.inspect_pub = self.create_publisher(Bool, '/onrobot_eye/inspecting', 10)
                self.get_logger().info(f"OnRobot Eye Node elindítva.")
                self.is_busy = False
                self.start_timer = self.create_timer(1.0, self.start_sequence)

        # Kamera engedélyezés/tiltás
        def set_camera_inspection(self, enabled: bool):
                msg = Bool()
                msg.data = enabled
                self.inspect_pub.publish(msg)
                if enabled:
                        self.get_logger().info("Kamera engedélyezve")
                else:
                        self.get_logger().info("Kamera letiltva")

        # Elindulás
        def start_sequence(self):
                self.start_timer.cancel()
                try:
                        self.set_robot_mode(self.ROBOT_MODE_AUTONOMOUS)

                        # Globális sebesség és gyorsulás értékek
                        self.set_velx(30, 20) # (mm/sec) (deg/sec)
                        self.set_accx(60, 40) # (mm/sec^2) (deg/sec^2)

                        global p1
                        p1 = self.posj(-4.89, 1.78, 101.17, 3.36, 75.28, -4.83)

                        # Alaphelyzetbe állítás
                        self.rg.open_gripper()
                        time.sleep(1)
                        self.movej(p1, vel=100, acc=50)
                        time.sleep(0.5)

                        self.set_camera_inspection(True)

                except Exception as e:
                        self.get_logger().error(f"Hiba a kezdésnél: {e}")

        # Robot státusz ellenőrzése és kamera engedélyezés/tiltás vezérlése
        def object_detected_callback(self, msg):
                if msg.data and self.is_busy == False:
                        self.get_logger().info("Munkadarab észlelve!")
                        self.is_busy = True
                        self.set_camera_inspection(False)
                        self.perform_pickup()
                        self.set_camera_inspection(True)
                        self.is_busy = False

        # Pozíciók és mozgások
        def perform_pickup(self):
                try:
                        # Pozíciók felvétele
                        p0 = self.posj(0, 0, 0, 0, 0, 0)
                        p2 = self.posj(-4.29, 16.32, 116.44, 3.43, 45.66, -9.97)
                        p3 = self.posj(-183.03, 1.78, 101.17, 3.36, 75.28, -4.83)
                        asztal = self.posj(-51.67, 35.45, 76.34, 1.66, 65.37, -52.02)
                        
                        # Program
                        self.movej(p2, vel=100, acc=50)
                        time.sleep(0.5)
                        self.rg.close_gripper()
                        time.sleep(1)
                        self.movej(p1, vel=100, acc=50)
                        time.sleep(0.5)
                        self.movej(p3, vel=100, acc=50)
                        time.sleep(0.5)
                        # Ezután majd ráteszi a mobilrobotra

                        # Vissza az alaphelyzetbe
                        self.movej(p1, vel=100, acc=50)
                        time.sleep(0.5)

                        # Rakja a munkasztalomra
                        self.movej(asztal, vel=100, acc=50)
                        time.sleep(0.5)
                        self.rg.open_gripper()
                        time.sleep(1)
                        self.movej(p1, vel=100, acc=50)
                        time.sleep(0.5)

                except Exception as e:
                        self.get_logger().error(f"Hiba a munkadarab felvételénél: {e}")

def main(args=None):
        rclpy.init(args=args)
        options = get_options()
        dsr_base_node = Node('onrobot_eye_node', namespace=ROBOT_ID)

        import DR_init
        DR_init.__dsr__id   = ROBOT_ID
        DR_init.__dsr__model = ROBOT_MODEL
        DR_init.__dsr__node = dsr_base_node

        from DSR_ROBOT2 import print_ext_result, movej, movejx, movesj, movesx, movel, movec, move_periodic, move_spiral, moveb, set_velx, set_accx, set_robot_mode
        from DSR_ROBOT2 import posj, posx, posb
        from DSR_ROBOT2 import DR_LINE, DR_CIRCLE, DR_BASE, DR_TOOL, DR_AXIS_X, DR_AXIS_Z, DR_MV_MOD_ABS, ROBOT_MODE_AUTONOMOUS

        dsr_functions = {'movej': movej,'movel': movel, 'posj': posj, 'set_velx': set_velx, 'set_accx': set_accx, 
                        'set_robot_mode': set_robot_mode, 'ROBOT_MODE_AUTONOMOUS': ROBOT_MODE_AUTONOMOUS}
        eye_node = OnRobotEyeNode(options, dsr_functions)

        try:
                rclpy.spin(eye_node)
        except KeyboardInterrupt:
                pass
        finally:
                eye_node.destroy_node()
                rclpy.shutdown()

if __name__ == "__main__":
        main()