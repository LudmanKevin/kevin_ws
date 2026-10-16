import rclpy
import argparse
from rclpy.node import Node
from std_msgs.msg import Bool
from .onrobot import RG
import time
from action_msgs.msg import GoalStatusArray, GoalStatus
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose2D

# Robot ID és modell
ROBOT_ID   = "dsr01"
ROBOT_MODEL= "m1013"

def get_options():
        parser = argparse.ArgumentParser(description='Set options.')
        parser.add_argument('--gripper', dest='gripper', type=str, default="rg2", choices=['rg2', 'rg6'], help='Gripper type')
        parser.add_argument('--ip', dest='ip', type=str, default="192.168.10.1", help='IP address')
        parser.add_argument('--port', dest='port', type=str, default="502", help='Port number')
        args, _ = parser.parse_known_args()
        return args

class OnRobotEyeNode(Node):

        def __init__(self, options, dsr_functions):
                super().__init__('onrobot_eye_node', namespace=ROBOT_ID)

                self.cgb = ReentrantCallbackGroup()
                qos_status = QoSProfile(
                        reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST,
                        depth=10)

                self.movej = dsr_functions['movej']
                self.movel = dsr_functions['movel']
                self.posj = dsr_functions['posj']
                self.posx = dsr_functions['posx']
                self.set_velx = dsr_functions['set_velx']
                self.set_accx = dsr_functions['set_accx']
                self.set_robot_mode = dsr_functions['set_robot_mode']
                self.ROBOT_MODE_AUTONOMOUS = dsr_functions['ROBOT_MODE_AUTONOMOUS']

                self.rg = RG(options.gripper, options.ip, options.port)
                #self.subscription = self.create_subscription(Bool, '/onrobot_eye/object_detected', self.object_detected_callback, 10) # Mozgás kikapcsolás teszthez
                self.target_sub = self.create_subscription(Pose2D, '/target_pose', self.target_pose_callback, 10, callback_group=self.cgb)
                self.inspect_pub = self.create_publisher(Bool, '/onrobot_eye/inspecting', 10)
                self.start_timer = self.create_timer(1.0, self.start_sequence)
                self.is_busy = False
                self.get_logger().info(f"OnRobot Eye Node elindítva.")
                
                self.pose_sub = self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.nav_status_callback, qos_profile=qos_status, callback_group=self.cgb)
                self.last_status = None
                self.latest_status = None

                self.status_names = {
                GoalStatus.STATUS_UNKNOWN: 'UNKNOWN (0)',
                GoalStatus.STATUS_ACCEPTED: 'ACCEPTED (1)',
                GoalStatus.STATUS_EXECUTING: 'EXECUTING (2) - Úton van',
                GoalStatus.STATUS_CANCELING: 'CANCELING (3)',
                GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED (4) - MEGÉRKEZETT!',
                GoalStatus.STATUS_CANCELED: 'CANCELED (5)',
                GoalStatus.STATUS_ABORTED: 'ABORTED (6) - Hiba/Megszakadt',
                }

        def nav_status_callback(self, msg: GoalStatusArray):
                if msg.status_list:
                        self.latest_status = msg.status_list[-1].status

                        if self.latest_status != self.last_status:
                                self.last_status = self.latest_status
                                if self.latest_status == GoalStatus.STATUS_SUCCEEDED:
                                        self.get_logger().info("A mobilrobot elérte a célpontot.")

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
                        self.set_velx(30, 20) # (mm/sec), (deg/sec)
                        self.set_accx(60, 40) # (mm/sec^2), (deg/sec^2)

                        global robot_home
                        robot_home = self.posj(-12.57, -7.36, 110.43, 0, 77.08, -104.77)

                        # Alaphelyzetbe állítás
                        self.movej(robot_home, vel=100, acc=50)
                        time.sleep(1)
                        self.rg.open_gripper()
                        time.sleep(1)
                        self.set_camera_inspection(True)
                        time.sleep(4)

                except Exception as e:
                        self.get_logger().error(f"Hiba a kezdésnél: {e}")

        def target_pose_callback(self, msg: Pose2D):
                if self.is_busy == False:
                        self.is_busy = True
                        self.get_logger().info(f"Új célpozíció érkezett: X={msg.x:.2f} mm, Y={msg.y:.2f} mm, Rz={msg.theta:.2f}°")
                        self.set_camera_inspection(False)
                        self.perform_pickup(msg.x, msg.y, msg.theta)
                        self.set_camera_inspection(True)
                        self.is_busy = False
        
        # Pozíciók és mozgások
        def perform_pickup(self, x_target, y_target, rz_target):
                try:
                        # Pozíciók felvétele
                        rx_base = 167.43
                        ry_base = 179.84

                        z_home = 519.98
                        z_pick = 246.0

                        above_workpiece = self.posx(x_target, y_target, z_home, rx_base, ry_base, rz_target)
                        pickup_workpiece = self.posx(x_target, y_target, z_pick, rx_base, ry_base, rz_target)
                        
                        prepare_to_place = self.posj(-183.03, 1.78, 101.17, 3.36, 75.28, -180)
                        place_down = self.posj(-172.98, 61.46, 81.72, 5.62, 36.46, -180.42)

                        # Elemgyünk a megtalált munkadarab fölé
                        self.movel(above_workpiece, vel=100, acc=50)
                        time.sleep(0.5)

                        # Lemegyünk érte és megfogjuk
                        self.movel(pickup_workpiece, vel=100, acc=50)
                        time.sleep(0.5)
                        self.rg.close_gripper()
                        time.sleep(1)

                        # Felemeljük a munkadarabot és átvisszük a másik oldalra
                        self.movel(above_workpiece, vel=100, acc=50)
                        time.sleep(0.5)
                        self.movej(prepare_to_place, vel=100, acc=50)
                        time.sleep(0.5)

                        # Ha a mobilrobot nem éri el a pakolópontot 60s-en bellül, akkor visszafordulunk
                        start_time = time.time()
                        while self.latest_status != GoalStatus.STATUS_SUCCEEDED:
                                self.get_logger().info(f"Legutóbbi státusz: {self.latest_status}")
                                time.sleep(0.5)
                                if time.time() - start_time > 60:
                                        self.get_logger().warn("A mobilrobot nem érkezett meg a célpontra a várakozási időn belül.")
                                        break

                        # Ha a mobilrobot elérte a pakolópontot, akkor rárakjuk a munkadarabot
                        if self.latest_status == GoalStatus.STATUS_SUCCEEDED:
                                self.movej(place_down, vel=10, acc=5)
                                time.sleep(0.5)
                                self.rg.open_gripper()
                                time.sleep(1)
                                self.movej(prepare_to_place, vel=10, acc=5)
                                time.sleep(0.5)

                        # Visszatérünk a kiidnuló pontba
                        self.movej(robot_home, vel=100, acc=50)
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

        dsr_functions = {'movej': movej,'movel': movel, 'posj': posj, 'posx': posx, 'set_velx': set_velx, 'set_accx': set_accx, 
                        'set_robot_mode': set_robot_mode, 'ROBOT_MODE_AUTONOMOUS': ROBOT_MODE_AUTONOMOUS}
        eye_node = OnRobotEyeNode(options, dsr_functions)

        executor = MultiThreadedExecutor()
        executor.add_node(eye_node)

        try:
                executor.spin()
        except KeyboardInterrupt:
                pass
        finally:
                eye_node.destroy_node()
                if rclpy.ok():
                        rclpy.shutdown()

if __name__ == "__main__":
        main()