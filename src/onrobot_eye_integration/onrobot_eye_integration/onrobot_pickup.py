import rclpy
import argparse
from rclpy.node import Node
from std_msgs.msg import Bool, Int32, String
from .onrobot import RG
import time
from action_msgs.msg import GoalStatusArray, GoalStatus
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Pose2D
import os

# Robot ID és modell
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m1013"

def get_options():
        parser = argparse.ArgumentParser(description='Set options.')
        parser.add_argument('--gripper', dest='gripper', type=str, default="rg2", choices=['rg2', 'rg6'], help='Gripper type')
        parser.add_argument('--ip', dest='ip', type=str, default="192.168.10.1", help='IP address')
        parser.add_argument('--port', dest='port', type=str, default="502", help='Port number')
        args, _ = parser.parse_known_args()
        return args

class OnRobotEyeNode(Node):

        def __init__(self, options, dsr_functions):
                super().__init__('onrobot_pickup_node', namespace=ROBOT_ID)

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

                # Subscription-ök
                self.target_sub_pickup = self.create_subscription(Pose2D, '/onrobot_eye/target_pose_pickup', self.target_pose_pickup_callback, 10, callback_group=self.cgb)
                self.target_sub_unload = self.create_subscription(Pose2D, '/onrobot_eye/target_pose_unload', self.target_pose_unload_callback, 10, callback_group=self.cgb)
                self.workpiece_type_sub = self.create_subscription(String, '/onrobot_eye/workpiece_type', self.placed_workpiece_list, qos_status)
                self.mobilrobot_statemachine_sub = self.create_subscription(String, '/mobile_robot_status', self.mobilrobot_statemachine, qos_status)
                self.pose_sub = self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.nav_status_callback, qos_status, callback_group=self.cgb)

                # Publisher-ek
                self.inspect_pub = self.create_publisher(Bool, '/onrobot_eye/inspecting', 10)
                self.state_pub = self.create_publisher(Int32, '/onrobot_eye/robot_state', qos_status)
                
                # Változók alapértékeinek beállítása
                self.robot_state = 0
                self.pending_target_pickup = None
                self.pending_target_unload = None
                self.is_busy = False
                self.workpiece_type = None
                self.workpieces_on_mobilrobot = []
                self.next_workpiece = None
                self.statemachine = None
                self.last_status = None
                self.latest_status = None

                self.get_logger().info(f"OnRobot Eye Node elindítva.")

                self.ws_dir = os.path.expanduser('~')
                self.file_path = os.path.join(self.ws_dir, "kevin_ws", "camera_view.png")

                # Globális sebesség és gyorsulás értékek
                self.set_velx(30, 20) # (mm/sec), (deg/sec)
                self.set_accx(60, 40) # (mm/sec^2), (deg/sec^2)

                # Munkatér figyelő és munkadarab felvevő pozíciók
                self.rx_pickup_home = 167.43
                self.ry_pickup_home = 179.84
                self.z_pickup_home = 519.98
                self.z_pickup_pick = 241.0

                # Mobilrobot tetejét figyelő és munkadarab felvevő pozíciók
                self.rx_unload_home = 3.48
                self.ry_unload_home = -179.39
                self.z_unload_home = 162.1
                self.z_unload_pick = -113

                # Pozíciók
                self.factory_home = self.posj(0, 0, 0, 0, 0, 0)
                self.robot_home = self.posj(-12.57, -7.36, 110.43, 0, 77.08, -104.77)
                self.turn_around = self.posj(-183.03, 1.78, 101.17, 3.36, 75.28, -180)
                self.red_square_delivery_space = self.posj(-174.2, 81.44, 34.09, -0.65, 64.54, -170.99)
                self.red_hexagon_delivery_space = self.posj(-173.85, 73.89, 51.13, -0.72, 55.06, -170.5)
                self.red_circle_delivery_space = self.posj(-173.44, 68.32, 64.2, -0.8, 47.57, -169.97)
                self.prepare_to_unload = self.posj(-176.47, 71.46, 25.34, 0, 82.57, -268.15)
                self.final_destination_prepare = self.posj(-90, -0.36, 110.44, 0, 77.08, -104.77)
                self.final_destination = self.posj(-55.65, 32.28, 84.17, 0, 64.08, -190)

                # Mobilrobot állapotok
                self.status_names = {
                GoalStatus.STATUS_UNKNOWN: 'UNKNOWN (0)',
                GoalStatus.STATUS_ACCEPTED: 'ACCEPTED (1)',
                GoalStatus.STATUS_EXECUTING: 'EXECUTING (2) - Úton van',
                GoalStatus.STATUS_CANCELING: 'CANCELING (3)',
                GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED (4) - MEGÉRKEZETT!',
                GoalStatus.STATUS_CANCELED: 'CANCELED (5)',
                GoalStatus.STATUS_ABORTED: 'ABORTED (6) - Hiba/Megszakadt',
                }

                self.start_sequence()

        # Mobilrobot állapotának átadása változónak
        def nav_status_callback(self, msg: GoalStatusArray):
                if msg.status_list:
                        self.latest_status = msg.status_list[-1].status
                        if self.latest_status != self.last_status:
                                self.last_status = self.latest_status
                                if self.latest_status == GoalStatus.STATUS_SUCCEEDED:
                                        self.get_logger().info("A mobilrobot elérte a célpontot.")
                        self.state_switch_to_3()

        # Mobilrobot állapotgép stádiuma
        def mobilrobot_statemachine(self, msg: String):
                self.statemachine = msg.data
                self.state_switch_to_3()
                if self.statemachine == 'DOCKING_STAGE_1':
                        self.publish_state(0)

        # Robotkar állapot közlés
        def publish_state(self, robot_state):
                msg = Int32()
                msg.data = robot_state
                self.state_pub.publish(msg)
                self.robot_state = robot_state

        # Mobilrobotra lerakott munkadarabot eltároljuk egy listában
        def placed_workpiece_list(self, msg: String):
                self.workpiece_type = msg.data
                if self.robot_state < 2 and len(self.workpieces_on_mobilrobot) < 3 and self.workpiece_type:
                        self.workpieces_on_mobilrobot.append(self.workpiece_type)
                        self.get_logger().info(f'\nRakomány: {self.workpieces_on_mobilrobot}')
                                        
        # Mobilrobotról leszedett munkadarabokat kiszedjük a listából
        def remove_workpiece_from_list(self): 
                if len(self.workpieces_on_mobilrobot) > 0:
                        self.get_logger().info(f'\nMunkdarab felvevétel: {self.next_workpiece}')
                        self.next_workpiece = self.workpieces_on_mobilrobot.pop(0)
                        self.get_logger().info(f'\nRakomány: {self.workpieces_on_mobilrobot}')
                        return self.next_workpiece
                return None
                        
        # Kamera engedélyezés/tiltás, és visszajelzés
        def set_camera_inspection(self, enabled: Bool):
                msg = Bool()
                msg.data = enabled
                self.inspect_pub.publish(msg)
                if enabled:
                        self.get_logger().info("Kamera engedélyezve")
                else:
                        self.get_logger().info("Kamera letiltva")

        # Alaphelyzetbe állítás elindításkor
        def start_sequence(self):
                try:
                        self.set_robot_mode(self.ROBOT_MODE_AUTONOMOUS)

                        if os.path.exists(self.file_path):
                                os.remove(self.file_path)
                        self.movej(self.factory_home, vel=20, acc=10)
                        self.publish_state(0)
                        self.movej(self.robot_home, vel=100, acc=50)
                        time.sleep(1)
                        self.rg.open_gripper()
                        time.sleep(1)
                        self.set_camera_inspection(True)
                        time.sleep(4)

                except Exception as e:
                        self.get_logger().error(f"Hiba a kezdésnél: {e}")

        # Munkaasztalon érzékelt munkadarabok koorindinátáinak átadása változónak
        def target_pose_pickup_callback(self, msg: Pose2D):
                self.pending_target_pickup = msg
                if self.is_busy == False:
                        self.is_busy = True
                        self.state_switch_to_2()

        # Mobilroboton érzékelt munkadarabok koorindinátáinak átadása változónak
        def target_pose_unload_callback(self, msg: Pose2D):
                self.pending_target_unload = msg
                if self.is_busy == False:
                        self.is_busy = True
                        self.state_switch_to_3()

        # Robot állapotának átváltása 1 -> 2
        def state_switch_to_2(self):
                if self.robot_state < 2:
                        self.publish_state(1)

                        while True:
                                target = self.pending_target_pickup
                                self.pending_target_pickup = None

                                if target is not None:
                                        self.set_camera_inspection(False)
                                        self.perform_pickup(target.x, target.y, target.theta)
                                        self.set_camera_inspection(True)

                                state_1_countdown = time.time()
                                self.new_target = False

                                while time.time() - state_1_countdown < 10:
                                        if self.pending_target_pickup is not None:
                                                self.new_target = True
                                                break
                                        time.sleep(0.2)

                                if self.new_target:
                                        continue
                                else:
                                        self.publish_state(2)
                                        self.is_busy = False
                                        break

        # Robot állapotának átváltása 2 -> 3
        def state_switch_to_3(self):
                if self.robot_state == 2 and self.latest_status == GoalStatus.STATUS_SUCCEEDED and self.statemachine == 'RETURN_TO_PICKUP':
                        self.publish_state(3)

                        self.set_camera_inspection(False)
                        self.movej(self.turn_around, vel=100, acc=50)
                        time.sleep(0.5)
                        self.movej(self.prepare_to_unload, vel=10, acc=5)
                        time.sleep(0.5)
                        if os.path.exists(self.file_path):
                                os.remove(self.file_path)
                        self.set_camera_inspection(True)
                        time.sleep(4)

                        target = self.pending_target_unload
                        self.pending_target_unload = None

                        if target is not None:
                                self.set_camera_inspection(False)
                                self.perform_unload(target.x, target.y, target.theta)
                                self.set_camera_inspection(True)

        # Munkadarabok felvétele és lepakolása a mobilrobotra
        def perform_pickup(self, x_target, y_target, rz_target):
                try:
                        # Munkadarab felvételi pozíciók meghatározása
                        above_workpiece = self.posx(x_target, y_target, self.z_pickup_home, self.rx_pickup_home, self.ry_pickup_home, rz_target)
                        pickup_workpiece = self.posx(x_target, y_target, self.z_pickup_pick, self.rx_pickup_home, self.ry_pickup_home, rz_target)

                        # Elemgyünk a megtalált munkadarab fölé
                        self.movel(above_workpiece, vel=100, acc=50)
                        time.sleep(0.5)

                        # Töröljük a képet, hogy ne legyenek félrevezető információk
                        if os.path.exists(self.file_path):
                                os.remove(self.file_path)

                        # Lemegyünk érte és megfogjuk
                        self.movel(pickup_workpiece, vel=100, acc=50)
                        time.sleep(0.5)
                        self.rg.close_gripper()
                        time.sleep(1)

                        # Felemeljük a munkadarabot és átvisszük a másik oldalra
                        self.movel(above_workpiece, vel=100, acc=50)
                        time.sleep(0.5)
                        self.movej(self.turn_around, vel=100, acc=50)
                        time.sleep(0.5)
                        
                        # Ha a mobilrobot nem éri el a pakolópontot 60s-en bellül, akkor visszafordulunk
                        waiting_for_mobilrobot = time.time()
                        while self.latest_status != GoalStatus.STATUS_SUCCEEDED:
                                self.get_logger().info(f"Legutóbbi státusz: {self.latest_status}")
                                time.sleep(0.2)
                                if time.time() - waiting_for_mobilrobot >= 60:
                                        self.get_logger().warn("A mobilrobot nem érkezett meg a célpontra a várakozási időn belül.")
                                        break
                                        
                        # Ha a mobilrobot elérte a pakolópontot, akkor rárakjuk a munkadarabokat a megfelelő helyeikre
                        if self.robot_state == 1 and self.latest_status == GoalStatus.STATUS_SUCCEEDED:
                                place_down_position = None
                                
                                if self.workpiece_type == 'Piros négyzet':
                                        place_down_position = self.red_square_delivery_space
                                elif self.workpiece_type == 'Piros hatszög':
                                        place_down_position = self.red_hexagon_delivery_space
                                elif self.workpiece_type == 'Piros kör':
                                        place_down_position = self.red_circle_delivery_space

                                if place_down_position is not None:
                                        self.movej(place_down_position, vel=10, acc=5)
                                        time.sleep(0.5)
                                        self.rg.open_gripper()
                                        time.sleep(1)
                                        self.movej(self.turn_around, vel=10, acc=5)
                                        time.sleep(0.5)
                                        place_down_position = None
                        
                        # Visszatérünk a kiidnuló pontba
                        self.movej(self.robot_home, vel=100, acc=50)
                        time.sleep(0.5)
                        self.rg.open_gripper()
                        time.sleep(1)

                except Exception as e:
                        self.get_logger().error(f"Hiba a munkadarab felvételénél: {e}")

        # Munkadarabok lepakolása a mobilrobotról
        def perform_unload(self, x_target_unload, y_target_unload, rz_target_unload):
                try:
                        if len(self.workpieces_on_mobilrobot) > 0:
                                
                                self.next_workpiece = self.remove_workpiece_from_list()

                                # Munkadarab felvételi pozíciók meghatározása
                                above_workpiece = self.posx(x_target_unload, y_target_unload, self.z_unload_home, self.rx_unload_home, self.ry_unload_home, rz_target_unload)
                                pickup_workpiece = self.posx(x_target_unload, y_target_unload, self.z_unload_pick, self.rx_unload_home, self.ry_unload_home, rz_target_unload)

                                # Elemgyünk a megtalált munkadarab fölé
                                self.movel(above_workpiece, vel=100, acc=50)
                                time.sleep(0.5)

                                # Töröljük a képet, hogy ne legyenek félrevezető információk
                                if os.path.exists(self.file_path):
                                        os.remove(self.file_path)
        
                                # Lemegyünk érte és megfogjuk
                                self.movel(pickup_workpiece, vel=100, acc=50)
                                time.sleep(0.5)
                                self.rg.close_gripper()
                                time.sleep(1)
        
                                # Felemeljük a munkadarabot és átvisszük a lerakóhelyre
                                self.movel(above_workpiece, vel=100, acc=50)
                                time.sleep(0.5)
                                self.movej(self.final_destination_prepare, vel=100, acc=50)
                                time.sleep(0.5)
                                self.movej(self.final_destination, vel=50, acc=25)
                                time.sleep(0.5)
                                self.rg.open_gripper()
                                time.sleep(1)

                        self.movej(self.robot_home, vel=50, acc=25)
                        time.sleep(0.5)
                        self.publish_state(2)

                except Exception as e:
                        self.get_logger().error(f"Hiba a munkadarab lepakolásánál: {e}")
                finally:
                        self.is_busy = False

def main(args=None):
        rclpy.init(args=args)
        options = get_options()
        dsr_base_node = Node('dsr_base_node', namespace=ROBOT_ID)

        import DR_init
        DR_init.__dsr__id   = ROBOT_ID
        DR_init.__dsr__model = ROBOT_MODEL
        DR_init.__dsr__node = dsr_base_node

        from DSR_ROBOT2 import (
                movej, movel, set_velx, set_accx,
                set_robot_mode, posj, posx,
                ROBOT_MODE_AUTONOMOUS
        )

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
                
                if os.path.exists(eye_node.file_path):
                        os.remove(eye_node.file_path)
                eye_node.destroy_node()
                if rclpy.ok():
                        rclpy.shutdown()

if __name__ == "__main__":
        main()