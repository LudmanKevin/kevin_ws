#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
import socketio
import requests
import threading

class OnRobotEyeNode(Node):
    def __init__(self):
        super().__init__('onrobot_eye_node')
        
        self.declare_parameter('compute_box_ip', '192.168.1.1')
        
        # Munkadarabok hozzárendelése
        self.task_mapping = {
            #: "Piros kör",
            #: "Piros hatszög",
            8: "Piros négyzet",
            #: "Piros háromszög",
        }
        
        # Kinyerjük a Task ID-kat a szótárból
        self.task_uids = list(self.task_mapping.keys())
        self.ip = self.get_parameter('compute_box_ip').value
        self.current_task_index = 0

        # ROS 2 Publisher
        self.detection_pub = self.create_publisher(Bool, '/onrobot_eye/object_detected', 10)

        self.is_processing = False

        # Socket.IO Kliens
        self.sio = socketio.Client(reconnection=True, reconnection_attempts=5)
        self.setup_socketio()

        # Időzítő
        self.timer = self.create_timer(1.0, self.trigger_inspection)
        self.get_logger().info(f"OnRobot Eye Node elindítva. IP: {self.ip}")
        self.get_logger().info(f"Regisztrált munkadarabok: {self.task_mapping}")

    def setup_socketio(self):
        @self.sio.on('connect')
        def on_connect():
            self.get_logger().info("Sikeresen csatlakozva az OnRobot Socket.IO szerveréhez!")

        @self.sio.on('message')
        def on_message(data):
            if isinstance(data, dict):
                try:
                    computebox_data = data.get('computebox', {})
                    variables = computebox_data.get('variable', {})
                    d_output = variables.get('dOutput', [])

                    if len(d_output) > 0:
                        has_object = bool(d_output[0])
                        
                        # Megkeressük az aktuális ID-hoz tartozó nevet
                        active_uid = self.task_uids[self.current_task_index]
                        active_name = self.task_mapping.get(active_uid, f"Ismeretlen_ID_{active_uid}")
                        
                        msg = Bool()
                        msg.data = has_object
                        self.detection_pub.publish(msg)

                        if has_object:
                            self.get_logger().info(f"'{active_name}' érzékelve.")
                        else:
                            self.get_logger().info(f" Nincs '{active_name}' érzékelve.")
                except Exception as e:
                    self.get_logger().error(f"Hiba a dOutput feldolgozásakor: {e}")

        try:
            self.sio.connect(
                f'http://{self.ip}', 
                socketio_path='socket.io',
                transports=['websocket', 'polling']
            )
        except Exception as e:
            self.get_logger().error(f"Socket.IO csatlakozási hiba: {e}")

    def _async_post(self, task_uid):
        """Aszinkron mérésindítás a megadott task_uid-ra"""
        run_url = f"http://{self.ip}/api/dc/vision/runProcess"
        payload = {"numericUid": task_uid}
        headers = {"Content-Type": "application/json"}
        try:
            requests.post(run_url, json=payload, headers=headers, timeout=5.0)
        except Exception as e:
            self.get_logger().warn(f"Mérés indítási hiba (UID {task_uid}): {e}")
        finally:
            self.is_processing = False

    def trigger_inspection(self):
        """Következő munkadarab mérésének elindítása ciklikusan"""
        if not self.is_processing and len(self.task_uids) > 0:
            self.is_processing = True
            
            target_uid = self.task_uids[self.current_task_index]
            target_name = self.task_mapping.get(target_uid, f"ID_{target_uid}")
            
            self.get_logger().info(f"Mérés indítása -> Munkadarab: '{target_name}' (ID: {target_uid})")
            
            # Léptetjük az indexet a következőre
            self.current_task_index = (self.current_task_index + 1) % len(self.task_uids)
            
            threading.Thread(target=self._async_post, args=(target_uid,), daemon=True).start()

    def destroy_node(self):
        if self.sio.connected:
            self.sio.disconnect()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = OnRobotEyeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()