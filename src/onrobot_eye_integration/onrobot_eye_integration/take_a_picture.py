import socketio
import json
import base64
import cv2
import numpy as np
import os
from .process_the_picture import WorkpieceDetector
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# Onrobot IP cím és süti
ip = "192.168.10.1"
cookie_token = "eyJhbGciOiJSUzI1NiIsInR5cCI6IkpXVCJ9.eyJzdWIiOjAsInJvbGU" \
"iOjAsImlhdCI6MTc4OTA0OTIyMCwiZXhwIjoxODIwNTg1MjIwfQ.UF-DXb7Ia4Wf8fujQZ" \
"pZZS4yhHi8UhmKyq1cJADCfxftc29WwmaZcX28QTG_6RKQpmDir5_y6h31b4w_WDt80joY2" \
"bo34vUzUGaBh8yMCinLPNok07dQVophAEMDNIxx__lN3WGriNxcCUB7kUZ1iu9AWs1Xbq5" \
"114pQNLhJcvlcSqI47OxdPgYUr8XQOhaUMx5fLt6EqqjDhWRDIU0bHoCLq3VqFR_1Qwx90" \
"3dFWl-PNksaxXq20KVefwOsy-ym_rorZtOf4mGOao8WvPZUVgdmMcOfbqXDPfMbAZ75w28" \
"tm91GrIJt1vGUPwb-htI9oLwEUMG9t15l9lullzzoRf3En5XqXSyXm7kt6F_1SO-9jRjcU" \
"_pA2eTDRHRVPF958jzbcuj4Z9lskJULhHgYyXsg8Vexn1xzt2SyLAWQLt7zeuryru2n6IU" \
"XRuNW_v-y-soOAe6p3q6K6uRe0ppVv0I_KrY6s6oaE11wGuLzgVRufWlcaVrA922-mYcJ5" \
"EadU81vhcDAOZpNyGihNSJFgT81r-DNotumsYL8KoFl24RIMbj1RHE1aUfS-t5kG6ONyi" \
"eaV4f3hf9oQWmZDQ5ZmcHV1SMMO5u2u2e02P3GCSVydkMfU61BUetMOOxwjrJBL2DXSY3" \
"W_UuVsL5sf8LW-9aLok4_aFX-TTJU3Gh3KL0"

Workpieces = {
    "EFAB2314-4517-4389-9471-D1A4F51EEAA4": "Piros négyzet",
    "80EA452B-B1AE-4D99-BB6A-D32FB90087A5": "Piros hatszög",
    "42DC9A21-30F3-40A4-8F3E-1C0F0998C10B": "Piros kör",
    "862F2411-AF1E-42BC-B5B6-D630C817FA9C": "Piros négyzet",
    "93DA946D-2DA6-40A8-B9C2-4D4CAFEF900D": "Piros hatszög",
    "600C3622-0EC9-48B5-8F12-02D7A49814D6": "Piros kör"
}

class PictureInfoNode(Node):
    def __init__(self):
        super().__init__('picture_info_node')

        qos_status = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.pose_publisher_for_pickup = self.create_publisher(Pose2D, '/onrobot_eye/target_pose_pickup', 10)
        self.pose_publisher_for_unload = self.create_publisher(Pose2D, '/onrobot_eye/target_pose_unload', 10)
        self.workpiece_type_pub = self.create_publisher(String, '/onrobot_eye/workpiece_type', qos_status)
        self.detector = WorkpieceDetector(
            mm_per_pixel_x = 0.0677,
            mm_per_pixel_y = 0.0666,
            x_home = 461.46,
            y_home = -66.01,
            z_home = 519.98,
            x_unload = -1139.89,
            y_unload = -104.98,
            z_unload = 162.08,
            rz_unload = -88.23
        )

        self.sio = socketio.Client()
        self.socket_events()

    def socket_events(self):
        @self.sio.event
        def connect():
            print("Socket.IO csatlakozás sikeres!")
            try:
                # Vision szoba, ahonnan a képek jönnek
                self.sio.emit('vision')
                print("Feliratkozva a vision csatornára.")
            except Exception as e:
                print("Hiba a feliratkozásnál:", e)

        @self.sio.on('vision_message')
        def on_vision_message(data):
            print("\n[ESEMÉNY]: 'vision_message' érkezett!")
            self.extract_and_save_image(data)

    def start_socket(self):
        # Webszerver
        self.sio.connect(
            f'http://{ip}', 
            headers={'Cookie': f'jwt={cookie_token}'}, 
            transports=['websocket', 'polling']
            )
        # Folyamatos eseményfigyelés
        self.sio.wait()

    def publish_target_for_pickup(self, x_target, y_target, angle_deg):
        msg = Pose2D()
        msg.x = float(x_target)
        msg.y = float(y_target)
        msg.theta = float(angle_deg)
        self.pose_publisher_for_pickup.publish(msg)

    def publish_target_for_unload(self, x_target_unload, y_target_unload, rz_target_unload):
        msg = Pose2D()
        msg.x = float(x_target_unload)
        msg.y = float(y_target_unload)
        msg.theta = float(rz_target_unload)
        self.pose_publisher_for_unload.publish(msg)

    def publish_workpiece_name(self, name_str):
        msg = String()
        msg.data = name_str
        self.workpiece_type_pub.publish(msg)

    def extract_and_save_image(self, raw_data):
        try:
            if isinstance(raw_data, str):
                try:
                    raw_data = json.loads(raw_data)
                except Exception as e:
                    print("JSON parse hiba: {e}")

            # Munkadarab keresés
            if isinstance(raw_data, dict):
                numeric_id = raw_data.get('Uid')
                if numeric_id is not None:
                    numeric_id_str = str(numeric_id).strip()
                    part_name = Workpieces.get(numeric_id_str)
                    print(f"Keresett munkadarab: {part_name} [{numeric_id}]")
                
                # Képből base64
                img_b64 = raw_data.get('Image')
                if img_b64:
                    # Data URL előtag törlés
                    if 'base64,' in img_b64:
                        img_b64 = img_b64.split('base64,')[1]

                    # Kép dekodolás OpenCV-nek
                    img_bytes = base64.b64decode(img_b64)
                    np_arr = np.frombuffer(img_bytes, np.uint8)
                    frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)

                    # Pillanatnyi kép mentése
                    if frame is not None:
                        result = self.detector.process_frame(frame)
                        if result and result['x_target'] is not None:
                            print(f"Cél pozíció a lepakoláshoz: X={result['x_target']:.2f} mm, Y={result['y_target']:.2f} mm, Rz={result['rz_target']:.2f}°")
                            print(f"Cél pozíció a felvételhez: X={result['x_target_unload']:.2f} mm, Y={result['y_target_unload']:.2f} mm, Rz={result['rz_target_unload']:.2f}°")

                            self.publish_target_for_pickup(
                                result['x_target'],
                                result['y_target'],
                                result['angle_deg'],
                                )

                            self.publish_target_for_unload(
                                result['x_target_unload'],
                                result['y_target_unload'],
                                result['rz_target_unload']
                                )

                            self.publish_workpiece_name(part_name)

                            final_image_to_save = result['annotated_frame']
                        else:
                            final_image_to_save = result['annotated_frame'] if result else frame

                        ws_dir = os.path.expanduser('~')
                        file_path = os.path.join(ws_dir, "kevin_ws", "camera_view.png")
                        
                        success = cv2.imwrite(file_path, final_image_to_save)
                        if success:
                            print(f"A kép elmentve ide: {file_path}")

                        return True, numeric_id
                    
        except Exception as e:
            print("Hiba a kép feldolgozásánál:", e)
        return False

def main():
    rclpy.init()
    node = PictureInfoNode()

    try:
        node.start_socket()
    except Exception as e:
        print("Hiba történt:", e)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()