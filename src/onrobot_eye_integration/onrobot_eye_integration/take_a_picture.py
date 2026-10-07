import socketio
import json
import base64
import cv2
import numpy as np
import os

# Onrobot IP cím és süti
ip = "192.168.1.1"
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
    "9A45B701-A8D0-4581-BF40-95F967942449": "Piros háromszög",
}

sio = socketio.Client()

def extract_and_save_image(raw_data):
    try:
        if isinstance(raw_data, str):
            try:
                raw_data = json.loads(raw_data)
            except:
                pass

        # Munkadarab keresés
        if isinstance(raw_data, dict):
            numeric_id = raw_data.get('Uid')
            if numeric_id is not None:
                numeric_id_str = str(numeric_id).strip()
                part_name = Workpieces.get(numeric_id_str)
                print(f"Keresett munkadarab: {part_name}")
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
                    file_name = "camera_view.png"
                    cv2.imwrite(file_name, frame)
                    full_path = os.path.abspath(file_name)
                    print(f"A kép elmentve ide: {full_path}")
                    return True, numeric_id
    except Exception as e:
        print("Hiba a kép feldolgozásánál:", e)
        return False

@sio.event
def connect():
    print("Socket.IO csatlakozás sikeres!")
    try:
        # Vision szoba, ahonnan a képek jönnek
        sio.emit('vision')
        print("Feliratkozva a vision csatornára.")
    except Exception as e:
        print("Hiba a feliratkozásnál:", e)

@sio.on('vision_message')
def on_vision_message(data):
    print("\n[ESEMÉNY]: 'vision_message' érkezett!")
    extract_and_save_image(data)

try:
    # Webszerver
    sio.connect(
        f'http://{ip}', 
        headers={'Cookie': f'jwt={cookie_token}'}, 
        transports=['websocket', 'polling']
    )

    # Folyamatos eseményfigyelés
    sio.wait()

except Exception as e:
    print("Hiba történt:", e)