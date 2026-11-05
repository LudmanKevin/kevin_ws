import cv2
import numpy as np

class WorkpieceDetector:
    def __init__(self, mm_per_pixel_x = 0.0677, mm_per_pixel_y = 0.0666,
                 x_home=461.46, y_home=-66.01, z_home=519.98, rz_home=75.23,
                 x_unload=-1139.89, y_unload=-104.98, z_unload=162.08, rz_unload=-88.23):
        self.mm_per_pixel_x = mm_per_pixel_x
        self.mm_per_pixel_y = mm_per_pixel_y
        self.x_home = x_home
        self.y_home = y_home
        self.z_home = z_home
        self.rz_home = rz_home
        self.x_unload = x_unload
        self.y_unload = y_unload
        self.z_unload = z_unload
        self.rz_unload = rz_unload

    def static_points(self, image, cx_img, cy_img):
        annotated = image.copy()
        cv2.circle(annotated, (cy_img, cx_img), 10, (255, 255, 255), -1) # Pont a kamera közepére
        cv2.line(annotated, (560, 430), (590, 460), (255, 255, 255), 5) # A megfogó alatt
        cv2.line(annotated, (560, 460), (590, 430), (255, 255, 255), 5)
        return annotated

    def process_frame(self, frame):
        if frame is None or frame.size == 0:
            return None

        h, w, _ = frame.shape
        cx_img  = h // 2
        cy_img = w // 2

        base_annotated_frame = self.static_points(frame, cx_img, cy_img)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_green = np.array([35, 170, 100]) # Színárnyalat, telítettség, fényerő
        upper_green = np.array([85, 255, 255])

        mask = cv2.inRange(hsv, lower_green, upper_green)

        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Kontúrok keresése a zöld maszkban
        find_res = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if len(find_res) == 2:
            contours = find_res[0]
        else:
            contours = find_res[1]

        if not contours:
            return {
                'x_mm': None,
                'y_mm': None,
                'x_target': None,
                'y_target': None,
                'z_approach': self.z_home,
                'angle_deg': None,
                'rz_target': None,
                'x_target_unload': None,
                'y_target_unload': None,
                'rz_target_unload': None,
                'annotated_frame': base_annotated_frame
            }

        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)

        # Kiszűrjük a zajokat
        if area < 400:
            return {
                'x_mm': None,
                'y_mm': None,
                'x_target': None,
                'y_target': None,
                'z_approach': self.z_home,
                'angle_deg': None,
                'rz_target': None,
                'x_target_unload': None,
                'y_target_unload': None,
                'rz_target_unload': None,
                'annotated_frame': base_annotated_frame
            }

        # Befoglaló téglalap és elforgatási szög
        rect = cv2.minAreaRect(largest_contour)
        x_px = rect[0][0]
        y_px = rect[0][1]
        angle = rect[2]

        dx_mm = (cx_img - y_px) * self.mm_per_pixel_x
        dy_mm = (x_px - cy_img) * self.mm_per_pixel_y

        # Rajzolás az ellenőrző képre
        annotated_frame = base_annotated_frame.copy()
        box = cv2.boxPoints(rect)
        box = np.int64(box)
        
        cv2.drawContours(annotated_frame, [box], 0, (255, 0, 0), 2) # Kék keret munkadarab köré
        cv2.circle(annotated_frame, (int(x_px), int(y_px)), 5, (255, 0, 0), -1) # Kék pont munkadarab közepére
        cv2.line(annotated_frame, (575, 445), (int(x_px), int(y_px)), (0, 255, 255), 2) # Vonal a megfogó és a munkadarab között
        cv2.line(annotated_frame, (575, 445), (cy_img, cx_img), (0, 255, 255), 2) # Vonal a megfogó és a kamera középpontja között
        
        # A lila pont helyzetének megtartása a képen
        purple_x_px = int(x_px) - 63
        purple_y_px = int(y_px) - 157
        
        cv2.line(annotated_frame, (int(x_px), int(y_px)), (purple_x_px, purple_y_px), (0, 255, 255), 2) 
        cv2.circle(annotated_frame, (purple_x_px, purple_y_px), 10, (255, 0, 255), -1)
        cv2.line(annotated_frame, (cy_img, cx_img), (purple_x_px, purple_y_px), (255, 0, 255), 5)

        # Szöveges adatok kiírása
        workpiece_cords = f"X:{dx_mm:.1f}mm Y:{dy_mm:.1f}mm Rz:{angle:.1f}deg"
        cv2.putText(annotated_frame, workpiece_cords, (int(x_px) + 10, int(y_px) + 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
        # A megfogóhoz viszonyított elmozdulás
        cam_goto_x = ((int(y_px) - 445) * -1) * self.mm_per_pixel_x
        cam_goto_y = (int(x_px) - 575) * self.mm_per_pixel_y
        camera_goto = f"X:{cam_goto_x:.1f}mm Y:{cam_goto_y:.1f}mm"
        cv2.putText(annotated_frame, camera_goto, (purple_x_px + 25, purple_y_px),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        # Munkadarab célpozíciójának kiszámítása a munkaasztalon
        x_target = self.x_home + (cam_goto_x * 10)
        y_target = self.y_home - (cam_goto_y * 10)
        rz_target = self.rz_home + (90 - angle)

        # Munkadarab célpozíciójának kiszámítása mobilrobot tetejéhez
        x_target_unload = self.x_unload + (cam_goto_x * 10)
        y_target_unload = self.y_unload - (cam_goto_y * 10)
        rz_target_unload = self.rz_unload + (90 - angle)

        return {
            'x_mm': dx_mm,
            'y_mm': dy_mm,
            'x_target': x_target,
            'y_target': y_target,
            'z_approach': self.z_home,
            'angle_deg': angle,
            'rz_target': rz_target,
            'x_target_unload': x_target_unload,
            'y_target_unload': y_target_unload,
            'rz_target_unload': rz_target_unload,
            'annotated_frame': annotated_frame
        }