import cv2
import numpy as np

class WorkpieceDetector:
    def __init__(self, mm_per_pixel=0.635):
        self.mm_per_pixel = mm_per_pixel

    def static_points(self, image, cx_img, cy_img):
        annotated = image.copy()
        cv2.circle(annotated, (cx_img, cy_img), 10, (255, 255, 255), -1) # Pont a kamera közepére
        cv2.line(annotated, (560, 430), (590, 460), (255, 255, 255), 5) # A megfogó alatt
        cv2.line(annotated, (560, 460), (590, 430), (255, 255, 255), 5)
        return annotated

    def process_frame(self, frame):
        if frame is None or frame.size == 0:
            return None

        h, w, _ = frame.shape
        cx_img, cy_img = w // 2, h // 2

        base_annotated_frame = self.static_points(frame, cx_img, cy_img)

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        lower_green = np.array([35, 40, 100]) # Színárnyalat, telítettség, fényerő
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
                'angle_deg': None,
                'annotated_frame': base_annotated_frame
            }

        largest_contour = max(contours, key=cv2.contourArea)
        area = cv2.contourArea(largest_contour)

        # Kiszűrjük a zajokat
        if area < 400:
            return {
                'x_mm': None,
                'y_mm': None,
                'angle_deg': None,
                'annotated_frame': base_annotated_frame
            }

        # Befoglaló téglalap és elforgatási szög
        rect = cv2.minAreaRect(largest_contour)
        x_px = rect[0][0]
        y_px = rect[0][1]
        w_px = rect[1][0]
        h_px = rect[1][1]
        angle = rect[2]

        if y_px > (h-50) or x_px < 50:
            return {
                'x_mm': None,
                'y_mm': None,
                'angle_deg': None,
                'annotated_frame': base_annotated_frame
            }

        # Eltolás számítása a kép középpontjához képest (mm-ben)
        dx_mm = (x_px - cx_img) * self.mm_per_pixel
        dy_mm = (cy_img - y_px) * self.mm_per_pixel

        # Rajzolás az ellenőrző képre
        annotated_frame = base_annotated_frame.copy()
        box = cv2.boxPoints(rect)
        box = np.int0(box)
        
        cv2.drawContours(annotated_frame, [box], 0, (255, 0, 0), 2) # Kék keret munkadarab köré
        cv2.circle(annotated_frame, (int(x_px), int(y_px)), 5, (255, 0, 0), -1) # Kék pont munkadarab közepére
        cv2.line(annotated_frame, (575, 445), (int(x_px), int(y_px)), (0, 255, 255), 2) # Vonal a megfogó és a munkadarab között
        cv2.line(annotated_frame, (575, 445), (cx_img, cy_img), (0, 255, 255), 2) # Vonal a megfogó és a kamera középpontja között
        cv2.line(annotated_frame, (int(x_px), int(y_px)), (int(x_px)-63, int(y_px)-157), (0, 255, 255), 2) 
        cv2.circle(annotated_frame, (int(x_px)-63, int(y_px)-157), 10, (255, 0, 255), -1)
        cv2.line(annotated_frame, (cx_img, cy_img), (int(x_px)-63, int(y_px)-157), (255, 0, 255), 5)

        # Szöveges adatok kiírása
        workpiece_cords = f"X:{dx_mm:.1f}mm Y:{dy_mm:.1f}mm Rz:{angle:.1f}deg"
        cv2.putText(annotated_frame, workpiece_cords, (int(x_px) + 10, int(y_px) + 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
        
        camera_goto = f"X:{(int(x_px)-575):.1f}mm Y:{((int(y_px)-445)*-1):.1f}mm"
        cv2.putText(annotated_frame, camera_goto, ((int(x_px)-63) + 25, int(y_px)-157),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

        return {
            'x_mm': dx_mm,
            'y_mm': dy_mm,
            'angle_deg': angle,
            'annotated_frame': annotated_frame
        }