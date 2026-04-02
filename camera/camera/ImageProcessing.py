from __future__ import annotations
import cv2
import numpy as np


STATE_FOLLOWING: int = 0
STATE_SEARCHING: int = 1
STATE_STOP_LOST: int = 2


class ImageProcessing:
    def __init__(
        self,
        threshold:       int   = 80,
        roi_ratio:       float = 0.30,
        use_otsu:        bool  = True,
        min_area:        int   = 300,
        blur_size:       int   = 7,
        morph_size:      int   = 5,
    ) -> None:
        if blur_size % 2 == 0:
            raise ValueError(f"blur_size doit être impair, reçu {blur_size}")
        if not (0.0 < roi_ratio <= 1.0):
            raise ValueError(f"roi_ratio doit être dans ]0, 1], reçu {roi_ratio}")

        self.threshold    = threshold
        self.roi_ratio    = roi_ratio
        self.use_otsu     = use_otsu
        self.min_area     = min_area
        self.blur_ksize   = (blur_size, blur_size)
        self.morph_kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT, (morph_size, morph_size)
        )


    def process_frame(self, frame: np.ndarray) -> dict:
        h, w  = frame.shape[:2]
        roi_y = int(h * (1.0 - self.roi_ratio))
        roi   = frame[roi_y:h, :]

        binary   = self._binarize(roi)
        cx_image = w // 2

        result: dict = dict(
            line_detected = False,
            cx_image      = cx_image,
            error         = None,
            binary        = binary,
            debug         = None,
        )

        cx_line, cy_line = self._centroid_from_contour(binary)

        if cx_line is not None:
            cy_line_abs = roi_y + cy_line
            result["line_detected"] = True
            result["error"]         = float(cx_image - cx_line)
            result["debug"]         = self._draw_debug(
                frame, binary, cx_line, cy_line_abs,
                cx_image, roi_y, result["error"], h, w,
            )
        else:
            result["debug"] = frame.copy()

        return result


    def _binarize(self, roi: np.ndarray) -> np.ndarray:
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, self.blur_ksize, 0)

        flag      = cv2.THRESH_BINARY_INV + (cv2.THRESH_OTSU if self.use_otsu else 0)
        thresh    = 0 if self.use_otsu else self.threshold
        _, binary = cv2.threshold(gray, thresh, 255, flag)

        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, self.morph_kernel)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN,  self.morph_kernel)
        return binary

    def _centroid_from_contour(
        self, binary: np.ndarray
    ) -> tuple[int | None, int | None]:
        contours, _ = cv2.findContours(
            binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        if not contours:
            return None, None

        largest = max(contours, key=cv2.contourArea)
        if cv2.contourArea(largest) < self.min_area:
            return None, None

        M = cv2.moments(largest)
        if M["m00"] == 0:
            return None, None

        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        return cx, cy

    def _draw_debug(
        self,
        frame: np.ndarray, binary: np.ndarray,
        cx_line: int, cy_line: int, cx_image: int,
        roi_y: int, error: float, h: int, w: int,
    ) -> np.ndarray:
        debug = frame.copy()

        cv2.rectangle(debug, (0, roi_y), (w - 1, h - 1), (29, 158, 117), 2)
        cv2.circle(debug, (cx_line, cy_line), 10, (0, 255, 0), -1)
        cv2.line(debug, (cx_image, roi_y), (cx_image, h), (0, 0, 255), 2)
        cv2.arrowedLine(
            debug, (cx_image, cy_line), (cx_line, cy_line),
            (255, 0, 255), 3, tipLength=0.3,
        )

        direction = "← GAUCHE" if error > 0 else "→ DROITE" if error < 0 else "■ CENTRE"
        cv2.rectangle(debug, (0, 0), (w, 32), (30, 30, 30), -1)
        cv2.putText(
            debug, f"erreur: {error:+.1f} px  {direction}",
            (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (29, 158, 117), 2, cv2.LINE_AA,
        )
        return debug
