from __future__ import annotations
import cv2
import numpy as np


STATE_FOLLOWING: int = 0
STATE_SEARCHING: int = 1
STATE_STOP_LOST: int = 2

_STATE_LABELS: dict[int, str] = {
    STATE_FOLLOWING: "FOLLOWING",
    STATE_SEARCHING: "SEARCHING",
    STATE_STOP_LOST: "STOP_LOST",
}

_STATE_COLORS: dict[int, tuple[int, int, int]] = {
    STATE_FOLLOWING: (29, 158, 117),    # vert   (BGR)
    STATE_SEARCHING: (27, 156, 239),    # orange (BGR)
    STATE_STOP_LOST: (60,  60, 226),    # rouge  (BGR)
}


class ImageProcessing:
    def __init__(
        self,
        threshold:  int   = 80,
        roi_ratio:  float = 0.5,
        use_otsu:   bool  = True,
        min_area:   int   = 300,
        blur_size:  int   = 7,
        morph_size: int   = 5,
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


    def process_frame(self, frame: np.ndarray, state: int = STATE_FOLLOWING) -> dict:
        h, w  = frame.shape[:2]
        roi_y = int(h * (1.0 - self.roi_ratio))
        roi   = frame[roi_y:h, :]

        binary   = self._binarize(roi)
        cx_image = w // 2

        result: dict = dict(
            line_detected = False,
            cx_line       = None,
            cx_image      = cx_image,
            error         = None,  
            binary        = binary,
            debug         = None,
            roi_y         = roi_y,
        )

        M = cv2.moments(binary)
        if M["m00"] > self.min_area:
            cx_line_roi = int(M["m10"] / M["m00"])
            cy_line_roi = int(M["m01"] / M["m00"])
            cx_line     = cx_line_roi
            cy_line     = roi_y + cy_line_roi

            result["line_detected"] = True
            result["cx_line"]       = cx_line
            result["error"]         = float(cx_image - cx_line)
            result["debug"]         = self._draw_detected(
                frame, binary, cx_line, cy_line,
                cx_image, roi_y, result["error"], h, w, state,
            )
        else:
            result["debug"] = self._draw_lost(frame, w, h, state)

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


    def _draw_detected(
        self,
        frame:    np.ndarray,
        binary:   np.ndarray,
        cx_line:  int,
        cy_line:  int,
        cx_image: int,
        roi_y:    int,
        error:    float,
        h:        int,
        w:        int,
        state:    int,
    ) -> np.ndarray:
        debug     = frame.copy()
        color     = _STATE_COLORS.get(state, (200, 200, 200))
        label_str = _STATE_LABELS.get(state, "?")

        roi_overlay = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
        roi_overlay[:, :, 0] = 0
        roi_overlay[:, :, 2] = 0
        debug[roi_y:h, :] = cv2.addWeighted(
            debug[roi_y:h, :], 0.7, roi_overlay, 0.3, 0
        )

        cv2.rectangle(debug, (0, roi_y), (w - 1, h - 1), color, 2)

        cv2.circle(debug, (cx_line, cy_line), 10, (0, 255, 0), -1)
        cv2.circle(debug, (cx_line, cy_line), 12, (255, 255, 255), 2)

        cv2.line(debug, (cx_image, roi_y), (cx_image, h), (0, 0, 255), 2)

        cv2.arrowedLine(
            debug,
            (cx_image, cy_line), (cx_line, cy_line),
            (255, 0, 255), 3, tipLength=0.3,
        )

        direction = "← GAUCHE" if error > 0 else "→ DROITE" if error < 0 else "■ CENTRE"
        y_text    = max(roi_y - 15, 40)

        cv2.rectangle(debug, (0, 0), (w, 32), (30, 30, 30), -1)
        cv2.putText(
            debug, f"[{label_str}]  erreur: {error:+.1f} px  {direction}",
            (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA,
        )
        cv2.putText(
            debug, f"cx_line={cx_line}  cx_image={cx_image}",
            (8, y_text), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1, cv2.LINE_AA,
        )
        return debug

    def _draw_lost(
        self,
        frame: np.ndarray,
        w:     int,
        h:     int,
        state: int,
    ) -> np.ndarray:
        debug     = frame.copy()
        color     = _STATE_COLORS.get(state, (60, 60, 226))
        label_str = _STATE_LABELS.get(state, "LOST")

        cv2.rectangle(debug, (0, 0), (w, 32), (30, 30, 30), -1)
        cv2.putText(
            debug, f"[{label_str}]  LIGNE NON DÉTECTÉE",
            (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2, cv2.LINE_AA,
        )
        cv2.putText(
            debug, "LIGNE NON DÉTECTÉE",
            (w // 2 - 180, h // 2),
            cv2.FONT_HERSHEY_SIMPLEX, 1.1, color, 3, cv2.LINE_AA,
        )
        return debug

    def __repr__(self) -> str:
        return (
            f"ImageProcessing("
            f"roi_ratio={self.roi_ratio}, "
            f"use_otsu={self.use_otsu}, "
            f"threshold={self.threshold}, "
            f"min_area={self.min_area})"
        )