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
    STATE_FOLLOWING: (29, 158, 117),
    STATE_SEARCHING: (27, 156, 239),
    STATE_STOP_LOST: (60,  60, 226),
}


class ImageProcessing:
    def __init__(
        self,
        threshold:       int   = 80,
        roi_ratio:       float = 0.30,
        use_otsu:        bool  = True,
        min_area:        int   = 300,
        blur_size:       int   = 7,
        morph_size:      int   = 5,
        min_contrast:    float = 15.0,
        max_white_ratio: float = 0.80,
        use_contour:     bool  = True,
        center_band:     float = 0.7,
        use_multi_roi:   bool  = True,
        roi_levels:      int   = 3,
    ) -> None:
        if blur_size % 2 == 0:
            raise ValueError(f"blur_size doit être impair, reçu {blur_size}")
        if not (0.0 < roi_ratio <= 1.0):
            raise ValueError(f"roi_ratio doit être dans ]0, 1], reçu {roi_ratio}")

        self.threshold       = threshold
        self.roi_ratio       = roi_ratio
        self.use_otsu        = use_otsu
        self.min_area        = min_area
        self.blur_ksize      = (blur_size, blur_size)
        self.morph_kernel    = cv2.getStructuringElement(
            cv2.MORPH_RECT, (morph_size, morph_size)
        )
        self.min_contrast    = min_contrast
        self.max_white_ratio = max_white_ratio
        self.use_contour     = use_contour
        self.center_band     = center_band
        self.use_multi_roi   = use_multi_roi
        self.roi_levels      = roi_levels

        self._last_cx_line:   int | None = None
        self._last_cy_line:   int | None = None


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

        cx_line_roi, cy_line_roi = self._detect_with_fallback(frame, h, w, roi_y, binary)

        if cx_line_roi is not None:
            cx_line = cx_line_roi
            cy_line = roi_y + cy_line_roi

            self._last_cx_line = cx_line
            self._last_cy_line = cy_line

            result["line_detected"] = True
            result["cx_line"]       = cx_line
            result["error"]         = float(cx_image - cx_line)
            result["debug"]         = self._draw_detected(
                frame, binary, cx_line, cy_line,
                cx_image, roi_y, result["error"], h, w, state,
            )
        else:
            result["debug"] = self._draw_lost(frame, w, h, state, self._last_cx_line, cx_image)

        return result


    def _detect_with_fallback(
        self, frame: np.ndarray, h: int, w: int, roi_y_base: int, binary_base: np.ndarray
    ) -> tuple[int | None, int | None]:
        if self.use_contour:
            cx, cy = self._centroid_from_contour(binary_base)
        else:
            cx, cy = self._centroid_from_moments(binary_base)

        if cx is not None:
            return cx, cy

        if not self.use_multi_roi:
            return None, None

        roi_height = h - roi_y_base
        for level in range(1, self.roi_levels):
            shift      = int(roi_height * 0.5 * level)
            roi_y_up   = max(0, roi_y_base - shift)
            roi_up     = frame[roi_y_up : roi_y_base + roi_height // 2, :]

            if roi_up.shape[0] < 20:
                break

            binary_up  = self._binarize(roi_up)

            if self.use_contour:
                cx_up, cy_up = self._centroid_from_contour(binary_up)
            else:
                cx_up, cy_up = self._centroid_from_moments(binary_up)

            if cx_up is not None:
                cy_adjusted = cy_up + (roi_y_up - roi_y_base)
                return cx_up, max(0, cy_adjusted)

        return None, None


    def _binarize(self, roi: np.ndarray) -> np.ndarray:
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, self.blur_ksize, 0)

        if float(gray.std()) < self.min_contrast:
            return np.zeros(gray.shape, dtype=np.uint8)

        flag      = cv2.THRESH_BINARY_INV + (cv2.THRESH_OTSU if self.use_otsu else 0)
        thresh    = 0 if self.use_otsu else self.threshold
        _, binary = cv2.threshold(gray, thresh, 255, flag)

        white_ratio = float(np.count_nonzero(binary)) / binary.size
        if white_ratio > self.max_white_ratio:
            return np.zeros(gray.shape, dtype=np.uint8)

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

        largest = self._select_best_contour(contours, binary.shape)
        if largest is None:
            return None, None

        if cv2.contourArea(largest) < self.min_area:
            return None, None

        M = cv2.moments(largest)
        if M["m00"] == 0:
            return None, None

        cx = int(M["m10"] / M["m00"])
        cy = int(M["m01"] / M["m00"])
        h, w      = binary.shape
        band_w    = int(w * self.center_band)
        band_x    = (w - band_w) // 2
        band_mask = np.zeros_like(binary)
        band_mask[:, band_x:band_x + band_w] = 255
        binary_band = cv2.bitwise_and(binary, band_mask)

        M_band = cv2.moments(binary_band)
        if M_band["m00"] > self.min_area * 0.3:
            cx = int(M_band["m10"] / M_band["m00"])
            cy = int(M_band["m01"] / M_band["m00"])

        return cx, cy

    def _select_best_contour(
        self, contours: list, shape: tuple
    ):
        h, w = shape
        valid = [c for c in contours if cv2.contourArea(c) >= self.min_area]
        if not valid:
            return None

        if self._last_cx_line is not None:
            def proximity_score(c):
                M = cv2.moments(c)
                if M["m00"] == 0:
                    return float('inf')
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                dist_x = abs(cx - self._last_cx_line)
                bonus_bottom = (h - cy) * 0.3
                return dist_x - bonus_bottom

            return min(valid, key=proximity_score)
        else:
            def bottom_score(c):
                M = cv2.moments(c)
                if M["m00"] == 0:
                    return 0
                return int(M["m01"] / M["m00"])

            return max(valid, key=bottom_score)

    def _centroid_from_moments(
        self, binary: np.ndarray
    ) -> tuple[int | None, int | None]:
        M = cv2.moments(binary)
        if M["m00"] <= self.min_area:
            return None, None
        return int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])

    def _draw_detected(
        self,
        frame: np.ndarray, binary: np.ndarray,
        cx_line: int, cy_line: int, cx_image: int,
        roi_y: int, error: float, h: int, w: int, state: int,
    ) -> np.ndarray:
        debug     = frame.copy()
        color     = _STATE_COLORS.get(state, (200, 200, 200))
        label_str = _STATE_LABELS.get(state, "?")

        roi_overlay          = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
        roi_overlay[:, :, 0] = 0
        roi_overlay[:, :, 2] = 0
        debug[roi_y:h, :]    = cv2.addWeighted(
            debug[roi_y:h, :], 0.7, roi_overlay, 0.3, 0
        )

        band_w = int(w * self.center_band)
        band_x = (w - band_w) // 2
        cv2.rectangle(debug, (band_x, roi_y), (band_x + band_w, h), (200, 200, 0), 1)

        cv2.rectangle(debug, (0, roi_y), (w - 1, h - 1), color, 2)
        cv2.circle(debug, (cx_line, cy_line), 10, (0, 255, 0), -1)
        cv2.circle(debug, (cx_line, cy_line), 12, (255, 255, 255), 2)
        cv2.line(debug, (cx_image, roi_y), (cx_image, h), (0, 0, 255), 2)
        cv2.arrowedLine(
            debug, (cx_image, cy_line), (cx_line, cy_line),
            (255, 0, 255), 3, tipLength=0.3,
        )

        if self._last_cx_line is not None and not (cx_line == self._last_cx_line):
            cv2.circle(debug, (self._last_cx_line, cy_line), 6, (100, 200, 255), 2)

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
        self, frame: np.ndarray, w: int, h: int, state: int,
        last_cx: int | None = None, cx_image: int = 0,
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
        if last_cx is not None:
            side = "← cherche GAUCHE" if last_cx < cx_image else "→ cherche DROITE"
            cv2.putText(
                debug, side,
                (w // 2 - 120, h // 2 + 50),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA,
            )
        return debug

    def __repr__(self) -> str:
        return (
            f"ImageProcessing("
            f"roi_ratio={self.roi_ratio}, use_otsu={self.use_otsu}, "
            f"min_contrast={self.min_contrast}, "
            f"max_white_ratio={self.max_white_ratio}, "
            f"use_contour={self.use_contour}, "
            f"center_band={self.center_band}, "
            f"use_multi_roi={self.use_multi_roi})"
        )