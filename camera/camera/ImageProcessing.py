import cv2
import numpy as np


class ImageProcessing:
    def __init__(self,
                 threshold: int   = 80,
                 roi_ratio: float = 0.5,
                 use_otsu: bool   = True,
                 min_area: int    = 300,
                 blur_size: int   = 7,
                 morph_size: int  = 5):

        self.threshold = threshold
        self.roi_ratio = roi_ratio
        self.use_otsu  = use_otsu
        self.min_area  = min_area

        assert blur_size % 2 == 1, "blur_size doit être impair"
        self.blur_ksize   = (blur_size, blur_size)
        self.morph_kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT, (morph_size, morph_size))

    def process_frame(self, frame: np.ndarray) -> dict:
        h, w  = frame.shape[:2]
        roi_y = int(h * (1.0 - self.roi_ratio))
        roi   = frame[roi_y:h, :]

        binary = self._binarize(roi)

        cx_image = w // 2
        result   = dict(cx_line=None, cx_image=cx_image,
                        error=None,   binary=binary,
                        debug=None,   roi_y=roi_y)

        M = cv2.moments(binary)

        if M['m00'] > self.min_area:
            cx_line_roi = int(M['m10'] / M['m00'])
            cy_line_roi = int(M['m01'] / M['m00'])

            cx_line = cx_line_roi
            cy_line = roi_y + cy_line_roi

            error = float(cx_image - cx_line)

            result['cx_line'] = cx_line
            result['error']   = error
            result['debug']   = self._draw_debug(
                frame, binary, cx_line, cy_line,
                cx_image, roi_y, error, h, w)
        else:
            result['debug'] = self._draw_no_line(frame, w, h)

        return result


    def _binarize(self, roi: np.ndarray) -> np.ndarray:
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        gray = cv2.GaussianBlur(gray, self.blur_ksize, 0)

        if self.use_otsu:
            _, binary = cv2.threshold(
                gray, 0, 255,
                cv2.THRESH_BINARY_INV + cv2.THRESH_OTSU)
        else:
            _, binary = cv2.threshold(
                gray, self.threshold, 255, cv2.THRESH_BINARY_INV)

        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, self.morph_kernel)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN,  self.morph_kernel)

        return binary

    def _draw_debug(self, frame: np.ndarray, binary: np.ndarray,
                    cx_line: int, cy_line: int,
                    cx_image: int, roi_y: int,
                    error: float, h: int, w: int) -> np.ndarray:
        
        debug = frame.copy()

        cv2.rectangle(debug, (0, roi_y), (w, h), (255, 220, 0), 2)

        roi_color = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)
        roi_color[:, :, 0] = 0
        roi_color[:, :, 2] = 0
        debug[roi_y:h, :] = cv2.addWeighted(
            debug[roi_y:h, :], 0.7, roi_color, 0.3, 0)

        cv2.circle(debug, (cx_line, cy_line), 10, (0, 255, 0), -1)
        cv2.circle(debug, (cx_line, cy_line), 12, (255, 255, 255), 2)

        cv2.line(debug, (cx_image, roi_y), (cx_image, h), (0, 0, 255), 2)

        cv2.arrowedLine(debug,
                        (cx_image, cy_line),
                        (cx_line,  cy_line),
                        (255, 0, 255), 3, tipLength=0.3)

        direction = "← GAUCHE" if error > 0 else "→ DROITE" if error < 0 else "■ CENTER"
        y_text    = max(roi_y - 15, 25)
        cv2.putText(debug,
                    f'Erreur: {error:+.1f} px  [{direction}]',
                    (10, y_text),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2,
                    cv2.LINE_AA)
        cv2.putText(debug,
                    f'cx_line={cx_line}  cx_image={cx_image}',
                    (10, max(y_text - 28, 10)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1,
                    cv2.LINE_AA)
        return debug

    def _draw_no_line(self, frame: np.ndarray, w: int, h: int) -> np.ndarray:
        debug = frame.copy()
        cv2.putText(debug, 'LIGNE NON DETECTEE',
                    (w // 2 - 150, h // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 3,
                    cv2.LINE_AA)
        return debug

    def __repr__(self) -> str:
        return (f"ImageProcessing(roi_ratio={self.roi_ratio}, "
                f"use_otsu={self.use_otsu}, "
                f"threshold={self.threshold}, "
                f"min_area={self.min_area})")