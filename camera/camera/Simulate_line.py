import sys
import os
import numpy as np
import cv2
from ImageProcessing import ImageProcessing

class PController:

    def __init__(self, kp: float = 0.005,
                 max_angular: float = 0.8,
                 base_speed: float  = 0.15):
        self.kp          = kp
        self.max_angular = max_angular
        self.base_speed  = base_speed

    def compute(self, error):
        if error is None:
            return 0.0, 0.0
        angular = float(np.clip(self.kp * error,
                                -self.max_angular, self.max_angular))
        return self.base_speed, angular

    def __repr__(self):
        return (f"PController(kp={self.kp}, "
                f"max_angular={self.max_angular}, "
                f"base_speed={self.base_speed})")


def simulate(image_paths: list[str],
             processor: ImageProcessing,
             controller: PController,
             display: bool  = True,
             save_dir: str  = None) -> list:
    results = []

    print("=" * 70)
    print("  SIMULATION SUIVI DE LIGNE")
    print(f"  {processor}")
    print(f"  {controller}")
    print("=" * 70)

    for i, path in enumerate(image_paths):
        if not os.path.isfile(path):
            print(f"  [SKIP] introuvable : {path}")
            continue

        frame = cv2.imread(path)
        if frame is None:
            print(f"  [ERREUR] lecture impossible : {path}")
            continue

        tag = os.path.basename(path)
        res = processor.process_frame(frame)
        vlin, vang = controller.compute(res['error'])

        print(f"\n  Image : {tag}")
        if res['error'] is not None:
            direction = "← GAUCHE" if res['error'] > 0 else \
                        "→ DROITE" if res['error'] < 0 else "■ CENTER"
            print(f"  Erreur         : {res['error']:+.1f} px  [{direction}]")
            print(f"  cx_line        : {res['cx_line']}  |  cx_image : {res['cx_image']}")
            print(f"  cmd_vel        : linear.x={vlin:.3f} m/s  "
                  f"angular.z={vang:+.4f} rad/s")
        else:
            print("  → LIGNE NON DETECTEE — arrêt robot")
        print()

        results.append({'image': tag, 'error': res['error'],
                        'linear': vlin, 'angular': vang})

        if save_dir and res['debug'] is not None:
            os.makedirs(save_dir, exist_ok=True)
            stem = os.path.splitext(tag)[0]

            path_dbg = os.path.join(save_dir, f"debug_{stem}.jpg")
            cv2.imwrite(path_dbg, res['debug'])

            path_bin = os.path.join(save_dir, f"binary_{stem}.jpg")
            cv2.imwrite(path_bin, res['binary'])

            print(f"  Sauvegardé → {path_dbg}")
            print(f"  Sauvegardé → {path_bin}")

        if display and res['debug'] is not None:
            max_h = 700
            dbg   = res['debug']
            bin_  = cv2.cvtColor(res['binary'], cv2.COLOR_GRAY2BGR)

            if dbg.shape[0] > max_h:
                scale = max_h / dbg.shape[0]
                dbg   = cv2.resize(dbg,  None, fx=scale, fy=scale)
                bin_  = cv2.resize(bin_, None, fx=scale, fy=scale)

            cv2.imshow(f'/line/debug  – {tag}',  dbg)
            cv2.imwrite(f'output_{i}_debug.bmp', dbg)
            
            cv2.imshow(f'/line/binary – {tag}',  bin_)
            cv2.imwrite(f'output_{i}_binary.bmp', bin_)

    print("=" * 70)
    print("  RÉSUMÉ")
    print("=" * 70)
    for r in results:
        err_str = f"{r['error']:+.1f} px" if r['error'] is not None \
                  else "NON DETECTEE"
        print(f"  {r['image']:20s}  erreur={err_str:>14s}  "
              f"ω={r['angular']:+.4f} rad/s")
    print("=" * 70)

    return results


if __name__ == '__main__':
    default_names = ['img/image1.jpeg', 'img/image2.jpeg',
                     'img/image3.jpeg', 'img/image4.jpeg']

    if len(sys.argv) > 1:
        image_paths = sys.argv[1:]
    else:
        script_dir  = os.path.dirname(os.path.abspath(__file__))
        image_paths = []
        for name in default_names:
            for d in ['.', script_dir]:
                p = os.path.join(d, name)
                if os.path.isfile(p):
                    image_paths.append(p)
                    break

    if not image_paths:
        print("Aucune image trouvée.")
        print("Usage : python simulate_line_follower.py img1.jpeg img2.jpeg …")
        sys.exit(1)

    processor  = ImageProcessing(
        roi_ratio  = 0.5,
        use_otsu   = True,
        min_area   = 300,
        blur_size  = 7,
        morph_size = 5,
    )

    controller = PController(
        kp          = 0.005,
        max_angular = 0.8,
        base_speed  = 0.15,
    )

    simulate(
        image_paths,
        processor  = processor,
        controller = controller,
        display    = True,
        save_dir   = 'debug_output',
    )
    cv2.waitKey(0)
    cv2.destroyAllWindows()