"""m_ranging —— 视觉测距（底边 + 相机倾角的地平面法）

纯算法，无硬件依赖：用已知几何正向投影再反解，验证测距公式与参数接线。
顺便输出**倾角误差敏感性**，提示现场标定倾角的重要性。
"""

import math

from .framework import register, ok, bad

MODULE = "ranging"
TITLE = "视觉测距（底边+倾角地平面法）"

DISTANCES = (200, 300, 500, 800, 1000, 1500, 2000, 3000)


@register(MODULE, TITLE)
def run(ctx):
    from rescue_robot import config as cfg
    from rescue_robot.perception.detection import CVDetector

    ev = []
    h = float(getattr(cfg, "CAMERA_HEIGHT_MM", 210.0))
    tilt = float(getattr(cfg, "CAMERA_TILT_DEG", 30.0))
    fov = float(getattr(cfg, "CAMERA_FOV_DEG", 77.0))
    W, H = tuple(getattr(cfg, "CAMERA_RES", (640, 480)))
    det = CVDetector(camera_height_mm=h, camera_tilt_deg=tilt,
                     camera_fov_deg=fov, image_size=(W, H))
    f = W / (2.0 * math.tan(math.radians(fov) / 2.0))
    ev.append(f"参数：h={h:.0f}mm  tilt={tilt:.1f}°  fov={fov:.0f}°  res={W}x{H}  → 焦距 f={f:.1f}px")

    worst = 0.0
    for d in DISTANCES:
        beta = math.atan(h / d)                                  # 地面点俯角（真值）
        y_bottom = H / 2.0 + f * math.tan(beta - math.radians(tilt))  # 正向投影
        got = det.estimate_ground_position(W / 2.0, y_bottom)
        if got is None:
            ev.append(f"  {d}mm → 反解被拒（视线接近水平，属预期保护）")
            continue
        err = (got[1] - d) / d * 100.0
        worst = max(worst, abs(err))
        ev.append(f"  {d}mm → 估 {got[1]:.0f}mm（误差 {err:+.2f}%）")

    # 倾角敏感性：告诉现场"倾角必须标定"
    beta_1k = math.atan(h / 1000.0)
    y_1k = H / 2.0 + f * math.tan(beta_1k - math.radians(tilt))
    det_bad = CVDetector(camera_height_mm=h, camera_tilt_deg=tilt + 5.0,
                         camera_fov_deg=fov, image_size=(W, H))
    bad_pos = det_bad.estimate_ground_position(W / 2.0, y_1k)
    if bad_pos:
        sens = (bad_pos[1] - 1000.0) / 1000.0 * 100.0
        ev.append(f"敏感性：倾角若偏差 +5°（实为 {tilt:.0f}°），1m 处会算成 {bad_pos[1]:.0f}mm（{sens:+.0f}%）")

    if worst > 1.0:
        return bad(MODULE, f"测距回环误差过大（最大 {worst:.2f}%）→ 公式或参数接线有问题",
                   ev, "检查 CVDetector.estimate_ground_position 与 config.Camera 的接线")
    return ok(MODULE, f"测距正常（200~3000mm 回环最大误差 {worst:.2f}%）", ev,
              "注意：倾角 TILT_DEG 必须真机标定；倾角错 5° 在 3m 处误差可达 -50%")
