import numpy as np
import math

ALPHA = 1
BHETA = 1 * np.pi / 180
RESOLUTION = 0.1
MAX_RANGE = 50
MAP_WIDTH = 100

MAP_SIZE_X = int(MAP_WIDTH / RESOLUTION)
MAP_SIZE_Y = int(MAP_WIDTH / RESOLUTION)

_XARR = np.arange(-MAP_WIDTH / 2, MAP_WIDTH / 2, RESOLUTION)
_YARR = np.arange(-MAP_WIDTH / 2, MAP_WIDTH / 2, RESOLUTION)
_MAP_XX, _MAP_YY = np.meshgrid(_XARR, -_YARR)
_RGRID = np.sqrt(np.add(np.square(_MAP_XX), np.square(_MAP_YY)))
OOR_MASK = _RGRID >= MAX_RANGE


def init_ogm():
    return np.ones((MAP_SIZE_Y, MAP_SIZE_X), dtype=np.float32) * 0.5


def generate_measurement_ogm(lidar_in, ogm_shape):
    H, W = int(ogm_shape[0]), int(ogm_shape[1])
    ogm_step = np.ones((H, W), dtype=np.float32) * 0.5

    if lidar_in is None:
        return ogm_step
    lidar_in = np.asarray(lidar_in)
    if lidar_in.size == 0 or lidar_in.ndim != 2 or lidar_in.shape[1] < 2:
        return ogm_step

    xy = lidar_in[:, :2].astype(np.float32)

    rphi_meas = np.zeros((xy.shape[0], 2), dtype=np.float32)
    rphi_meas[:, 1] = np.sqrt(xy[:, 0] ** 2 + xy[:, 1] ** 2) / ALPHA
    rphi_meas[:, 0] = (np.arctan2(xy[:, 1], xy[:, 0]) + np.pi) / BHETA
    rphi_meas = np.unique(rphi_meas.astype(np.int32), axis=0)

    sg_ang_bin = int(2 * np.pi / BHETA)
    sg_rng_bin = int(MAX_RANGE / ALPHA)
    rphi_meas = rphi_meas[(rphi_meas[:, 1] >= 0) & (rphi_meas[:, 1] < sg_rng_bin)]
    rphi_meas = rphi_meas[(rphi_meas[:, 0] >= 0) & (rphi_meas[:, 0] < sg_ang_bin)]

    scan_grid = np.ones((sg_ang_bin, sg_rng_bin), dtype=np.float32) * 0.5
    scan_grid[tuple(rphi_meas.T)] = 0.7 

    for ang in range(sg_ang_bin):
        ang_arr = rphi_meas[rphi_meas[:, 0] == ang, 1]
        if len(ang_arr) == 0:
            scan_grid[ang, :] = 0.3
        else:
            min_r = int(np.min(ang_arr))
            if min_r > 0:
                scan_grid[ang, :min_r] = 0.3

    phi_idx = ((np.arctan2(_MAP_YY, _MAP_XX) + np.pi) / BHETA).astype(np.int32)
    phi_idx = np.clip(phi_idx, 0, sg_ang_bin - 1)

    r_idx = (_RGRID / ALPHA).astype(np.int32)
    r_idx = np.clip(r_idx, 0, sg_rng_bin - 1)

    ogm_step = scan_grid[phi_idx, r_idx]
    ogm_step[OOR_MASK] = 0.5
    return ogm_step.astype(np.float32)


def logit(m):
    return np.log(np.divide(m, np.subtract(1, m)))


def inverse_logit(m):
    return np.divide(np.exp(m), np.add(1, np.exp(m)))


def update_ogm(prior_ogm, new_ogm):
    logit_map = logit(new_ogm) + logit(prior_ogm)
    out_ogm = inverse_logit(logit_map)
    out_ogm[out_ogm >= 0.98] = 0.98
    out_ogm[out_ogm <= 0.02] = 0.02
    return out_ogm.astype(np.float32)


def _shift_with_fill(img: np.ndarray, dr: int, dc: int, fill_val: float = 0.5) -> np.ndarray:
    out = np.empty_like(img)
    out[...] = fill_val

    H, W = img.shape[:2]
    r0s = max(0, -dr)
    r1s = min(H, H - dr)
    c0s = max(0, -dc)
    c1s = min(W, W - dc)

    r0d = max(0, dr)
    r1d = min(H, H + dr)
    c0d = max(0, dc)
    c1d = min(W, W + dc)

    if r0s < r1s and c0s < c1s:
        out[r0d:r1d, c0d:c1d] = img[r0s:r1s, c0s:c1s]
    return out


def _rotate_nearest(img: np.ndarray, angle_rad: float, fill_val: float = 0.5) -> np.ndarray:
    if abs(angle_rad) < 1e-8:
        return img

    H, W = img.shape[:2]
    cy = (H - 1) / 2.0
    cx = (W - 1) / 2.0

    yy, xx = np.indices((H, W))
    x = xx - cx
    y = yy - cy

    cos_a = math.cos(angle_rad)
    sin_a = math.sin(angle_rad)

    x_src = cos_a * x + sin_a * y
    y_src = -sin_a * x + cos_a * y

    xx_s = np.rint(x_src + cx).astype(np.int32)
    yy_s = np.rint(y_src + cy).astype(np.int32)

    valid = (xx_s >= 0) & (xx_s < W) & (yy_s >= 0) & (yy_s < H)

    out = np.empty_like(img)
    out[...] = fill_val
    out[valid] = img[yy_s[valid], xx_s[valid]]
    return out


def shift_pose_ogm(ogm, init, fin):
    yaw0 = float(init[2])
    yaw1 = float(fin[2])
    if abs(yaw0) > 2 * math.pi + 1.0:
        yaw0 = yaw0 * math.pi / 180.0
    if abs(yaw1) > 2 * math.pi + 1.0:
        yaw1 = yaw1 * math.pi / 180.0

    dyaw = yaw1 - yaw0

    tx = float(init[0] - fin[0])
    ty = float(init[1] - fin[1])
    cos1 = math.cos(yaw1)
    sin1 = math.sin(yaw1)
    dx_body = cos1 * tx + sin1 * ty
    dy_body = -sin1 * tx + cos1 * ty

    dc = int(round(dx_body / RESOLUTION))
    dr = int(round(-dy_body / RESOLUTION))

    dst = _shift_with_fill(ogm, dr, dc, fill_val=0.5)
    dst = _rotate_nearest(dst, -dyaw, fill_val=0.5)
    return dst.astype(np.float32)
