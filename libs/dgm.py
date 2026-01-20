import numpy as np
import math
import cv2

ALPHA = 1
BHETA = 1 * np.pi / 180
RESOLUTION = 0.1
MAX_RANGE = 50
MAP_WIDTH = 100
SPHERICAL2CARTESIAN_BIAS = 6  
FREE_CONF = 0.7
OCC_CONF = 0.7
DYNAMIC_THRESHOLD = 0.7

MAP_SIZE_X = int(MAP_WIDTH / RESOLUTION)
MAP_SIZE_Y = int(MAP_WIDTH / RESOLUTION)

xarr = np.arange(-MAP_WIDTH / 2, MAP_WIDTH / 2, RESOLUTION)
yarr = np.arange(-MAP_WIDTH / 2, MAP_WIDTH / 2, RESOLUTION)
MAP_XX, MAP_YY = np.meshgrid(xarr, -yarr)  # image-like y
rgrid = np.sqrt(np.add(np.square(MAP_XX), np.square(MAP_YY)))
OOR_MASK = rgrid >= MAX_RANGE


def init_dgm():
    """DGM mass grid: (unknown, occupied, free)."""
    dgm = np.zeros((MAP_SIZE_Y, MAP_SIZE_X, 3), dtype=np.float32)
    dgm[:, :, 0] = 1.0  
    return dgm


def _build_scan_grid(rphi_meas: np.ndarray) -> np.ndarray:
    sg_ang_bin = int(2 * np.pi / BHETA)
    sg_rng_bin = int(MAX_RANGE / ALPHA)

    scan_grid = np.zeros((sg_ang_bin, sg_rng_bin, 3), dtype=np.float32)
    scan_grid[:, :, 0] = 1.0 

    if rphi_meas.size == 0:
        scan_grid[:, :, :] = (1 - FREE_CONF, 0.0, FREE_CONF)
        return scan_grid

    scan_grid[tuple(rphi_meas.T)] = (1 - OCC_CONF, OCC_CONF, 0.0)

    for ang in range(sg_ang_bin):
        ang_arr = rphi_meas[rphi_meas[:, 0] == ang, 1]
        if len(ang_arr) == 0:
            scan_grid[ang, :, :] = (1 - FREE_CONF, 0.0, FREE_CONF)
        else:
            min_r = int(np.min(ang_arr))
            if min_r > 0:
                scan_grid[ang, :min_r, :] = (1 - FREE_CONF, 0.0, FREE_CONF)

    return scan_grid


def generate_measurement_dgm(lidar_in, dgm_shape):
    
    H, W = int(dgm_shape[0]), int(dgm_shape[1])

    dgm_step = np.zeros((H, W, 3), dtype=np.float32)
    dgm_step[:, :, 0] = 1.0

    if lidar_in is None:
        return dgm_step
    lidar_in = np.asarray(lidar_in)
    if lidar_in.size == 0 or lidar_in.ndim != 2 or lidar_in.shape[1] < 2:
        return dgm_step

    xy = lidar_in[:, :2].astype(np.float32)

    rphi_meas = np.zeros((xy.shape[0], 2), dtype=np.float32)
    rphi_meas[:, 1] = np.sqrt(xy[:, 0] ** 2 + xy[:, 1] ** 2) / ALPHA
    rphi_meas[:, 0] = (np.arctan2(xy[:, 1], xy[:, 0]) + np.pi) / BHETA

    rphi_meas = np.unique(rphi_meas.astype(np.int32), axis=0)

    sg_ang_bin = int(2 * np.pi / BHETA)
    sg_rng_bin = int(MAX_RANGE / ALPHA)

    rphi_meas = rphi_meas[(rphi_meas[:, 1] >= 0) & (rphi_meas[:, 1] < sg_rng_bin)]
    rphi_meas = rphi_meas[(rphi_meas[:, 0] >= 0) & (rphi_meas[:, 0] < sg_ang_bin)]

    scan_grid = _build_scan_grid(rphi_meas)

    phi_idx = ((np.arctan2(MAP_YY, MAP_XX) + np.pi) / BHETA).astype(np.int32)
    phi_idx = np.clip(phi_idx, 0, sg_ang_bin - 1)

    r_idx = (rgrid / ALPHA).astype(np.int32)
    r_idx = np.clip(r_idx, 0, sg_rng_bin - 1)

    dgm_step = scan_grid[phi_idx, r_idx]
    dgm_step[OOR_MASK] = (1.0, 0.0, 0.0)
    return dgm_step.astype(np.float32)


def update_dgm(prior_dgm, new_dgm):
    prior_dgm = prior_dgm.astype(np.float32)
    new_dgm = new_dgm.astype(np.float32)

    conflict_mass = np.multiply(prior_dgm[:, :, 2], new_dgm[:, :, 1])
    conflict_mass = np.add(conflict_mass, np.multiply(prior_dgm[:, :, 1], new_dgm[:, :, 2]))

    denom = (1.0 - conflict_mass)
    denom[denom == 0] = 1e-6

    free_mass = np.multiply(prior_dgm[:, :, 0], new_dgm[:, :, 2])
    free_mass = np.add(free_mass, np.multiply(prior_dgm[:, :, 2], new_dgm[:, :, 0]))
    free_mass = np.add(free_mass, np.multiply(prior_dgm[:, :, 2], new_dgm[:, :, 2]))
    free_mass = np.divide(free_mass, denom)

    occ_mass = np.multiply(prior_dgm[:, :, 0], new_dgm[:, :, 1])
    occ_mass = np.add(occ_mass, np.multiply(prior_dgm[:, :, 1], new_dgm[:, :, 0]))
    occ_mass = np.add(occ_mass, np.multiply(prior_dgm[:, :, 1], new_dgm[:, :, 1]))
    occ_mass = np.divide(occ_mass, denom)

    unknown_mass = np.multiply(prior_dgm[:, :, 0], new_dgm[:, :, 0])
    unknown_mass = np.divide(unknown_mass, denom)

    updated_dgm = np.stack((unknown_mass, occ_mass, free_mass), axis=2).astype(np.float32)
    return updated_dgm, conflict_mass.astype(np.float32)


def predict_dgm(dgm, dynamic_mass):
    max_mass = np.argmax(dgm, axis=2)
    pred_map = np.zeros(dgm.shape, dtype=np.float32)
    pred_map[max_mass == 0] = (123, 123, 123)
    pred_map[max_mass == 1] = (0, 0, 0)
    pred_map[max_mass == 2] = (255, 255, 255)
    if dynamic_mass is not None:
        pred_map[dynamic_mass >= DYNAMIC_THRESHOLD] = (255, 0, 0)
    return pred_map.astype(np.uint8)


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

def shift_pose_dgm(dgm, init, fin, fill_val: float = 0.5, interpolation: str = "linear", subpixel: bool = True):
    
    dgm_f = np.asarray(dgm, dtype=np.float32)
    H, W = dgm_f.shape[:2]

    yaw0 = float(init[2])
    yaw1 = float(fin[2])
    if abs(yaw0) > 2 * math.pi + 1.0:
        yaw0 = math.radians(yaw0)
    if abs(yaw1) > 2 * math.pi + 1.0:
        yaw1 = math.radians(yaw1)

    dyaw = yaw1 - yaw0

    tx = float(init[0] - fin[0])
    ty = float(init[1] - fin[1])
    cos1 = math.cos(yaw1)
    sin1 = math.sin(yaw1)
    dx_body = cos1 * tx + sin1 * ty
    dy_body = -sin1 * tx + cos1 * ty

    if subpixel:
        dc = float(dx_body / RESOLUTION)     
        dr = float(-dy_body / RESOLUTION)     
    else:
        dc = float(int(round(dx_body / RESOLUTION)))
        dr = float(int(round(-dy_body / RESOLUTION)))

    is_dgm_mass = (dgm_f.ndim == 3 and dgm_f.shape[2] == 3)

    if is_dgm_mass:
        border_value = (1.0, 0.0, 0.0)
        fallback_fill = (1.0, 0.0, 0.0)
    else:
        if dgm_f.ndim == 2:
            border_value = float(fill_val)
            fallback_fill = float(fill_val)
        else:
            border_value = (float(fill_val),) * dgm_f.shape[2]
            fallback_fill = (float(fill_val),) * dgm_f.shape[2]

    # ---- No OpenCV ----
    if cv2 is None:
        dr_i = int(round(dr))
        dc_i = int(round(dc))
        dst = _shift_with_fill(dgm_f.copy(), dr_i, dc_i, fill_val=fallback_fill)
        dst = _rotate_nearest(dst, -dyaw, fill_val=fallback_fill)
        if is_dgm_mass:
            dst = np.clip(dst, 0.0, 1.0)
            s = dst.sum(axis=2, keepdims=True)
            dst = dst / np.maximum(s, 1e-6)
        return dst.astype(np.float32, copy=False)

    # ---- OpenCV: one-shot warpAffine ----
    cx = (W - 1) / 2.0
    cy = (H - 1) / 2.0

    angle_deg = (-dyaw) * 180.0 / math.pi

    Mrot = cv2.getRotationMatrix2D((cx, cy), angle_deg, 1.0).astype(np.float32)

    T = np.array([[1.0, 0.0, dc],
                  [0.0, 1.0, dr],
                  [0.0, 0.0, 1.0]], dtype=np.float32)

    R = np.array([[Mrot[0, 0], Mrot[0, 1], Mrot[0, 2]],
                  [Mrot[1, 0], Mrot[1, 1], Mrot[1, 2]],
                  [0.0,        0.0,        1.0]], dtype=np.float32)

    M = (R @ T)[:2, :]

    interp = interpolation.lower().strip()
    if interp in ("nearest", "nn"):
        flags = cv2.INTER_NEAREST
    else:
        flags = cv2.INTER_LINEAR

    dst = cv2.warpAffine(
        dgm_f,
        M,
        dsize=(W, H),
        flags=flags,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=border_value,
    )

    if is_dgm_mass:
        dst = np.clip(dst, 0.0, 1.0)
        s = dst.sum(axis=2, keepdims=True)
        dst = dst / np.maximum(s, 1e-6)

    return dst.astype(np.float32, copy=False)
