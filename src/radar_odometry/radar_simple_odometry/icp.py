import numpy as np
from scipy.spatial import cKDTree

def _best_fit_transform(A: np.ndarray, B: np.ndarray):
    # A,B: Nx3 corresponded
    centroid_A = A.mean(axis=0)
    centroid_B = B.mean(axis=0)
    AA = A - centroid_A
    BB = B - centroid_B
    H = AA.T @ BB
    U, S, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    # reflection fix
    if np.linalg.det(R) < 0:
        Vt[2, :] *= -1
        R = Vt.T @ U.T
    t = centroid_B - R @ centroid_A
    T = np.eye(4, dtype=np.float64)
    T[:3,:3] = R
    T[:3, 3] = t
    return T

def icp_point_to_point(src: np.ndarray, dst: np.ndarray, max_iters=20, max_corr_dist=1.0):
    """Return T such that dst ~= T * src. src,dst are Nx3 float arrays."""
    if src.shape[0] < 3 or dst.shape[0] < 3:
        return np.eye(4)
    T = np.eye(4, dtype=np.float64)
    dst_tree = cKDTree(dst)
    prev_error = None
    for _ in range(max_iters):
        src_h = np.hstack([src, np.ones((src.shape[0],1))])
        src_tf = (T @ src_h.T).T[:, :3]
        dists, idx = dst_tree.query(src_tf, k=1, workers=-1)
        mask = dists < max_corr_dist
        if mask.sum() < 6:
            break
        A = src_tf[mask]
        B = dst[idx[mask]]
        dT = _best_fit_transform(A, B)
        T = dT @ T
        mean_err = float(dists[mask].mean())
        if prev_error is not None and abs(prev_error - mean_err) < 1e-4:
            break
        prev_error = mean_err
    return T
