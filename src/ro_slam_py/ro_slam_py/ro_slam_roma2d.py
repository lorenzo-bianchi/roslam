import numpy as np
from itertools import combinations

def roma2d(A: np.ndarray, B: np.ndarray, max_iterations: int, dist_thr: float, min_inliers: int):
    """
    RObust MAtching 2D Algorithm
    """
    for i, comb in enumerate(combinations(range(A.shape[0]), 3)):
        if i >= max_iterations:
            break

        comb = list(comb)
        # Compute transformation (R, t) using the 3 points sampled
        R, t = compute_transform(A[comb], B[comb])

        # Apply transformation to all points in A
        A_transformed = np.dot(A, R.T) + t

        # Compute distances between transformed A and B
        distances = np.linalg.norm(A_transformed - B, axis=1)

        # Count inliers
        inliers_count = np.sum(distances < dist_thr)

        if inliers_count >= min_inliers:
            inliers = np.where(distances < dist_thr)[0]
            return R, t, inliers

    return np.array([]), np.array([]), np.array([])

def compute_transform(source_points: np.ndarray, target_points: np.ndarray):
    # Compute centroids of both sets of points
    centroid_source = np.mean(source_points, axis=0)
    centroid_target = np.mean(target_points, axis=0)

    # Center points by subtracting centroids
    centered_source_points = source_points - centroid_source
    centered_target_points = target_points - centroid_target

    # Compute covariance matrix and SVD
    U, _, Vt = np.linalg.svd(centered_source_points.T @ centered_target_points)

    # Compute rotation
    R = Vt.T @ U.T

    # Ensure matrix is a proper rotation
    if R[0,0] * R[1,1] - R[0,1] * R[1,0] < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    # Compute translation
    t = centroid_target - R @ centroid_source

    return R, t

if __name__ == '__main__':
    import time

    # Test
    A = np.array([
        [-2.5257, 1.9402],
        [-2.1892, -7.7403],
        [0.7103, -0.7110],
        [-1.8062, -6.4765],
        [2.8895, -7.9924],
        [0.0521, 0.3754],
        [0.8739, -8.1434],
        [1.3665, -0.3018],
        [3.6489, -3.8663],
        [1.4439, -8.0681]
    ])
    B = np.array([
        [-5.2312, 4.7975],
        [0.2685, 0.6171],
        [-7.0679, 3.4119],
        [-1.2302, 1.0364],
        [0.1669, 5.7598],
        [-8.1613, 2.7375],
        [0.3636, 3.7533],
        [-7.5057, 4.0739],
        [-3.9676, 6.4261],
        [0.2740, 4.3213]
    ])

    tic = time.time()
    R, t, inliers = roma2d(A, B, 120, 0.1, np.round(0.7 * A.shape[0]))
    print(f"Elapsed time: {time.time() - tic}")
    print(R)
    print(t)
    print(inliers)