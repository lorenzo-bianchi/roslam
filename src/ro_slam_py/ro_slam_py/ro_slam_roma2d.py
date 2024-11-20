import numpy as np
from itertools import combinations
from random import shuffle

def minimize_conflicts(n, k, initial_shuffle=False) -> list:
    tuples = list(combinations(range(n), k))

    if initial_shuffle:
        shuffle(tuples)

    result = [tuples[0]]
    available = tuples[1:]

    while available:
        # Find tuple with minimum conflicts
        next_tuple = min(available, key=lambda t: sum(x == y for x in sum(result[-5:], ()) for y in t))
        result.append(next_tuple)
        available.remove(next_tuple)

    return result

def mySVD(A: np.ndarray) -> tuple:
    a, b, c, d = A[0, 0], A[0, 1], A[1, 0], A[1, 1]

    a2 = a * a
    b2 = b * b
    c2 = c * c
    d2 = d * d

    term1 = a2 - 2 * a * d + b2 + 2 * b * c + c2 + d2
    term2 = a2 + 2 * a * d + b2 - 2 * b * c + c2 + d2
    term3 = a2 - b2 + c2 - d2
    term4 = a2 + b2 - c2 - d2
    term5 = 2*a*c + 2*b*d
    term6 = 2*a*b + 2*c*d

    sqrt_term = np.sqrt(term1 * term2)

    epsilon = 1e-10

    v11 = 1
    v12 = term5 / (sqrt_term - (term3 + epsilon))
    v21 = -(sqrt_term + term3) / (term5 + epsilon)
    v22 = 1
    V = np.array([[v11, v12], [v21, v22]]) / np.sqrt(v11 * v22 - v12 * v21)

    u11 = 1
    u12 = term6 / (sqrt_term - (term4 + epsilon))
    u21 = -(sqrt_term + term4) / (term6 + epsilon)
    u22 = 1
    U = np.array([[u11, u12], [u21, u22]]) / np.sqrt(u11 * u22 - u12 * u21)

    return U, V.T

def roma2d(A: np.ndarray, B: np.ndarray, combs: list, max_iterations: int, dist_thr: float, min_inliers: int):
    """
    RObust MAtching 2D Algorithm
    """
    if max_iterations > len(combs):
        max_iterations = len(combs)

    for comb in combs[:max_iterations]:
        comb = list(comb)
        A3, B3 = A[comb], B[comb]
        dA = A3[0,0] * (A3[1,1] - A3[2,1]) + A3[1,0] * (A3[2,1] - A3[0,1]) + A3[2,0] * (A3[0,1] - A3[1,1])
        dB = B3[0,0] * (B3[1,1] - B3[2,1]) + B3[1,0] * (B3[2,1] - B3[0,1]) + B3[2,0] * (B3[0,1] - B3[1,1])
        if dA * dB < 0.1:   # Check if the 3 points are almost collinear or mirrored
            continue

        # Compute transformation (R, t) using the 3 points sampled
        source_points = A3
        target_points = B3

        centroid_source = np.mean(source_points, axis=0)
        centroid_target = np.mean(target_points, axis=0)

        # Center points by subtracting centroids
        centered_source_points = source_points - centroid_source
        centered_target_points = target_points - centroid_target

        # Compute covariance matrix and SVD
        U, _, Vt = np.linalg.svd(centered_source_points.T @ centered_target_points)
        # U, Vt = mySVD(centered_source_points.T @ centered_target_points)

        # Compute rotation and translation
        Rt = U @ Vt
        t = centroid_target - Rt.T @ centroid_source

        ############################

        # Apply transformation to all points in A
        A_transformed = np.dot(A, Rt) + t

        # Compute distances between transformed A and B
        distances = np.linalg.norm(A_transformed - B, axis=1)

        # Count inliers
        inliers_count = np.sum(distances < dist_thr)

        if inliers_count >= min_inliers:
            inliers = np.where(distances < dist_thr)[0]
            return Rt.T, t, inliers

    return np.array([]), np.array([]), np.array([])

if __name__ == '__main__':
    from timeit import timeit
    from math import floor

    def generate_test_case(n_points: int):
        points = np.random.uniform(-10.0, 10.0, (n_points, 2))

        angle = np.random.uniform(-np.pi, np.pi)
        t = np.random.uniform(-5.0, 5.0, 2)

        rotation_matrix = np.array([
            [np.cos(angle), -np.sin(angle)],
            [np.sin(angle),  np.cos(angle)]
        ])

        transformed_points = points @ rotation_matrix.T + t

        for i in np.random.choice(range(n_points), floor(n_points * 0.3)):
            offset = np.random.uniform(-2, 2, 2)
            transformed_points[i] += offset

        return points, transformed_points, rotation_matrix, t

    # Test case
    A, B, R_true, t_true = generate_test_case(10)

    # Single test
    combs = minimize_conflicts(A.shape[0], 3, initial_shuffle=False)
    #combs = list(combinations(range(A.shape[0]), 3))
    R, t, inliers = roma2d(A, B, combs, 120, 0.1, np.round(0.7 * A.shape[0]))
    print('Results:')
    print('========')
    print('True R:')
    print(R_true)
    print()
    print('True t:')
    print(t_true)
    print('--------')
    print('R:')
    print(R)
    print()
    print('t:')
    print(t)
    print()
    print('Inliers:')
    print(inliers)
    print()
    print(f'R error: {0 if np.linalg.norm(R_true - R) < 1e-6 else np.linalg.norm(R_true - R)}')
    print(f't error: {0 if np.linalg.norm(t_true - t) < 1e-6 else np.linalg.norm(t_true - t)}')
    print('========')

    # # # Test with timeit
    # n_tests = 1000
    # total_time = timeit('roma2d(A, B, combs, 120, 0.1, np.round(0.7 * A.shape[0]))', globals=globals(), number=n_tests)
    # print(f'Total time: {total_time}')
    # print(f'Average time: {total_time / n_tests}')