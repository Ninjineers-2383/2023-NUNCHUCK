import numpy as np

Kv = 0.023
Ka = 0.001

KturnRatio = 1/28

A = np.array([[-Kv/Ka, 0, 0],
              [0, -Kv/Ka, 0],
              [KturnRatio/2, KturnRatio/2, 0]])

B = np.array([[1/Ka, 0], [0, 1/Ka], [0, 0]])


def controllability_matrix(A, B):
    R = B
    for i in range(A.shape[0]):
        R = np.hstack((R, np.linalg.matrix_power(A, i) @ B))
    return R


R = controllability_matrix(A, B)
print(np.linalg.matrix_rank(R))
