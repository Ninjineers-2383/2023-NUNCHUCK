import numpy as np

Kv = 0.023
Ka = 0.001

KturnRatio = 1/28

A = np.array([[-Kv/Ka, 0, 0],
              [0, -Kv/Ka, 0],
              [KturnRatio/2, KturnRatio/2, 0]])

C = np.array([[1, 0, 0], [0, 1, 0], [0, 0, 1]])


def observability_matrix(A, C):
    Om = C
    for i in range(A.shape[0]):
        Om = np.vstack((Om, np.linalg.matrix_power(A, i) @ C))
    return Om


Om = observability_matrix(A, C)
print(np.linalg.matrix_rank(Om))
