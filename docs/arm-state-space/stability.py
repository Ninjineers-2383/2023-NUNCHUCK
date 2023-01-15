import numpy as np

Kv = 0.023
Ka = 0.001

KturnRatio = 1/28

A = np.array([[-Kv/Ka, 0, 0],
              [0, -Kv/Ka, 0],
              [KturnRatio/2, KturnRatio/2, 0]])

print(np.linalg.eigvals(A))