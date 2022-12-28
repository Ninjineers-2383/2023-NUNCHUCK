import numpy as np

Kv = 2
Ka = 0.5

KturnGear = 1/28

A = np.array([[-Kv/Ka, 0, 0, 0],
             [0, -Kv/Ka, 0, 0],
             [-Kv/(2*Ka), Kv/(2*Ka), 0, 0],
             [KturnGear/2, KturnGear/2, 0, 0]])

B = np.array([[1/Ka, 0], [0, 1/Ka], [1/Ka, -1/Ka], [0, 0]])

x = np.array([0, 0, 0, 0])
u = np.array([12, 12])

for i in range(2):
    x = x + np.matmul(A, x) + np.matmul(B, u)
print(np.matmul(A, x) + np.matmul(B, u))
u = np.array([12, -12])

for i in range(2):
    x = x + np.matmul(A, x) + np.matmul(B, u)
print(np.matmul(A, x) + np.matmul(B, u))
