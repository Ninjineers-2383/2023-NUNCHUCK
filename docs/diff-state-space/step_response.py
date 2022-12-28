import numpy as np
import control
import matplotlib.pyplot as plt

Kv = 0.023
Ka = 0.001

KturnGear = 1/28

A = [[-Kv/Ka, 0, 0],
     [0, -Kv/Ka, 0],
     [KturnGear/2, KturnGear/2, 0]]

B = [[1/Ka, 0], [0, 1/Ka], [0, 0]]

C = [[1, 0, 0], [0, 1, 0], [0, 0, 1]]
D = [[0, 0], [0, 0], [0, 0]]

sys = control.ss(A, B, C, D)

x0 = [0, 0, 0]

start = 0
stop = 20
step = 0.02

t = np.arange(start, stop, step)


t, y = control.step_response(
    sys, t, x0)

plt.figure(figsize=(10, 10))
plt.title('Step Response')
plt.subplot(3, 1, 1)
plt.plot(t, y[0][0], 'red')
plt.grid()
plt.legend(labels=('vt[r / s]',))
plt.title("Top Motor Velocity")
plt.subplot(3, 1, 2)
plt.plot(t, y[1][1], 'blue')
plt.grid()
plt.legend(labels=('vb[r / s]',))
plt.title("Bottom Motor Velocity")
plt.subplot(3, 1, 3)
plt.plot(t, y[2][1], 'green')
plt.grid()
plt.legend(labels=('th[radians]',))
plt.title("Wheel Rotation Angle")
plt.xlabel('t[s]')
plt.savefig("step_response.png")
