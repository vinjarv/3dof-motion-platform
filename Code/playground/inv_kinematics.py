import numpy as np
from numpy import pi, sqrt, sin, cos, arcsin
import matplotlib.pyplot as plt

def ikine(p, r):
    R = 40
    L = 225
    p = p * pi/180
    r = r * pi/180
    z = np.array([  sqrt(3)*L/6 * sin(p)*cos(r) + L/2*sin(r),
                    sqrt(3)*L/6 * sin(p)*cos(r) - L/2*sin(r),
                   -sqrt(3)*L/6 * sin(p)*cos(r)])
    print(z)
    Va = np.zeros(len(z))
    for i in range(len(z)):
        zr = min(-1, max(z[i]/R, 1))
        Va[i] = arcsin(zr) * 180/pi
    return Va

p = np.linspace(-30, 30, 200)
r = -15
# 12.5 degrees roll+pitch looks like the limit for +- 90 servos

Va = []
for i in range(len(Va)):
    Va.append(ikine(p[i], r))
plt.figure(0)
for i, angle in enumerate(Va):
    plt.plot(p, angle, label="Motor {}".format(i+1))
    plt.title("Pitch")
    plt.xlabel("Pitch angle")
    plt.ylabel("Motor angles")
    plt.legend()
plt.show()

r = p.copy()
p = -12.5
Va = []
for i in range(len(Va)):
    Va.append(ikine(p, r[i]))
plt.figure(1)
for i, angle in enumerate(Va):
    plt.plot(r, angle, label="Motor {}".format(i+1))
    plt.title("Roll")
    plt.xlabel("Roll angle")
    plt.ylabel("Motor angles")
    plt.legend()
plt.show()