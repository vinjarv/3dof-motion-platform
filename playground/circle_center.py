import numpy as np

def find_center(x1, y1, x2, y2, x3, y3):
    A = 2 * np.array([[(x2-x1), (y2-y1)],
                      [(x3-x2), (y3-y2)]])
    b = np.array([[x2**2 + y2**2 - x1**2 - y1**2], 
                  [x3**2 + y3**2 - x2**2 - y2**2]])
    center = np.linalg.solve(A, b)
    center = np.squeeze(center)
    r = np.sqrt((x1-center[0])**2 + (y1-center[1])**2)
    return center, r # center[0] = x, center [1] = y

c, r = find_center(2,1, 0,5, -1,2)
print("x:{:.2f}, y:{:.2f}, r:{:.2f}".format(c[0], c[1], r))