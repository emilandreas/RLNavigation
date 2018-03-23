import numpy as np
from math import pi


def get_angle_diff(a, b):
    ajusted_a = (a + 4*pi) % (2*pi)
    ajusted_b = (b + 4*pi) % (2*pi)
    if (abs(ajusted_a - ajusted_b) < abs(a - b)):
        return ajusted_a - ajusted_b
    else:
        return a - b
def get_dist_to_path(robo_pos, path_pos):
    min_dist = np.inf
    closest_index = -1
    for i in range(len(path_pos)):
        # path_xyz = np.array(point[:3])
        temp_dist = np.linalg.norm(path_pos[i] - robo_pos)
        if temp_dist < min_dist:
            min_dist = temp_dist
            closest_index = i
    return min_dist, closest_index

def vectorAngle(vec): # from -180 to 180
    x = vec[0]
    y = vec[1]
    if x == 0:
        if y > 0:
            return 90
        elif y == 0:
            return 0
        else:
            return -90
    elif y == 0:
        if x >= 0:
            return 0
        else:
            return 180
    ret = np.rad2deg(np.arctan(float(y)/x))
    if x < 0 and y < 0:
        ret = -180 + ret
    elif x < 0:
        ret = 180 + ret
    elif y < 0:
        ret = ret
    return ret

def gaussian(x, mu, sig):
    return np.exp(-np.power(x - mu, 2.) / (2 * np.power(sig, 2.)))

# print(vectorAngle([1,0]))
# print(vectorAngle([1,1]))
# print(vectorAngle([0,1]))
# print(vectorAngle([-1,1]))
# print(vectorAngle([-1,0]))
# print(vectorAngle([-1,-1]))
# print(vectorAngle([0,-1]))
# print(vectorAngle([1,-1]))

# print(gaussian(-5, 0, 2))
# print(gaussian(-4, 0, 2))
# print(gaussian(-3, 0, 2))
# print(gaussian(-2, 0, 2))
# print(gaussian(-1, 0, 2))
# print(gaussian(0, 0, 2))
# print(gaussian(1, 0, 2))
# print(gaussian(2, 0, 2))
# print(gaussian(3, 0, 2))
# print(gaussian(4, 0, 2))
# print(gaussian(5, 0, 2))
