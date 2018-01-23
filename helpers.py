import numpy as np
from math import pi


def get_angle_diff(a, b):
    ajusted_a = (a + 4*pi) % (2*pi)
    ajusted_b = (b + 4*pi) % (2*pi)
    print()
    print("ajusted_a: {}, ajusted_b: {}".format(ajusted_a, ajusted_b))
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
