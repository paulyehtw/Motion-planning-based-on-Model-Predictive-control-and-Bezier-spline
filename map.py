import numpy as np

def mapGenerator(start, obstacle_type, mapsize):
    if obstacle_type == 'Some_obstacle':
        obstacle = np.array([start, [10, 27], [12, 25], [14, 23], [16, 21], [18, 19], [20, 17]])  # Indices of obstacles
    else:
        obstacle = np.array([start])  # Indices of obstacles


    safe_distance = [[1, 0], [0, 1], [-1, 0], [0, -1], [1, 1], [1, -1], [-1, 1], [-1, -1]]
    danger_zone = obstacle  # Obstacle map taking safe distance into consideration
    freeGrid_num = int(mapsize * mapsize - len(obstacle))  # Open grid number
    for i in range(1, len(danger_zone)):
        temp = np.add(danger_zone[i], safe_distance)
        danger_zone = np.append(danger_zone, temp, axis=0)
    danger_zone = np.unique(danger_zone, axis=0)

    return freeGrid_num, obstacle, danger_zone