import numpy as np
import matplotlib.pyplot as plt
from math import *
from dynamics import Dynamics
from map import mapGenerator
from bezier import Bezier

class Planner:

    def __init__(self, start, goal, start_heading, goal_heading, start_steering, mapsize, freeGrid_num, tolerance, car_length, car_width, wheelbase, OBS_type):
        self.start = start
        self.goal = goal
        self.start_heading = start_heading
        self.goal_heading = goal_heading
        self.start_steering = start_steering
        self.freeGrid_num = freeGrid_num
        self.mapsize = mapsize
        self.tolerance = tolerance
        self.car_length = car_length
        self.car_width = car_width
        self.wheelbase = wheelbase
        self.OBS_type = OBS_type
        bz = Bezier(start, goal, self.start_heading, self.goal_heading, self.start_steering,
                    self.mapsize, self.car_length, self.car_width, self.wheelbase, self.mapsize * 3, "NoFound")
        self.bezier_spline = bz.calculation()
        self.freeGrid_num, self.obstacle, self.danger_zone = mapGenerator(start, self.OBS_type, self.mapsize)


    def calculation(self):
        start = self.start
        goal = self.goal

        theta, theta_future, displacement_rear, displacement_rear_future, steering_step = \
            Dynamics(self.wheelbase, search_length=2.5, speed=5, dt=1)
        # bz = Bezier(start, goal, self.start_heading, self.goal_heading, self.start_steering,
        #             self.mapsize, self.car_length, self.car_width, self.wheelbase, self.mapsize * 3, "NoFound")
        # bezier_spline = bz.calculation()
        x = start[0]
        y = start[1]
        x_future = x
        y_future = y
        x_prev = x
        y_prev = y
        heading_state = self.start_heading
        rotate_angle = heading_state
        steering_state = self.start_steering
        found = 0
        cost_list = [[0, [x, y], heading_state, steering_state]]
        next_state = cost_list
        path_discrete = list([])  # Initialize discrete path
        global path
        global path_tree
        path = []  # Initialize continuous path
        tree_leaf = [[x, y]]  # Initialize search tree leaf (search failed)
        search = 1
        step = 1
        path_tree = []  # Initialize continuous path for search trees

        while found != 1:
            if search >= self.freeGrid_num:
                break

            cost_list.sort(key=lambda x: x[0])
            next_state = cost_list.pop(0)
            path_discrete.append(np.round(next_state[1]))
            path.append(next_state[1])
            [x, y] = next_state[1]
            [x_future, y_future] = [x, y]
            heading_state = next_state[2]
            steering_state = next_state[3]
            if step > 1:
                [x_prev, y_prev] = path[step - 1]
            step += 1
            rotate_angle = heading_state

            if sqrt(np.dot(np.subtract([x, y], goal), np.subtract([x, y], goal))) <= self.tolerance:
                found = 1
            rotate_matrix = [[np.cos(rotate_angle), -np.sin(rotate_angle)],
                             [np.sin(rotate_angle), np.cos(rotate_angle)]]
            action = (np.dot(displacement_rear, rotate_matrix)).tolist()
            action_future = np.dot(displacement_rear_future, rotate_matrix)

            candidates = np.add([x, y], action)
            candidates_future = np.add([x_future, y_future], action_future)
            candidates_round = np.round(candidates).astype(int)
            heading_state = np.add(heading_state, theta)
            invalid_ID = [((candidates_round[i] == path).all(1).any() | (candidates_round[i] == self.danger_zone).all(
                1).any() | (candidates_round[i] == tree_leaf).all(1).any())
                          for i in range(len(candidates_round))]
            remove_ID = np.unique(np.where((candidates < 0) | (candidates > self.mapsize))[0])

            candidates = np.delete(candidates, remove_ID, axis=0)
            candidates_future = np.delete(candidates_future, remove_ID, axis=0)
            heading_state = np.delete(heading_state, remove_ID, axis=0)
            candidates = np.delete(candidates, np.where(invalid_ID), axis=0)
            candidates_future = np.delete(candidates_future, np.where(invalid_ID), axis=0)
            heading_state = np.delete(heading_state, np.where(invalid_ID), axis=0)
            if len(candidates) > 0:
                cost_list = []
                for i in range(len(candidates)):
                    diff = np.square(candidates[i] - self.bezier_spline)
                    min_dis = min(np.sqrt(np.sum(diff, axis=1)))
                    diff_future = np.square(candidates_future[i] - self.bezier_spline)
                    min_dis_future = min(np.sqrt(np.sum(diff_future, axis=1)))
                    total_cost = min_dis + min_dis_future
                    cost_list.append([total_cost, candidates[i], heading_state[i], steering_step[i]])
            else:
                search += 1
                if (next_state[1] == tree_leaf).all(1).any():
                    tree_leaf.append(np.round([x_prev, y_prev]))
                else:
                    tree_leaf.append((np.round([x, y])).tolist())
                x = start[0]
                y = start[1]
                x_future = x
                y_future = y
                x_prev = x
                y_prev = y
                heading_state = self.start_heading
                rotate_angle = heading_state
                steering_state = self.start_steering
                found = 0
                cost_list = [[0, [x, y], heading_state, steering_state]]
                next_state = cost_list
                path_discrete = list([])  # Initialize discrete path
                path_tree.append(path)
                path = []  # Initialize continuous path
                step = 1

        bz_last = Bezier(next_state[1], goal, next_state[2], self.goal_heading, self.start_steering, self.mapsize,
                         self.car_length, self.car_width, self.wheelbase, self.tolerance * 3, "Found")
        bz_last = bz_last.calculation()
        path.append(bz_last)
        return path, path_tree
