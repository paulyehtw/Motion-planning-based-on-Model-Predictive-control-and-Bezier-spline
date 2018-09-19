# Motion-planning-based-on-Model-Predictive-control-and-Bezier-spline
A quasi Hybrid A* method is introduced for motion planning of autonomous driving car, based on MPC and Bezier spline 

In this algorithm, the following factors are considered:

1. Non-Holonomic feature of the vehicle
2. Speed of the vehicle
3. Start heading and steering angle
4. Goal heading
5. Grid size is 50mx50m

Non-Holonomic feature of the vehicle is taken into consideraion, so the path is continuous and drivable instead of discrete in the
conventinoal A* algorithm.

Due to the usage of Bezier spline, no expand grid or heuristic layer is pre-computed, making it very efficient.

The speed of vehicle is considered, thus it can minimize the error using model predictive control.

The pictures below show that the algorithm can plan a path according the starting/goal heading and steering.
Also it can avoid obstacles and manage to return to Bezier spline as the reference path.
The search tree (green dotted lines) is pruned due to the advantage of th Bezier spline.

![alt text](https://github.com/paulyehtw/Motion-planning-based-on-Model-Predictive-control-and-Bezier-spline/blob/master/Obstacle_Avoid.png)
![alt text](https://github.com/paulyehtw/Motion-planning-based-on-Model-Predictive-control-and-Bezier-spline/blob/master/Search_Tree.png)
