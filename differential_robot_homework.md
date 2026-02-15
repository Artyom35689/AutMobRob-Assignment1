# Assignment 1: Differential Drive Robot Control

## Objective
The goal of this assignment is to develop a control algorithm 
for a differentially driven robot to follow a predefined 
sequence of waypoints. 

Subscribe to odometry data, and publish velocity commands to 
navigate the robot along the given trajectory. Additionally, 
visualize 
 and compare the desired and actual movement of the robot.

---

## Task 1: Implement a Control Algorithm
Write a ROS 2 node that enables the robot to move along the following sequence of waypoints:

```
T1 = [3, 0], T2 = [6, 4], T3 = [3, 4], T4 = [3, 1], T5 = [0, 3]
```

The initial pose of the robot is:

```
[x(0), y(0), ϕ(0)] = [0, 0, 0°]
```

### Requirements:
- Subscribe to the `/odom` topic to obtain the robot’s current position and orientation.
- Publish velocity commands to the `/cmd_vel` topic to move the robot along the given trajectory.
- Implement a control algorithm:
    - Pure Pursuit Controller
    - Proportional Controller (P-Controller)
    - Stanley Controller
    - Model Predictive Control (MPC)
 .
- Stop the robot when it reaches the final waypoint.

---

## Task 2: Data Collection and Visualization
After implementing the control node, collect and plot the following data:

1. **XY Trajectory:** Plot the actual trajectory of the robot versus the expected trajectory.
2. **X vs Time:** Plot the x-coordinate of the robot’s position over time.
3. **Y vs Time:** Plot the y-coordinate of the robot’s position over time.
4. **ϕ (Heading) vs Time:** Plot the robot’s orientation over time.

Use **Matplotlib** or any other suitable tool for visualization.

---

## Submission Requirements
1. **Code Implementation:** Submit your Python script containing the ROS 2 control node.
2. **Plots:** Include the four required plots as images or PDFs.
3. **Short Report:** Provide a brief explanation of your control strategy, challenges faced, and observations from the collected data.

---

## Hints
- Convert the robot’s orientation from quaternion to Euler angles for easier calculations.
    ```python
    _, _, yaw = tf.euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
    ```
- install TF package
    ```bash
    sudo apt install ros-humble-tf-transformations
    ```
- Use **Euclidean distance** to determine when the robot has reached a waypoint.

- Ensure smooth transitions between waypoints by setting a threshold for switching to the next waypoint.


---

 **Good luck!**
