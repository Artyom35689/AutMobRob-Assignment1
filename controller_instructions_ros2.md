# Running the Controllers as ROS 2 Nodes

## Steps to Set Up and Run

### 1.Create the Python scripts 
Before running the scripts, make them executable:
```
pure_pursuit_controller.py
p_controller.py
stanley_controller.py
mpc_controller.py
```

### 2. Create a ROS 2 Package
If you haven't already, create a ROS 2 package:
```sh
ros2 pkg create --build-type ament_python my_robot_controllers
cd my_robot_controllers
```

### 3. Move the Scripts
Move your Python scripts to the package directory:
```sh
mv /path/to/scripts/*.py my_robot_controllers/my_robot_controllers/
```

### 4. Update `setup.py`
Edit `setup.py` inside `my_robot_controllers` and add the following under `entry_points`:
```python
entry_points={
    'console_scripts': [
        'pure_pursuit = my_robot_controller.pure_pursuit_controller:main',
        'p_controller = my_robot_controller.p_controller:main',
        'stanley_controller = my_robot_controller.stanley_controller:main',
        'mpc_controller = my_robot_controller.mpc_controller:main',
    ],
}
```

### 5. Build the Package
Run the following commands:
```sh
cd ~/AMR_ws  # Navigate to your ROS 2 workspace
colcon build --packages-select my_robot_controllers
source install/setup.bash
```

### 6. Run the Controllers
Once the package is built, you can run each controller using:
```sh
ros2 run my_robot_controllers pure_pursuit
ros2 run my_robot_controllers p_controller
ros2 run my_robot_controllers stanley_controller
ros2 run my_robot_controllers mpc_controller
```

Each controller will subscribe to the `/odom` topic and publish velocity commands to the `/cmd_vel` topic.

---

