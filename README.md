

# install
```bash
sudo su
pip install keyboard
```


# calibration (need root)
```bash
cd ~/catkin_ws
sudo su
source /opt/ros/melodic/setup.bash
source devel/setup.bash

rosrun sensorglove_sy_gripper glove_gripper_caliration.py
```

# run

```bash
rosrun sensorglove_sy_gripper run_gripper.py
```