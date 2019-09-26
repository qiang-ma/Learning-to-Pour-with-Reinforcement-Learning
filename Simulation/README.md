### Packages
-cv2 (opencv-python)
-python 2.7
-tensorflow 1.7


### Preparation
world file: /ros\_ws/src/fetch\_gazebo/fetch\_gazebo/worlds/

launch file: /ros\_ws/src/fetch\_gazebo/fetch\_gazebo/launch/

put 'test\_zone\_pour.sdf' in /ros\_ws/src/fetch\_gazebo/fetch\_gazebo/worlds/

put 'fetch\_pour.launch' in /ros\_ws/src/fetch\_gazebo/fetch\_gazebo/launch/

To run 'fetch\_pour.launch', cd /ros\_ws/src/fetch\_gazebo/fetch\_gazebo/launch/
and roslaunch fetch\_pour.launch

### Mandatory commands
roslaunch fetch\_moveit\_config move\_group.launch
roslaunch graspit\_interface graspit\_interface.launch

### Core coordinates
-actor cup
cup1 <pose>3.8 3.15 0.83 0 0 3.14</pose>

-target cup
cup2 <pose>3.8 2.85 0.83 0 0 3.14</pose>
