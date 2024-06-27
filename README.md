The project is about object detection and object coordinates calculation using the camera Realsense D435.
Realsense_ws The folder named "Realsense_ws" is a catkin workspace for the first project. You can run the simulation using the following commands:
  - roslaunch realsense2_description view_d435_model_rviz_gazebo.launch
  - rosrun realsense2_description detection.py (code for detecting object)
  - rosrun realsense2_description 3d_coordinate.py (code for calculating detected object coordiante)

you can also find usefull files like:
  - camera.py (for visualizing the image from the camera)
  - 2d_coordinate.py (to calculate the 2D coordinates of an detected object)
