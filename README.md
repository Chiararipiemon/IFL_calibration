# IFL_calibration
## Prelminary checks and the T E--> B from forward kinematics (tf live):
After running roscore and starting the robot application
1. Make sure you havr the URDF and the correct frme names. For example mine are:
  - base: iiwa_link_0 wich is equal to world
  - ee: iiwa_link_ee wich is equal to tool0
```
rosrun tf view_frames
```
Then, to obtain **the transformations FROM EE TO BASE for each robot new position**
```
rosrun tf tf_echo iiwa_link_0 iiwa_link_ee
```
And I obtined, for example:
```
At time 1767868133.714
- Translation: [-0.000, 0.000, 1.306]
- Rotation: in Quaternion [0.000, -0.000, -0.000, 1.000]
            in RPY (radian) [0.000, -0.000, -0.000]
            in RPY (degree) [0.000, -0.000, -0.000]
At time 1767868134.414
- Translation: [-0.000, 0.000, 1.306]
- Rotation: in Quaternion [0.000, -0.000, -0.000, 1.000]
            in RPY (radian) [0.000, -0.000, -0.000]
            in RPY (degree) [0.000, -0.000, -0.000]
```
What I got is T(E-->B)
2. Check if joint states exist
```
rostopic echo -n 1 /ii/joint_states
```
I obtained:
```
header: 
  seq: 6025
  stamp: 
    secs: 1767868219
    nsecs: 314143657
  frame_id: ''
name: 
  - iiwa_joint_1
  - iiwa_joint_2
  - iiwa_joint_3
  - iiwa_joint_4
  - iiwa_joint_5
  - iiwa_joint_6
  - iiwa_joint_7
position: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
velocity: []
effort: []
```
# Transformation from rgb and depth (extrinic transformation)
```
rosrun tf tf_echo rgb_camera_link depth_camera_link
At time 0.000
- Translation: [-0.032, -0.002, 0.004]
- Rotation: in Quaternion [-0.051, -0.001, -0.001, 0.999]
            in RPY (radian) [-0.102, -0.002, -0.003]
            in RPY (degree) [-5.862, -0.086, -0.152]
```
# To take a look of the camera intrinsics
## Find the camera_info topics
```
rostopic list | grep camera_info
```
You should see something like (names vary):

.../rgb/camera_info

.../depth/camera_info
## Read one CameraInfo message
```
 rostopic echo -n i /rgb/camera_info
```
For computing the eye in hand calibration we will use the RGB intrinsics, then for computing the cloudpoint and surface, normal estimation we wil use the depth intrinsics
So now you have:
- fx, fy, cx, cy
- distortion D
- the exact topics used (RGB and/or depth)
the camera TF frame (header.frame_id)

# Eye-in-hand calibration
With the help of OpenCV you need to compute the transformation between camera and end-effector first.
## Publish the camera frames 
You need to publish first the camera frames by launching the driver.launch file
```
roslaunch --screen azure_kinect_ros_driver driver.launch
```
## Run the capture calibration node
This rosnode is helpfull for capturiing the 17-20 robot poses samples
