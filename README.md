# IFL_calibration
The main goal of the calibration is to find the T matrix from camera frame to base frame. With the **eye-in-hand** calibration you will get the T matrix between camera frame and end-effector (constant because the camera is rigidly mounted). It's also important to perform another type of calibration: the one from US imge coordintes to probe (tool) pyshical coordinates. After all these calibrations you can get the cloud point surface of the phantom, the local normals and start to control the robot.

First, set the naming convention of the coordinate frames
- {B} = Robot base frame (KUKA base), in the middle, not in the corner
- {E} = Robot end-effector (as defined by KUKA kinematics)
- {C} = Camera frame (Azure Kinect depth or color frame, choose one and be consistent)
- {M} = ArUco marker frame (fixed on the table)
- {P} = Phantom frame (optional)
- {T} = Tool (ultrasound probe TCP frame)

## First things to do
1. Roscore
2. Run the visualization bridge between the real robot and the rviz gui. Spawn the robot and the connected frames.
    ```
    roslaunch iiwa_visualization display_robot.launch
    ```
## Preliminary checks and get the T E--> B from forward kinematics (tf live):
After running roscore and starting the robot application   
1. Make sure you have the URDF and the correct frme names. For example mine are:
  - **base**: iiwa_link_0 wich is equal to world
  - **ee**  : iiwa_link_ee wich is equal to tool0
    
The command to run and see the tf frames is:
```
rosrun tf view_frames
```
Then, to obtain **the transformations FROM EE TO BASE for each robot new position** (this is the forward kinematics)
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
rostopic echo -n 1 /iiwa/joint_states
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
It's all okay because all my joints were in in 0 position.
# Camera entrinsics and intrinsics
**Camera intrinsics** describe how the camera forms an image. They model the mapping between:
- a 3D point in the camera coordinate frame (X,Y,Z)
- its 2D pixel location (u,v) on the image.

They include:
- focal lengths fx,fy
- principal point cx,cy​
- lens distortion parameters (radial/tangential or rational models)
This is essential because most vision algorithms need a metric interpretation of pixels.

**Camera extrinsics** are the rigid transform between the camera coordinate system and another reference frame (robot end-effector, robot base, world/table, etc.). They describe:

- the camera position (translation)
- the camera orientation (rotation)
## Transformation from rgb and depth (extrinic transformation)
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
- the camera TF frame (header.frame_id)

# Eye-in-hand calibration
The goal of the eye-in-hand calibration is to estimate the rigid transform between the RGB camera frame and the robot end-effector frame.
This transform is required to express visual measurements (e.g., ArUco pose, 3D points, surface normals) in robot coordinates and therefore enables vision-guided motion.
With the help of OpenCV you need to compute the transformation between camera and end-effector first.
## Conda activate 
First you need to activate the conda env to start to work with OpenCV. Of course you env name will be differents from ours. Ours is auto_liver_ultrasound
```
conda activate auto_liver_ultrasound
```
## Publish the camera frames 
First, launch the Azure Kinect driver to publish:
- RGB images (/rgb/image_raw)
- camera intrinsics (/rgb/camera_info)
- camera TF frames (e.g. rgb_camera_link, depth_camera_link)
You need to publish first the camera frames by launching the driver.launch file:
```
roslaunch --screen azure_kinect_ros_driver driver.launch
```
## Run the capture calibration node
A dedicated ROS node is used to collect 17–20 calibration samples.
For each sample, the node records:
- the robot FK transform from TF (iiwa_link_0 → iiwa_link_ee)
- the marker pose estimated from the RGB image using OpenCV (PnP)

**Important**: The ArUco marker dictionary and marker ID must match the printed marker. In our setup the correct dictionary is DICT_ARUCO_ORIGINAL, with marker id 582.
```
rosrun manual_handeye handeye_calibrate.py \
  _image_topic:=/rgb/image_raw \
  _info_topic:=/rgb/camera_info \
  _base_frame:=iiwa_link_0 \
  _ee_frame:=iiwa_link_ee \
  _aruco_dict:=DICT_ARUCO_ORIGINAL \
  _marker_id:=582 \
  _marker_length_m:=0.04 \
  _out_file:=/home/aorta-scan/auto_liver_ultrasound/catkin_ws/src/manual_handeye/handeye_samples.npz
```
screenshot of the GUI 
As you can see, the marker is detected and now you can start to sample.
The node opens a GUI window showing the RGB image and the detected marker.
1. Move the robot to a pose where the marker is visible and the robot is static.
2. When the marker is detected (axes are drawn), press s to save one sample.
3. Repeat for ~20 diverse poses (especially varying orientation).
4. Press q to exit and save the dataset.

## See the saved results 
```
rosrun manual_handeye view_samples.py
```
You will get the number of samples you got, the name of the output file and for each samplet you took all the informations about the forwards kinematics and visual 

## Solve the equation AX = XB
Run:
```
rosrun manual_handeye handeye_solve.py --npz path+name.npz 
```
Now you got the transformation you wanted.
