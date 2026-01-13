# IFL_calibration
The main goal of the calibration is to find the T matrix from camera frame to base frame.To obtain so, you need to perform first other calibrations. With the **eye-in-hand** calibration you will get the T matrix between camera frame and end-effector (constant because the camera is rigidly mounted), note: in easy_handeye you fist obtain the T matrix from EE to Camera. Then it's also important to get the transformation between Marker and Base. It's also important to perform another type of calibration: the one from US imge coordintes to probe (tool) pyshical coordinates. After all these calibrations you can get the cloud point surface of the phantom, the local normals and start to control the robot.

<img width="537" height="381" alt="immagine" src="https://github.com/user-attachments/assets/8522d908-554a-44a1-b533-b19d439c638b" />

Referance link: https://docs.opencv.org/4.5.4/d9/d0c/group__calib3d.html#gaebfc1c9f7434196a374c382abf43439b

First, set the naming convention of the coordinate frames
- {B} = Robot base frame (KUKA base), in the middle, not in the corner
- {E} = Robot end-effector (as defined by KUKA kinematics)
- {C} = Camera frame (Azure Kinect depth or color frame, choose one and be consistent)
- {M} = ArUco marker frame (fixed on the table)
- {P} = Phantom frame (optional)
- {T} = Tool (ultrasound probe TCP frame)

## First things to do
1. Start roscore
2. Start the robot aplication

## Preliminary checks and get the T E--> B from forward kinematics (tf live):
After running roscore and starting the robot application, it's better to check first if everything it's ok.

0. Check your rostopic list and see that the robot is in communication with your pc
```
rostopic list
```
Sometimes it happens that your robot application is just not publishing things
1. Make sure you have the URDF and the correct frme names. For example mine are:
  - **base**: iiwa_link_0 wich is equal to world
  - **ee**  : iiwa_link_ee wich is equal to tool0
    
The command to run and see the tf frames is:
```
rosrun tf view_frames
```
Then, to obtain **the transformations FROM EE TO BASE for each robot new position** (this is the **forward kinematics**). This one is the one used inside easy_handeye
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
```
What I got is T(E-->B)

This is **from base to ee**:
```
rosrun tf tf_echo iiwa_link_ee iiwa_link_0
```
<img width="503" height="403" alt="immagine" src="https://github.com/user-attachments/assets/c2138d91-fd38-472a-8e95-4c3f5226c35f" />

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

**Camera extrinsics** are the rigid transform between the camera coordinate system and another reference frame (robot end-effector, robot base, world/table, etc.). 
They describe:

- the camera position (translation)
- the camera orientation (rotation)

<img width="375" height="200" alt="immagine" src="https://github.com/user-attachments/assets/29ae817e-df5a-422d-937c-f6a0316ed80c" />

## Transformation from rgb and depth 
```
rosrun tf tf_echo rgb_camera_link depth_camera_link
At time 0.000
- Translation: [-0.032, -0.002, 0.004]
- Rotation: in Quaternion [-0.051, -0.001, -0.001, 0.999]
            in RPY (radian) [-0.102, -0.002, -0.003]
            in RPY (degree) [-5.862, -0.086, -0.152]
```
# To take a look of the camera intrinsics
This is optain using pnp solver:

<img width="710" height="440" alt="immagine" src="https://github.com/user-attachments/assets/3c380c5c-a2aa-4d09-a693-3d5e70560c52" />

## Find the camera_info topics
```
rostopic list | grep camera_info
```
You should see something like:

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
The goal of the eye-in-hand calibration is to estimate the rigid transform between the RGB camera frame and the robot end-effector frame. In particular: the trasnformation represents the pose of the camera base frame (tracking_base_frame = camera_base) with respect to the robot end-effector frame (robot_effector_frame = iiwa_link_ee).

So it is the transform:

<img width="171" height="40" alt="immagine" src="https://github.com/user-attachments/assets/37c182ed-ab97-46d9-95c4-5bec0d3522ea" />

​
Translation: [x,y,z] in meters, expressed in the iiwa_link_ee frame

Rotation: quaternion (qx,qy,qz,qw) describing the rotation from iiwa_link_ee to camera_base

This transform is required to express visual measurements (e.g., ArUco pose, 3D points, surface normals) in robot coordinates and therefore enables vision-guided motion.

<img width="465" height="344" alt="immagine" src="https://github.com/user-attachments/assets/6870db45-b250-442a-abd6-0906405f5cdc" />

## Easy_hand_eye
It's really simple to perform but the only outut that you have is ^C(T)E:
1. Run the tool in rviz env
   
```
roslaunch easy_handeye calibrate_inside_moveit.launch
```
3. take 17-20 samples
4. compute the calibration
5. save the result
6. publish the result inside Rviz
   
```
roslaunch easy_handeye publish_inside_moveit.launch
```
With this method is really important to start with a good starting pose, the robot should not be so stracthed, the marker should be well visible and, most important thing, each joint should be very far from joints limits. If not, you will get a warning and you can't start the calibration.
**!!!!IMPORTANT!!!!**: with this method you will get the transformation from HAND (EE) to CAMERA --> *hand_camera*

<img width="1219" height="531" alt="immagine" src="https://github.com/user-attachments/assets/fa59dfcc-0458-4922-83a9-9d4e7d84a26f" />

<img width="433" height="504" alt="immagine" src="https://github.com/user-attachments/assets/821d8c6b-c023-4954-8820-f50c5dd61041" />

we obtained this using this:

```
parameters:
  eye_on_hand: true
  freehand_robot_movement: false
  move_group: manipulator
  move_group_namespace: /iiwa
  namespace: /my_eih_calib_eye_on_hand/
  robot_base_frame: iiwa_link_0
  robot_effector_frame: iiwa_link_ee
  tracking_base_frame: camera_base
  tracking_marker_frame: aruco_marker_frame
transformation:
  qw: 0.4999110706628359
  qx: -0.50621128609123011
  qy: -0.49197077528942124
  qz: -0.5018998403158033
  x: 0.0011749601976121817
  y: 0.11322924708755273
  z: 0.021480657476985972
```
