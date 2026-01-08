# IFL_calibration
## Prelminary checks:
After running roscore and starting the robot application
1. Make sure you havr the URDF and the correct frme names. For example mine are:
  - base: iiwa_link_0 wich is equal to world
  - ee: iiwa_link_ee wich is equal to tool0
```
rosrun tf view_frames
```
Then
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

