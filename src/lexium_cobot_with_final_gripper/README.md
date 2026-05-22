# lexium_cobot_with_final_gripper — Z reference version

This package was rebuilt from the URDF code provided by the user in `Pegado text(23).txt`.

## Main change
A fixed `world_to_base_link_z_reference` joint was added before `base_link`:

```xml
<origin xyz="0 0 0" rpy="1.57079632679 0 0"/>
```

This rotates the complete cobot + gripper assembly by +90° around X, so the original cobot +Y working/reference direction becomes global +Z in RViz.

## Gripper
- `gripper_base` stays mounted from `tool0`.
- `appendage_prismatic_joint` remains prismatic.
- It moves in local +Y with a 0.015 m limit, exactly as in the provided URDF.

## Run

```bash
cp -r lexium_cobot_with_final_gripper ~/catkin_ws/src/
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch lexium_cobot_with_final_gripper display.launch
```

In RViz, use `world` as the fixed frame.
