# turntable_rivet_cell_description V11

URDF package for the turntable rivet cell in RViz.

## V11 changes

- Moved the visual belt below the turntable disc, aligned with the lower visible sprocket/gear area.
- Removed prismatic solenoid behavior: both solenoid/piston assemblies are now fixed.
- The sensor pedestal is mounted directly to `base_link` as an independent stand, not to the rotating disc or the turntable mechanical base.
- The photoelectric sensor still points to the right side of fixture 1 only.
- The table rotation joint remains limited to -180 to +180 degrees.

## Launch

```bash
roslaunch turntable_rivet_cell_description display.launch
```

Move only:

```text
table_rotation_joint
```
