# schneider_cell V20

Reestructuracion modular de V19. La descripcion del modelo (URDF, meshes,
materiales) vive aqui. Toda la logica de proceso esta en los 10 paquetes
hermanos.

Ver `schneider_cell_description_V20_resumen.md` para los detalles.

## Paquetes V20

| Paquete                          | Responsabilidad                                                                  |
|----------------------------------|----------------------------------------------------------------------------------|
| `schneider_cell_description`     | URDF + meshes + materiales + safe_joint_filter                                   |
| `schneider_cell_bringup`         | Launch principal + YAML global + RViz config                                     |
| `schneider_state_manager`        | FSM global de la celda + dispatcher anti-deadlock                                |
| `schneider_conveyor_sim`         | Conveyor + sensor DI1 + spawn interlock + acumulacion FAULT                      |
| `schneider_robot_controller`     | Trayectorias del Lexium (joint-space) + motion_done                              |
| `schneider_gripper_sim`          | Jaw prismatico real + grasp confirmation                                          |
| `schneider_rotary_fixture_sim`   | Disco + solenoides A/B + rivet 30 s timer                                         |
| `schneider_vision_sim`           | Sensor vision + camera trigger + PASS/FAIL TCP/IP                                |
| `schneider_object_manager`       | CAFIs (estado + gravedad + attach + snap)                                        |
| `schneider_visualization`        | Joint state fuser + hoses dinamicas + labels                                     |
| `schneider_hmi`                  | 2 botones operador + monitor I/O                                                  |

## Como correr

```bash
cd ros_ws
catkin_make
source devel/setup.bash
roslaunch schneider_cell_bringup schneider_cell.launch
```

Args opcionales: `rviz:=false hmi:=false safe:=false markers:=false`.

## Validacion offline

`validate_v20.py` (fuera del workspace) hace expansion xacro + parse
URDF + tests geometricos + unit tests del state_manager. Correrlo desde
Windows:

```
python validate_v20.py
```
