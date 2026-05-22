# Schneider Project — Simulation (V49)

Simulación ROS 1 (catkin) de la celda de remachado Schneider con cobot
Lexium L03S. Estructurada en 14 paquetes: 3 del cobot Lexium (descripción,
URDF, gripper final) y 11 del sistema Schneider (descripción de celda,
bringup, FSM, conveyor, controlador de robot, gripper, fixture rotatorio,
visión, gestor de objetos, visualización y HMI).

Este repositorio contiene únicamente la parte de **simulación**. Está
pensado para integrarse posteriormente con un proyecto web/control externo.

## Estructura

```
.
├── .catkin_workspace            # marcador de workspace catkin
├── src/                         # paquetes ROS
│   ├── lexium_cobot_description/
│   ├── lexium_cobot_urdfbueno/
│   ├── lexium_cobot_with_final_gripper/
│   ├── schneider_cell_description/
│   ├── schneider_cell_bringup/
│   ├── schneider_state_manager/
│   ├── schneider_conveyor_sim/
│   ├── schneider_robot_controller/
│   ├── schneider_gripper_sim/
│   ├── schneider_rotary_fixture_sim/
│   ├── schneider_vision_sim/
│   ├── schneider_object_manager/
│   ├── schneider_visualization/
│   └── schneider_hmi/
└── evidence/                    # logs, URDFs expandidos, diagnósticos
```

## Paquetes

| Paquete                          | Responsabilidad                                                |
|----------------------------------|----------------------------------------------------------------|
| `schneider_cell_description`     | URDF + meshes + materiales + safe_joint_filter                 |
| `schneider_cell_bringup`         | Launch principal + YAML global + RViz config                   |
| `schneider_state_manager`        | FSM global de la celda + dispatcher anti-deadlock              |
| `schneider_conveyor_sim`         | Conveyor + sensor DI1 + spawn interlock + acumulación FAULT    |
| `schneider_robot_controller`     | Trayectorias del Lexium (joint-space) + motion_done            |
| `schneider_gripper_sim`          | Jaw prismático real + grasp confirmation                       |
| `schneider_rotary_fixture_sim`   | Disco + solenoides A/B + timer de remache 30 s                 |
| `schneider_vision_sim`           | Sensor de visión + camera trigger + PASS/FAIL TCP/IP           |
| `schneider_object_manager`       | CAFIs (estado + gravedad + attach + snap)                      |
| `schneider_visualization`        | Joint state fuser + mangueras dinámicas + labels               |
| `schneider_hmi`                  | 2 botones operador + monitor I/O                               |
| `lexium_cobot_description`       | Descripción base del cobot Schneider Lexium L03S               |
| `lexium_cobot_urdfbueno`         | Variante URDF validada del cobot                               |
| `lexium_cobot_with_final_gripper`| Cobot + gripper final integrado                                |

## Cómo correr

```bash
# Clonar como workspace catkin
git clone https://github.com/Quique2/SchneiderProjectWeb_Simulation.git ros_ws
cd ros_ws

# Compilar
catkin_make
source devel/setup.bash

# Lanzar la celda completa
roslaunch schneider_cell_bringup schneider_cell.launch
```

Args opcionales del launch principal:

```
rviz:=false hmi:=false safe:=false markers:=false
```

## Requisitos

- ROS 1 (Noetic recomendado)
- `catkin`, `xacro`, `robot_state_publisher`, `rviz`, `topic_tools`
- Python 3

## evidence/

Carpeta de referencia con artefactos generados durante el desarrollo:
ciclos simulados (`cycle_simulate_V*.txt`), tests de cinemática
(`fk_selftest_*`, `ik_poses_*`), checks de colisiones, URDFs expandidos
y logs de runtime. No es necesaria para compilar/ejecutar.

## Integración futura

Este repo cubre la capa de simulación. Se conectará con el proyecto web
de control externo vía los tópicos/servicios expuestos por
`schneider_state_manager`, `schneider_hmi` y `schneider_vision_sim`.

## Licencia

BSD (ver `package.xml` de cada paquete).

## Mantenedor

Miguel Aor — `miguelaor681@outlook.com`
