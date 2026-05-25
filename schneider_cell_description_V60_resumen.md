# schneider_cell_description V60 — J6 = -45° en LOAD/RIVET

## Cambio único de comportamiento

Las 7 poses LOAD/RIVET ahora tienen **J6 = -π/4 = -45°** (V59 las
tenía en 0 = HOME). Las demás poses, geometría y constantes intactas.

```
Target J6 = -0.785398 rad (-45.00 deg)

POSE_APPROACH_LOAD_FIXTURE       J6 = -0.785398 (-45.00 deg)  OK
POSE_PLACE_LOAD_FIXTURE          J6 = -0.785398 (-45.00 deg)  OK
POSE_RELEASE_LOAD_FIXTURE        J6 = -0.785398 (-45.00 deg)  OK
POSE_RETREAT_LOAD_FIXTURE        J6 = -0.785398 (-45.00 deg)  OK
POSE_APPROACH_PICK_RIVETED       J6 = -0.785398 (-45.00 deg)  OK
POSE_PICK_RIVETED                J6 = -0.785398 (-45.00 deg)  OK
POSE_LIFT_RIVETED                J6 = -0.785398 (-45.00 deg)  OK
```

J5 sigue en -π/2 (-90°). Conveyor +30 cm, disco +30 cm, indicador,
START/STOP/RESET, 3 cm safety, todo intacto.

## Efecto colateral inevitable

El lock J6=-π/4 forzó al IK a una rama "elbow-up" en LOAD/RIVET (q2
positivo en lugar del q2 negativo de V58). Eso se propagaba vía
`prev_q` al grupo VISION, que también pasó a una rama elbow-up
ligeramente distinta. La trayectoria *directa* RETREAT_VISION →
APPROACH_*_BIN (que V58 ejecutaba sin problema) en la nueva rama
rozaba la tapa de los bins a 22 mm — por debajo del límite de 30 mm.

**Fix mínimo en `robot_controller_node.py`**: añadí `POSE_HOME` al
final de `TRAJ_PICK_VISION`. Antes la trayectoria PICK_VISION
terminaba en RETREAT_VISION y la siguiente (PLACE_*_BIN) arrancaba
ahí mismo, así que el swing era directo. Con el HOME forzado:

- PICK_VISION: APPROACH → PLACE → CLOSE → RETREAT → **HOME** ← nuevo
- PLACE_BIN:   APPROACH_BIN → DROP → OPEN → APPROACH → HOME

La transición vision → bin queda partida en dos traverses limpios
a través de HOME. Igual que ya lo hacían `TRAJ_PLACE_VISION`,
`TRAJ_PLACE_ACCEPT` y `TRAJ_PLACE_REJECT` (regla V29).

## Pruebas (todas pasan)

```
$ python3 v56_geom_test.py       → J5=-90 exact, FK error 0.04 mm        PASS
$ python3 v57_collision_test.py  → 0 violaciones (≥3 cm a todo obstáculo) PASS
$ python3 v55_aggressive_test.py → 52 CAFIs, sin faults/watchdogs        PASS
```

## Archivos cambiados

```
ros_ws/src/schneider_cell_description/scripts/resolve_poses.py         (J6 lock -pi/4 + vision seed bias)
ros_ws/src/schneider_cell_description/scripts/resolved_poses.py        (regenerado)
ros_ws/src/schneider_cell_description/scripts/validate_robot_poses.txt (regenerado)
ros_ws/src/schneider_robot_controller/src/robot_controller_node.py     (TRAJ_PICK_VISION termina en HOME)
ros_ws/src/schneider_cell_description/scripts/v57_collision_test.py    (PLACE_BIN test arranca en HOME)
```
