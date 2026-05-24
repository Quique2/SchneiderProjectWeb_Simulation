# schneider_cell_description V55 — START / RESET + cabin & J5 interlock

## Lo que cambia respecto a V54

V55 añade al HMI los dos botones que pediste y reorganiza la FSM y el
robot_controller alrededor de un par de interlocks de seguridad. No se
toca la URDF; los cambios son todos en software.

### HMI (`schneider_hmi/hmi_node.py`)
- Cuatro botones operativos (antes eran 2):
  - **START** — habilitado solo si `cell == IDLE`. Publica
    `/operator/start` (Empty). El state_manager transiciona
    IDLE → RUNNING. Después queda desactivado hasta el próximo IDLE.
  - **Colocar CAFI** — habilitado solo si `cell == RUNNING` y
    `/conveyor/spawn_allowed == True`. El gating de “sólo una CAFI si ya
    hay una” lo sigue haciendo `conveyor_sim._spawn_can_proceed` (rechaza
    si hay CAFI en spawn zone, en el sensor de pick, o acumulación).
  - **STOP** — habilitado solo si `cell == RUNNING`. Publica
    `/operator/stop=True`. El antiguo *toggle* RESUME ya no existe.
  - **RESET** — habilitado solo si `cell == PAUSED/FAULT`. Publica
    `/operator/reset` (Empty).
- Nueva etiqueta `CABIN RAISED/LOWERED/RAISING/LOWERING` con
  color-coded según `/rivet/cabin_state`.

### state_manager (`schneider_state_manager/state_manager_node.py`)
- `/operator/spawn_cafi` ya **no** arranca la celda. Antes (V21..V54) un
  spawn en IDLE forzaba IDLE→RUNNING; ahora la única vía es
  `/operator/start`. Esto desacopla el “arranque” del “meter pieza”.
- `/operator/start` → IDLE→RUNNING, marca `started_once=True`.
- `/operator/stop=True` → RUNNING→PAUSED como antes. El RESUME-por-toggle
  está suprimido a propósito: RESET es la única salida.
- `/operator/reset` (sólo aceptado en PAUSED/FAULT) ejecuta el recovery:
  1. **Sube la cabina** (`/rivet/cabin_cmd=RAISE`,
     `/rivet/cabin_state=RAISED`).
  2. Si el gripper está `CLOSED` (lleve CAFI o no) dispara
     `TRAJ_PLACE_REJECT` → el cobot suelta lo que tenga en la papelera
     de rechazo y vuelve a HOME.
  3. Pasa a `STAGE_RESET_CLEAN` y va eliminando CAFIs **una por
     segundo** en este orden:
     - vision (`location == "at_vision"`)
     - outer (`location == "in_fixture_<outer_id>"`)
     - inner: si sólo queda inner, manda `INDEX +180` y, tras el
       `disc_index_done`, lo elimina como outer.
  4. Cuando ya no queda nada → IDLE, baja la cabina, `started_once=False`
     (hay que volver a pulsar START).
- **Cabin interlock automático**: la cabina sube antes de cada `PLACE_*`
  (LOAD / VISION / BIN) y baja al final de cada place. El propósito —
  como pediste — es que el cobot nunca entregue con la cabina abajo y
  evite cualquier colisión con la canopa.
- Watchdogs nuevos para el reset:
  `WD_RESET_REJECT_S=25 s`, `WD_RESET_INDEX_S=8 s`,
  `WD_RESET_TICK_S=1 s`, `WD_RESET_GLOBAL_S=90 s`.

### robot_controller (`schneider_robot_controller/robot_controller_node.py`)
- Nuevo helper `maybe_clamp_j5(step, q)` que **fuerza joint 5 = -π/2
  (= -90°)** en cada paso de entrega:
  `POSE_RELEASE_LOAD_FIXTURE`, `POSE_RELEASE_VISION`,
  `POSE_DROP_ACCEPT_BIN`, `POSE_DROP_REJECT_BIN`.
- Se aplica al **enqueue** del segmento, antes del clamping de límites y
  del `shortest_path_target`, así la interpolación ya viaja al ángulo
  clampado y la motion sigue siendo suave. Los valores resueltos por IK
  en V54 ya estaban entre −87° y −90°, por lo que el TCP se mueve
  menos de 5 mm — muy por dentro de la tolerancia lateral V51.

### conveyor_sim (`schneider_conveyor_sim/conveyor_sim_node.py`)
- `_spawn_can_proceed` ya **no** acepta IDLE. V21..V54 lo permitían para
  romper un deadlock de arranque que en V55 ya no existe (START arma la
  celda explícitamente).

### object_manager (`schneider_object_manager/object_manager_node.py`)
- Nuevo subscriber `/objects/remove_cafi` que acepta:
  - `"<int>"` → elimina por id
  - `"at_vision"`, `"in_fixture_A"`, `"in_fixture_B"`, `"on_conveyor"`
    → elimina todas las CAFIs en esa location
  - `"*"` → hard reset (borra todas)
  Lo usa el state_manager durante el recovery para simular que el
  operador remueve la pieza con la mano.

## Pruebas

Dos scripts nuevos bajo
`ros_ws/src/schneider_cell_description/scripts/`:

| Script | Qué hace |
|---|---|
| `rospy_stub.py` | Stand-in mínimo de `rospy` + `std_msgs.msg` para ejecutar el FSM real sin un master ROS. Gestiona un reloj falso, bus de tópicos y captura de logs. |
| `v55_aggressive_test.py` | Carga **el state_manager real** vía el stub y corre las dos pruebas exigidas. |

### Resultado de las pruebas

```
TEST 1 OK: 52 CAFIs through the cell, no fault, no watchdog,
           cabin transitions=313
TEST 2 OK: START -> STOP -> RESET cycle with gripper CLOSED.
           Cleanup order removed=['11', '12', '13'],
           cell back to IDLE, cabin LOWERED, started_once=False.

ALL V55 AGGRESSIVE TESTS PASSED.
```

- **Test 1 (throughput)**: START → 2 CAFIs back-to-back + 50 CAFIs más
  durante 5 minutos simulados. 52 ciclos completos
  PICK_CONV → PLACE_LOAD → SEAT → INDEX → RIVET → INDEX_BACK →
  PICK_RIVETED → PLACE_VISION → INSPECT → PICK_VISION → PLACE_BIN sin
  un solo fault y sin disparar ningún watchdog. La cabina alterna
  RAISED/LOWERED 313 veces (sube/baja en cada delivery).
- **Test 2 (botones)**: START → spawn → llega a PLACE_LOAD con gripper
  CLOSED → STOP → RESET. El SM dispara `request_place_reject_bin`,
  luego elimina las 3 CAFIs strand-eadas en el orden correcto (vision
  primero, outer después, inner al final tras `INDEX +180`),
  transiciona a IDLE, baja la cabina y limpia `started_once`. Cero
  faults, cero watchdogs.

### Cómo correrlas

```bash
cd ros_ws/src/schneider_cell_description/scripts
python3 v55_aggressive_test.py
```

Sin dependencias externas — usa el stub que viene en el zip.

## Topics nuevos

| Topic | Tipo | Publicador | Quién consume |
|---|---|---|---|
| `/operator/start`     | `std_msgs/Empty`  | HMI               | state_manager |
| `/operator/reset`     | `std_msgs/Empty`  | HMI               | state_manager |
| `/rivet/cabin_cmd`    | `std_msgs/String` | state_manager     | (logging / future sim) |
| `/rivet/cabin_state`  | `std_msgs/String` | state_manager     | HMI |
| `/objects/remove_cafi`| `std_msgs/String` | state_manager     | object_manager |

## Archivos modificados

```
ros_ws/src/schneider_hmi/src/hmi_node.py
ros_ws/src/schneider_state_manager/src/state_manager_node.py
ros_ws/src/schneider_robot_controller/src/robot_controller_node.py
ros_ws/src/schneider_conveyor_sim/src/conveyor_sim_node.py
ros_ws/src/schneider_object_manager/src/object_manager_node.py
ros_ws/src/schneider_cell_description/scripts/rospy_stub.py            (nuevo)
ros_ws/src/schneider_cell_description/scripts/v55_aggressive_test.py   (nuevo)
```

## Notas y caveats

- La cabina de remache es un **interlock lógico** en V55 — no se añadió
  un joint prismático a la URDF. El estado se publica en
  `/rivet/cabin_state` y se ve en el HMI, pero la canopa visual no se
  mueve. Razón: en V54 la canopa ya estaba 214 mm por encima del CAFI y
  el `press_pillar/head/tip` está `include_press="false"`, así que no
  hay colisión geométrica a evitar; el interlock es funcional para que
  la secuencia respete el protocolo que pediste. Si más adelante
  quieres que la canopa realmente suba en RViz, se añade un
  `prismatic_joint` en `riveting_station.xacro` y un sim node que
  consuma `/rivet/cabin_cmd` — pero eso ya no afecta a esta entrega.
- El RESET asume que `disc_index_done` se publica cuando el disco
  completa el +180 (idéntico al ciclo normal). Si la rotación falla,
  el watchdog `WD_RESET_INDEX_S=8 s` salta al substage CLEAN y se
  ignora la CAFI inner; el watchdog global de 90 s te devuelve a IDLE
  pase lo que pase, para que la celda no se cuelgue.
- Los tests son **offline** y simulan el mundo (cobot, gripper,
  fixture, disco) mediante callbacks directos. No reemplazan un
  smoke-test sobre la URDF + RViz, pero garantizan que la lógica de
  start / stop / reset / placement-watchdogs y la secuencia de cleanup
  del RESET son correctas. Para validar el J5=-90° visualmente basta
  con grep en los logs (`grep "V55 clamp J5" ~/.ros/log/.../*.log`).
