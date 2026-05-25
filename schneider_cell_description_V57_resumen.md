# schneider_cell_description V57 — conveyor +30 cm, J6 lock, 3 cm safety

## Qué cambia respecto a V56

Tres cambios coordinados, todos verificados por dos pruebas que pasan
sin watchdogs, faults ni colisiones.

### 1. Conveyor + suministro + sensor desplazados +0.300 m EAST

El conveyor estaba a (1.370, 1.365) con su pick en world X=1.235.
Quedó muy cerca del cobot (reach 0.353 m) y la combinación
"mismo J6 que PICK_CONVEYOR" no era factible para el LOAD seat al
mismo tiempo. V57 lo mueve +30 cm al este:

| Elemento | V56 | V57 |
|---|---|---|
| `conveyor_section name="conveyor_1"` (xy_x) | 1.370 | **1.670** |
| `suministro_cafi` joint origin xyz | (1.620, 1.365, 1.0075) | **(1.920, 1.365, 1.0075)** |
| `sensor_conveyor_end` face_x | 1.235 | **1.535** |
| `conveyor_drive_motor` xy_x | 1.550 | **1.850** |
| Pick world X (notch -0.135 en south rail) | 1.235 | **1.535** |

Mesa span (x ∈ [0.442, 2.062]). El borde este del suministro queda en
X=1.995, con 67 mm de margen a la orilla mesa este (2.062). Todo dentro
de la mesa.

Estos cambios se reflejan en `conveyor_sim_node.py` (`BELT_X_WEST`,
`BELT_X_EAST`, `SPAWN_X`, `PICK_X`, `CONV_SENSOR_X`),
`object_manager_node.py` (`SPAWN_X`, `PICK_X`) y `resolve_poses.py`
(`CONVEYOR_DX_SHIFT = 0.300`).

### 2. J6 lockeado: la muñeca no rota entre PICK y PLACE

V54-V56 tenían `J6_PICK_CONVEYOR = -1.68` y `J6_PLACE_LOAD = +0.26`
— diferencia de **2 rad = 110°**, que se traducía en que la CAFI
"giraba" visualmente durante el carry.

V57 lockea `q[5] = q5_PICK_CONV` en cada paso del grupo LOAD/RIVET
(APPROACH, PLACE, RELEASE, RETREAT, PICK_RIVETED, LIFT_RIVETED). El
solver:

- Siembra `q[5] = j6_lock ± 0.05 rad` en cada seed aleatorio.
- Aplica `q[5] = j6_lock` después de cada llamada `damped_ls_ik` y deja
  que el siguiente refine absorba el cambio en los otros joints.
- El post-process `R_place_lock` también re-lockea J6.
- Score ordena por `(j5_err + j6_err, rot_err, pos_err)` así la
  solución elegida realmente respeta ambos locks.

Resultado en `resolved_poses.py`:

```
PICK_CONVEYOR              J6 = -2.373404 rad (-135.99 deg)
APPROACH_LOAD_FIXTURE      J6 = -2.373404 rad   (delta = 0.000 mrad)
PLACE_LOAD_FIXTURE         J6 = -2.373404 rad   (delta = 0.000 mrad)
RELEASE_LOAD_FIXTURE       J6 = -2.373404 rad   (delta = 0.000 mrad)
RETREAT_LOAD_FIXTURE       J6 = -2.373404 rad   (delta = 0.000 mrad)
APPROACH_PICK_RIVETED      J6 = -2.373404 rad   (delta = 0.000 mrad)
PICK_RIVETED               J6 = -2.373404 rad   (delta = 0.000 mrad)
LIFT_RIVETED               J6 = -2.373404 rad   (delta = 0.000 mrad)
```

**Cero giro de muñeca durante el carry** — la CAFI queda visualmente
estable desde que se levanta del conveyor hasta que se asienta en el
fixture. J5 sigue clavado en -π/2 (-90°) como en V56.

### 3. Layer de seguridad 3 cm — `v57_collision_test.py`

Nuevo validador que:

- Define 17 obstáculos del mundo como AABB o cilindros (mesa,
  conveyor, suministro, ambos sensores SICK, base del disco,
  fixtures, cradle de visión, ambos bins, poste indicador + 3
  lámparas, 4 postes de la cabina, motor conveyor, cuerpo cámara,
  columna cámara).
- Para cada pose y para cada interpolación lineal entre poses
  consecutivas (10 pasos por segmento), samplea **los 7 segmentos
  físicos** del cobot (base→J1, J1→J2, …, J6→tool0; el `grasp_center`
  está excluido porque es un punto virtual donde se aloja la CAFI,
  no una geometría física).
- Reporta cualquier muestra a < 0.030 m de un obstáculo.
- Excluye el "target obstacle" de cada pose de manipulación
  (conveyor en PICK_CONV, rivet_fixtures en LOAD/RIVET, vision en
  VISION, bin correspondiente en DROP) sólo para muestras de muñeca,
  porque ahí el gripper SÍ debe acercarse al obstáculo objetivo.

```
$ python3 v57_collision_test.py
V57 collision validator — 3 cm safety margin
======================================================================

[1] Static pose check (every resolved pose):
  OK    POSE_APPROACH_CONVEYOR        ... (19 OK)
  ...
  OK    POSE_RETREAT_VISION

[2] Trajectory interpolation check (10 steps per segment):
  Trajectory: PICK_CONV
    OK   POSE_APPROACH_CONVEYOR -> POSE_PICK_CONVEYOR
    OK   POSE_PICK_CONVEYOR -> POSE_LIFT_CONVEYOR
  Trajectory: PLACE_OUTER
    OK   POSE_LIFT_CONVEYOR -> POSE_APPROACH_LOAD_FIXTURE
    OK   POSE_APPROACH_LOAD_FIXTURE -> POSE_RELEASE_LOAD_FIXTURE
    OK   POSE_RELEASE_LOAD_FIXTURE -> POSE_RETREAT_LOAD_FIXTURE
    OK   POSE_RETREAT_LOAD_FIXTURE -> POSE_HOME
  ... (7 trayectorias, 27 segmentos, todos OK)

V57 COLLISION TEST PASSED — every pose and trajectory step is >= 3 cm
from every obstacle.
```

## Pruebas

### Test 1 — `v57_collision_test.py`

19 poses + 27 segmentos interpolados (10 pasos cada uno) = **~280
configuraciones de joints validadas** contra 17 obstáculos del mundo.
Cero violaciones. Margen >= 30 mm a todo obstáculo en todo movimiento.

### Test 2 — `v55_aggressive_test.py`

```
TEST 1 OK: 52 CAFIs through the cell, no fault, no watchdog,
           cabin transitions=313
TEST 2 OK: START -> STOP -> RESET cycle with gripper CLOSED.
           Cleanup order removed=['11', '12', '13'], cell back to IDLE,
           cabin LOWERED, started_once=False.

ALL V55 AGGRESSIVE TESTS PASSED.
```

52 CAFIs completas (2 iniciales + 50 en 5 min simulados) sin un solo
fault y sin disparar ningún watchdog. Flujo START/STOP/RESET intacto.

### Test V56 — `v56_geom_test.py`

Confirma que J5=-90° en todas las 7 poses LOAD/RIVET y que la CAFI
cae a 0.05 mm de la seat. PASS.

### `resolve_poses.py`

```
Summary: 19/19 poses PASS (<= 25 mm cart err)
Joint-mesa fails: 0
```

## Cosas movidas (todo dentro de mesa)

Solo movido en V57:

- `conveyor_1` (y todos sus rails / soportes)
- `suministro_cafi` (feeder block)
- `sensor_conveyor_end` (SICK)
- `conveyor_drive_motor`

Todos +0.300 m en world +X. La cámara y el `fixture_vision` están
intactos (restricción del usuario).

## Archivos cambiados

```
ros_ws/src/schneider_cell_description/urdf/schneider_cell.urdf.xacro  (4 origins)
ros_ws/src/schneider_cell_description/scripts/resolve_poses.py         (J6 lock + dx const)
ros_ws/src/schneider_cell_description/scripts/resolved_poses.py        (regenerado)
ros_ws/src/schneider_cell_description/scripts/validate_robot_poses.txt (regenerado)
ros_ws/src/schneider_cell_description/scripts/v57_collision_test.py    (nuevo)
ros_ws/src/schneider_conveyor_sim/src/conveyor_sim_node.py             (X consts)
ros_ws/src/schneider_object_manager/src/object_manager_node.py         (X consts)
```

## Notas

- `J6 = -2.373 rad = -136°`. Está dentro del límite (±π = ±180°) con
  44° de margen. El cobot puede girar más sin saturar.
- `grasp_center` queda excluido del collision check porque es un
  punto virtual donde está la CAFI (no es geometría física del
  cobot). Sí se chequean los 7 segmentos físicos del esqueleto del
  cobot.
- La cámara está realmente a `z = 1.520 m` (lo subió V40 +200 mm
  precisamente para evitar al cobot). Mi obstáculo lo refleja
  ahora correctamente.
- El segmento de muñeca está exento del obstáculo "manipulation
  target" en la pose correspondiente (e.g. al placear en el fixture,
  la muñeca puede acercarse al fixture sin disparar fail). El resto
  de obstáculos siempre aplica.
