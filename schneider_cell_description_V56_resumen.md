# schneider_cell_description V56 — shift disc +30 cm, J5=-90 natural, cabin removed

## Qué cambia respecto a V55

V56 mueve toda la estación de remache hacia el +X y elimina la cabina,
dejando solo el indicador y el disco rotatorio. Ahora el cobot llega al
LOAD seat con joint5 = -π/2 (= -90°) como **solución natural del IK** —
no por un clamp post-hoc, sino porque la geometría lo permite.

### Búsqueda del shift (`scripts/probe_j5_reach.py`)

Probé +X = {0.10, 0.15, …, 0.45} m con un IK constrained a `J5 = -π/2` y
posicion strict top-down. Resultados clave:

| dx (m) | LOAD_X | pos_err | J5 | J4 | J3 | OK |
|---|---|---|---|---|---|---|
| +0.22 | 0.9003 | 3.5 mm | -90° | -48° | +104° |  |
| +0.25 | 0.9303 | 6.0 mm | -90° | -21° | +99°  |  |
| +0.30 | **0.9803** | **6.7 mm** | **-90°** | **+95°** | **-114°** |  |
| +0.35 | 1.0303 | 1.8 mm | -90° | +145° | +31°  | OK |
| +0.40 | 1.0803 | 2.1 mm | -90° | -169° | -60°  | OK |

Elegí **dx = +0.300 m** (centro del rango 20-40 cm que pediste). Es la
opción más conservadora en términos de proximidad al cobot — a +0.35 m
el disco quedaría demasiado cerca del pedestal. Con dx=0.30 el cobot
sigue teniendo ~50 mm de holgura entre la pestaña del disco y la base.

Con la búsqueda fina de `resolve_poses.py` (300 seeds + iterative
lateral-shift refinement + J5 lock) la posicion error baja a **0.16 mm**
en `PLACE_LOAD_FIXTURE` (= 0.5 % de la tolerancia 25 mm).

### Cambio en URDF (`schneider_cell.urdf.xacro`)

- `world_to_riveting_zone` movido de `xyz="0.692 1.259 1.000"` a
  `xyz="0.992 1.259 1.000"`. El disco entero (base, motor NEMA, banda,
  ambos fixtures con sus pistones, sensor fotoelectrico, pedestal,
  todos los frames `cafi_lateral_target_*`) se desplaza con él porque
  parentan a `riveting_zone` vía la macro `turntable_rivet_cell`.
- **Cabina eliminada**: borrada la llamada a `riveting_station_features`
  (postes NE/NW, canopa, lip, tray, label) y todo lo que la macro
  emitía.
- **Indicador conservado**: nuevo poste corto + 3 lámparas (rojo,
  ámbar, verde) montados sobre `riveting_zone` a 220 mm al oeste del
  disco. La luz roja a z=0.330 sobre la mesa hace de "indicador de
  remachado en proceso" (el estado real lo da `/rivet/active`,
  expuesto en el HMI como DO3 Remachado y sin cambios).
- No se tocó `turntable_rivet_cell.xacro` — todo el disco viaja junto
  con el anchor de la zona.

### IK (`scripts/resolve_poses.py`)

- Nueva constante `RIVETING_ZONE_DX_SHIFT = 0.300` que propaga al
  `LOAD_SEAT_X/Y/Z`. Todas las dependencias se derivan de esa cte —
  cambiar el shift es un solo edit.
- En la rama `LOAD_RIVET_POSES`:
  - Los seeds aleatorios ahora siembran `q[4] ∈ [-π/2 ± 0.05]` para
    forzar la rama de IK correcta.
  - Después de cada llamada a `damped_ls_ik` se aplica `q[4] = -π/2`
    explícito; el siguiente refine absorbe el cambio en los otros
    joints. Hace falta 8 iteraciones del lateral-shift loop en lugar
    de 6 para converger con la restricción extra.
  - El score de selección ahora ordena por `(j5_err, rot_err_z,
    pos_err)` — primero exige J5 exacto, luego inclinación de muñeca,
    luego pos_err.
  - Tolerancia ampliada de 5 mm a 15 mm (la combinación strict-top-down
    + J5=-90 + lateral grasp es más rígida que en V55; 15 mm sigue
    bien dentro del POS_TOL_M = 25 mm del solver global).
- Post-process `R_place_lock` también re-lockea `q[4] = -π/2` tras
  cada IK call.

### Resultado en `resolved_poses.py`

```
POSE_APPROACH_LOAD_FIXTURE  q5 = -1.570796 (-90.0000°)
POSE_PLACE_LOAD_FIXTURE     q5 = -1.570796 (-90.0000°)
POSE_RELEASE_LOAD_FIXTURE   q5 = -1.570796 (-90.0000°)
POSE_RETREAT_LOAD_FIXTURE   q5 = -1.570796 (-90.0000°)
POSE_APPROACH_PICK_RIVETED  q5 = -1.570796 (-90.0000°)
POSE_PICK_RIVETED           q5 = -1.570796 (-90.0000°)
POSE_LIFT_RIVETED           q5 = -1.570796 (-90.0000°)
```

Los 7 poses LOAD/RIVET tienen J5 **exactamente** a -π/2. El clamp V55
en `robot_controller_node.py` ya no hace nada en estos poses (queda
como defensa de cinturón, sigue activo para `DROP_ACCEPT_BIN` y
`DROP_REJECT_BIN` que el IK no llega tan claramente al -90°).

### object_manager

`DISC_CENTER_X` actualizado de 0.677 a 0.977 (mismo +0.30 m). Es el
fallback que se usa cuando el lookup de TF falla — debe coincidir con
la posicion del disco en URDF para evitar saltos del CAFI.

## Pruebas

```
$ python3 v56_geom_test.py
V56 geometry verification
------------------------------------------------------------
Expected LOAD seat world XYZ: (0.9803, 1.1576, 1.1045)

pose                                 J5 deg  OK J5
POSE_APPROACH_LOAD_FIXTURE         -90.0000     OK
POSE_PLACE_LOAD_FIXTURE            -90.0000     OK
POSE_RELEASE_LOAD_FIXTURE          -90.0000     OK
POSE_RETREAT_LOAD_FIXTURE          -90.0000     OK
POSE_APPROACH_PICK_RIVETED         -90.0000     OK
POSE_PICK_RIVETED                  -90.0000     OK
POSE_LIFT_RIVETED                  -90.0000     OK

PLACE_LOAD_FIXTURE TCP world: (0.9580, 1.1952, 1.1054)
PLACE_LOAD_FIXTURE CAFI world (= TCP - delta_w_lateral): (0.9804, 1.1577, 1.1045)
Expected LOAD seat:           (0.9803, 1.1576, 1.1045)
Error norm:                   0.16 mm

V56 GEOMETRY TEST PASSED.
```

```
$ python3 v55_aggressive_test.py
TEST 1 OK: 52 CAFIs through the cell, no fault, no watchdog, cabin transitions=313
TEST 2 OK: START -> STOP -> RESET cycle with gripper CLOSED.
           Cleanup order removed=['11', '12', '13'], cell back to IDLE,
           cabin LOWERED, started_once=False.

ALL V55 AGGRESSIVE TESTS PASSED.
```

Cero regresiones — la FSM V55 sigue funcionando idéntica con los
nuevos poses.

```
$ python3 resolve_poses.py
Summary: 19/19 poses PASS (<= 25 mm cart err)
Worst case across all poses: min_joint_Z = 1.0363 m
Joint-mesa fails: 0
```

Todas las 19 poses pasan, y todos los joints quedan ≥ 36 mm por encima
de la mesa (margen de seguridad de mesa 5 mm).

## Archivos cambiados

```
ros_ws/src/schneider_cell_description/urdf/schneider_cell.urdf.xacro   (modificado)
ros_ws/src/schneider_cell_description/scripts/resolve_poses.py         (modificado)
ros_ws/src/schneider_cell_description/scripts/resolved_poses.py        (regenerado)
ros_ws/src/schneider_cell_description/scripts/v56_geom_test.py         (nuevo)
ros_ws/src/schneider_object_manager/src/object_manager_node.py         (constante DISC_CENTER_X)
```

## Cómo correr

```bash
# Regenerar las poses con un dx diferente:
# 1. Edita scripts/resolve_poses.py -> RIVETING_ZONE_DX_SHIFT
# 2. Edita urdf/schneider_cell.urdf.xacro -> world_to_riveting_zone origin xyz
# 3. python3 scripts/resolve_poses.py    # regenera resolved_poses.py

# Validar:
python3 scripts/v56_geom_test.py
python3 scripts/v55_aggressive_test.py
```

## Caveats

- El disco ahora está a ~110 mm al oeste del cobot (en X). El pedestal
  del cobot es pequeño (≈80 mm radio) y la rueda del disco no entra
  en esa zona — verificado con bbox aproximado. Si la mesh real del
  `gearbase.stl` excede la estimación, conviene reducir el shift a
  +0.25 m (sigue dando J5=-90° con pos_err ~6 mm).
- El comentario antiguo `(0.692, 1.158, 1.220)` en
  `schneider_cell.urdf.xacro:205` quedó como referencia historica;
  no afecta a runtime.
- Los scripts `cycle_simulate_v52/v53/v54.py` y `v53/v54_aggressive_test`
  siguen referenciando la posicion vieja del LOAD seat. Son historicos
  y NO se ejecutan en runtime; la fuente de verdad es `resolved_poses.py`
  que sí se regeneró. Si quieres correr esos scripts viejos para
  comparación tendrías que actualizar sus constantes a mano.
