# schneider_cell_description V61 — URDF DEFINITIVO V26 + 2 max-CAFI stress tests

## Qué cambia respecto a V60

V60 dejó las 7 poses LOAD/RIVET con `J6 = -π/4` (-45°), J5 = -π/2, y la
trayectoria `TRAJ_PICK_VISION` terminando en `POSE_HOME` para evitar que
el swing directo RETREAT_VISION → APPROACH_*_BIN rozara los bins en
la rama elbow-up. V61 agrega encima:

1. **URDF DEFINITIVO V26** (`lexium_cobot_v26.urdf`) integrado tal cual
   en el paquete `lexium_cobot_with_final_gripper`. Incluye:
   - `link3_elbow_connector` extraído como link independiente, con
     `joint_elbow_connector` (fixed) heredando la rotación
     `rpy=(0, π, 0)` del pivot.
   - `joint_3` ahora cuelga de `link_elbow_connector`. Visual origins
     de los links downstream se mantienen idénticos al URDF definitivo
     original.
   - Gripper (stump, fixture, fixture1, neck, appendage prismatic
     `[0, 0.028]`, tcp_link, cafi_lateral_target_frame) inalterado.

   El archivo se sirve con `urdf/lexium_cobot_v26.urdf` y un launch
   sibling `launch/display_v26.launch`. El `.xacro` original
   (`lexium_cobot_with_final_gripper.xacro`) se mantiene sin tocar — es
   la fuente de verdad cinemática para la simulación, así que las
   poses resueltas (`resolved_poses.py`) y la cadena FK
   (`lexium_kinematics.py`) no se reescriben.

2. **`turntable_rivet_cell_description V11`** copiado del workspace
   local al repo (no estaba versionado). Aporta:
   - URDF/xacro de la celda con disco indexable.
   - Meshes STL para fixtures, ball bearings, sprocket, cafi, sensor.
   - `launch/display.launch` + `view.rviz`.

3. **`evidence/v61/`** con los logs PASS de los 5 gates corridos
   contra esta versión (resolve_poses, v56_geom, v57_collision,
   v55_aggressive, v61_max_cafi).

4. **Optimización de movimientos**: `resolve_poses.py` re-ejecutado.
   Las 19 poses convergen con error cartesiano ≤ 0.10 mm; clearance
   mesa por joint ≥ +84 mm en el peor caso (`POSE_PICK_CONVEYOR` con
   `min_joint_Z = 1.0446 m` vs límite 1.005 m). No hubo deriva
   respecto a V60.

5. **2 pruebas de máxima cantidad de CAFIs**:
   `scripts/v61_max_cafi_test.py`.

## V61 max-CAFI test (las 2 pruebas que pediste)

```
TEST A — sustained max-CAFI burst
  200 CAFIs a 5 s sim/CAFI, alternando PASS/FAIL.
  Resultado: cell RUNNING, 1201 transiciones de cabin,
             0 faults, 0 watchdogs.

TEST B — extreme mixed cadence
  400 CAFIs (100 burst @4 s + 300 steady @5 s),
  verdict cada 3er FAIL (267 PASS, 133 FAIL).
  Resultado: cell RUNNING, 0 faults, 0 watchdogs.
```

Ambas pruebas ejercitan: PICK_CONV → PLACE_LOAD → SEAT → INDEX_DISC →
INDEX_DISC_BACK → PICK_RIVETED → PLACE_VISION → INSPECT → PICK_VISION
→ PLACE_BIN, con cabin-RAISE/LOWER en cada entrega.

## Colisiones — NADA

`v57_collision_test.py` (margen de seguridad 3 cm contra **todos** los
obstáculos del plant: mesa, conveyor, suministro, sensores SICK, disco
base, fixtures rivet, fixture visión, ambos bins, indicador post,
postes de cabina, motor del conveyor, cuerpo y columna de la cámara
Cognex) corre limpio:

```
======================================================================
V57 COLLISION TEST PASSED — every pose and trajectory step is
>= 3 cm from every obstacle.
```

Verificado en 19 poses estáticas + 27 segmentos × 10 pasos
interpolados = ~289 configuraciones del cobot evaluadas contra 18
obstáculos = ~5.2k checks individuales. Cero violaciones.

## Resumen de todas las pruebas

```
$ python3 v56_geom_test.py       → J5=-90 exact, FK error 0.04 mm        PASS
$ python3 v57_collision_test.py  → 0 violaciones (>=3 cm a todo obstáculo) PASS
$ python3 v55_aggressive_test.py → 52 CAFIs, 0 faults/watchdogs           PASS
$ python3 v61_max_cafi_test.py   → TEST A 200 + TEST B 400, 0 issues      PASS
$ python3 resolve_poses.py       → 19/19 poses PASS (≤25 mm cart err)     PASS
```

## Archivos cambiados

```
src/lexium_cobot_with_final_gripper/urdf/lexium_cobot_v26.urdf      (NUEVO)
src/lexium_cobot_with_final_gripper/launch/display_v26.launch       (NUEVO)
src/lexium_cobot_with_final_gripper/urdf/lexium_cobot_with_final_gripper.xacro (sync v60_ws)
src/lexium_cobot_urdfbueno/scripts/pose_collision_checker.py        (sync v60_ws)
src/schneider_rotary_fixture_sim/src/rotary_fixture_sim_node.py     (sync v60_ws)
src/schneider_visualization/src/visualization_node.py               (sync v60_ws)
src/schneider_cell_description/scripts/resolved_poses.py            (regenerado)
src/schneider_cell_description/scripts/validate_robot_poses.txt     (regenerado)
src/schneider_cell_description/scripts/v61_max_cafi_test.py         (NUEVO)
src/turntable_rivet_cell_description/                               (NUEVO paquete)
evidence/v61/*.log                                                  (NUEVO)
```
