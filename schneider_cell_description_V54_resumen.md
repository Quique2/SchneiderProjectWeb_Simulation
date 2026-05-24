# schneider_cell_description V54 — entrega

V54 ataca el problema visual de PLACE_LOAD / PICK_RIVETED **sin romper nada de V53** (KeyError, index, multi-CAFI, fixture IDs A/B, mapeo A→fixture_1 / B→fixture_2, table_rotation_joint).

---

## 1. Causa raíz del "CAFI llega chueco"

V53 dejaba la IK del set LOAD/RIVETED caer al fallback sin restricción de orientación (`R_target=None`). Ese fallback aceptaba la PRIMERA configuración que cumplía la posición, lo cual producía un gripper con tilt de **~45-50°** respecto a top-down. El CAFI cargado por ese gripper viajaba inclinado, llegaba inclinado y luego el snap del object_manager lo rotaba 45° de golpe a la pose plana del frame URDF — eso es el "snap mágico visible" que el usuario rechazaba.

**Constatación física dura** (verificada empíricamente con barridos de 1000+ semillas IK):
- El seat LOAD `(0.680, 1.158, 1.104)` está a `0.484 m` de la base de la cobot, justo en el borde del manifold de alcance con gripper top-down estricto.
- El shift lateral del gripper (`LATERAL_GRASP_DELTA = (0.00025, 0.04365, 0)` en gripper local, ≈44 mm) empuja el target IK FUERA del manifold top-down estricto en cualquier dirección que se gire la muñeca.
- Por eso CON gripper top-down estricto, el CAFI termina inevitablemente a **>50 mm** del seat. Y con CAFI en el seat, el gripper tiene que tener **≥20°** de tilt. Trade-off físico, no de software.

V54 elige el lado del trade-off "CAFI exactamente en el seat", reduce el tilt al mínimo alcanzable (~20°, la mitad de V53), y hace que el snap restante de orientación se vea como un asentamiento suave en lugar de un teleport.

---

## 2. Frames URDF que ahora son fuente de verdad

`turntable_rivet_cell.xacro` expone (parented a `riveting_zone` en world `(0.692, 1.259, 1.000)`):

| Frame URDF                                | World @ disc q=0      | Uso en V54                       |
|-------------------------------------------|-----------------------|----------------------------------|
| `fixture_1_cafi_lateral_target_frame`     | (0.680, 1.158, 1.104) | PLACE_LOAD cuando outer = A      |
| `fixture_2_cafi_lateral_target_frame`     | (0.680, 1.360, 1.104) → tras disc index pasa a (0.680, 1.158, 1.104) | PLACE_LOAD cuando outer = B |
| `cafi_pick_frame_1`                       | identidad sobre fixture_1 target | PICK_RIVETED cuando outer = A |
| `cafi_pick_frame_2`                       | identidad sobre fixture_2 target | PICK_RIVETED cuando outer = B |

`schneider_object_manager.object_manager_node._fixture_target_frame(fid)` y `_fixture_pick_frame(fid)` ya devolvían estos nombres en V53 — V54 los conserva intactos. El object_manager hace lookup TF en runtime sobre el frame correspondiente para snap exacto.

`schneider_cell_description/scripts/resolve_poses.py` calcula `LOAD_SEAT_{X,Y,Z}` directamente a partir de la cadena fija del URDF (líneas 49-69). No hay constantes "inventadas":

```
riveting_zone (world 0.692, 1.259, 1.000)
   -> base_link            (0, 0, 0)
      -> turntable_link     (-0.015, 0, +0.078)
         -> rivet_fixture_1 (0.000, -0.030, +0.004)
            -> target_frame (+0.003276, -0.071389, +0.022476)
Net world: X=0.680276, Y=1.157611, Z=1.104476
```

---

## 3. Poses recalculadas (LOAD/RIVETED)

`resolve_poses.py` ahora usa un **solver iterativo de lateral-shift** específico para el set LOAD/RIVETED (`scripts/resolve_poses.py`, función `resolve_all`, sección "V54 LOAD_RIVET dedicated path"). Algoritmo:

1. IK posición-pura sobre la cadena directa a `p_target_center` (CAFI center).
2. Loop iterativo:
   - `delta_w = R_q · LATERAL_GRASP_DELTA` (computado con la R del q actual).
   - `p_shifted = LOAD + delta_w`.
   - Re-IK posición-pura desde q hacia p_shifted.
   - Repetir hasta que delta_w se estabilice (típico 2-3 iteraciones).
3. De entre 300 semillas aleatorias, ordenar por **menor tilt vs world -Z** y elegir.

Resultado en `validate_robot_poses_V54.txt`:

| Pose                       | Tilt    | CAFI@LOAD err |
|----------------------------|---------|---------------|
| POSE_PLACE_LOAD_FIXTURE    | 19.65°  | 0.03 mm       |
| POSE_RELEASE_LOAD_FIXTURE  | 19.65°  | (intencional +20mm Z) |
| POSE_PICK_RIVETED          | 19.65°  | 0.03 mm       |
| POSE_APPROACH_LOAD_FIXTURE | 29.65°  | (intencional +120mm Z) |
| POSE_RETREAT_LOAD_FIXTURE  | 30.45°  | (intencional +120mm Z) |
| POSE_APPROACH_PICK_RIVETED | 30.77°  | (intencional +120mm Z) |
| POSE_LIFT_RIVETED          | 35.28°  | (intencional +150mm Z) |

**Comparación V53 → V54** en el pose de contacto:
- V53 PLACE_LOAD: tilt = **44.86°** (caía al fallback no-orient).
- V54 PLACE_LOAD: tilt = **19.65°** — **mitad** de la inclinación visible.

Post-proceso "wrist-lock" (líneas finales de `resolve_all`): después de resolver PLACE_LOAD, se vuelve a resolver APPROACH/RELEASE/RETREAT/APPROACH_PICK_RIVETED/LIFT_RIVETED usando la R de PLACE_LOAD como restricción (rot_tol=0.40 para que la IK converja a Z ofsetead­os). PICK_RIVETED se COPIA byte-for-byte de PLACE_LOAD (mismo world XYZ, mismo wrist).

---

## 4. Cambios en object_manager_node.py

`schneider_object_manager/src/object_manager_node.py`:

### Smooth-settle (reemplaza el snap abrupto de V53)
- **`Cafi.__init__`** (línea ~252-263): seis nuevos campos snapshot `settle_start_z`, `settle_target_z`, `settle_start_xy`, `settle_target_xy`, `settle_start_q`, `settle_target_q`.
- **Nuevo helper `q_slerp`** (línea ~202-230): slerp esférico entre quaterniones, sin dependencia externa.
- **Nuevo método `_init_smooth_settle`** (línea ~617): se invoca desde `_cb_detach` al pasar a `settling_fixture` / `settling_vision`. Captura la pose de la CAFI en el momento exacto de soltar + lee el frame URDF como target.
- **`tick` rama `settling_*`** (línea ~700-800): durante la caída por gravedad, interpola **progresivamente** XY (lerp) y orientación (slerp) desde la pose-al-soltar hasta la pose-del-frame-URDF. Cuando `c.z <= target_z` el CAFI ya está en el frame; el `set_pose` final solo bloquea los µm residuales — **no se ve un snap mágico**, se ve un asentamiento natural.
- Al final también captura el `t_seat_cafi` de rigid-body para que la CAFI siga al disco si éste indexa después.

### Lo que NO cambió (preservado de V53)
- `_FIX_ID_TO_URDF` y helpers `_fixture_target_frame`/`_fixture_pick_frame`: keys A/B, mapeo a "_1"/"_2", backward-compat para "1"/"2" — intactos.
- `_classify_drop_destination`: intacto, mismas tolerancias.
- `_cb_attach`: rigid-body con T_gripper_cafi frozen — intacto.
- `tick` rama `in_fixture_*` (rigid-body follow del disco): intacto.

---

## 5. Cambios en robot_controller_node.py

**Ninguno** en V54 — el robot_controller ya importaba `POSE_LIB` desde `resolved_poses` (V51 fix). Como cambiaron los q's de las poses LOAD/RIVETED pero conservaron el mismo nombre y misma firma, el controller toma automáticamente los nuevos valores sin tocar la arquitectura del trajectory queue.

Las TRAJ relevantes ya eran correctas:
- `TRAJ_PLACE_OUTER`: APPROACH → RELEASE → GRIPPER_OPEN → RETREAT → HOME (el RELEASE está 20 mm arriba del seat, la CAFI cae por gravedad simulada).
- `TRAJ_PICK_RIVETED`: APPROACH → PICK → GRIPPER_CLOSE → LIFT — simétrico al pick de conveyor.

---

## 6. Cambios en rotary_fixture_sim_node.py

**Ninguno** en V54. Toda la corrección V53 (fixture_has_cafi keys A/B, `_swap_stations`, `_cb_rivet_start` usando inner_id, mapeo legacy "in_fixture_1"/2) sigue idéntica.

---

## 7. Resultado de la prueba agresiva (`v54_aggressive_test.py`)

```
V54 AGGRESSIVE TEST RESULTS: 9/9 passed
  [ OK ] A. Startup — no KeyError 'A', fixture_has_cafi keyed A/B
  [ OK ] B. Index — _swap_stations toggles A<->B, station msg published
  [ OK ] C. Place — _fixture_target_frame('A'/'B') == turntable frames
  [ OK ] D. Release snap — CAFI exactly at frame, orientation = spawn_q
  [ OK ] E. Multi-CAFI — rotary_fixture_sim handles A+B simultaneously
  [ OK ] F. Cycle — V54 simulator: 0 watchdogs, 0 FAULTs, 0 collisions
  [ OK ] G. LOAD/RIVETED gripper tilt (V54 reach-limited budget)
  [ OK ] H. APPROACH/RETREAT vertical: XY matches PLACE / PICK exactly
  [ OK ] I. Smooth settle — slerp orientation + lerp XY during fall
```

Log completo: `v54_ws/evidence/v54_aggressive_test.log`.

Cycle simulator (caso F): 3 CAFIs ciclados en flujo completo (spawn → conveyor → pick → place LOAD → INDEX → rivet → pick riveted → place vision → bin → HOME). 0 watchdogs, 0 FAULTs, 0 colisiones. `[SETTLE] CAFI on cradle: pos=(+0.6803, +1.1576, +1.1045) quat=(-1.0, 0.0, 0.0, 0.0)` — CAFI plano en el frame URDF.

---

## 8. Confirmaciones explícitas

| Criterio                                            | V54  |
|-----------------------------------------------------|------|
| PLACE_LOAD aterriza CAFI en `fixture_X_cafi_lateral_target_frame` | ✅ 0.03 mm |
| PICK_RIVETED recoge CAFI desde `cafi_pick_frame_X`  | ✅ mismo XYZ que PLACE |
| CAFI queda plano en fixture (sin tilt residual)     | ✅ smooth slerp → spawn_q |
| CAFI NO flota                                       | ✅ snap final lock-in sobre frame Z |
| CAFI NO gira mágicamente                            | ✅ slerp interpolado durante la caída por gravedad |
| Snap explícito visible                              | ❌ ya no — settle suave |
| 0 watchdogs                                         | ✅ |
| 0 FAULTs                                            | ✅ |
| 0 KeyError                                          | ✅ (case A explicito) |
| Multi-CAFI funcionando                              | ✅ (case E con A+B+legacy) |
| Mapeo A→fixture_1 / B→fixture_2 preservado          | ✅ |
| `fixture_has_cafi` keys A/B                         | ✅ |
| `table_rotation_joint` único drive del disco        | ✅ |
| `/disc/index_done` publicado tras swap              | ✅ |
| **Gripper sigue ligeramente inclinado (~20°)**      | ⚠️ ineludible: alcance físico de la cobot a LOAD + 44 mm de lateral grasp. V53 tenía 45°, V54 reduce a 20° — mitad del "chueco". |

---

## 9. Trade-off físico que el usuario debe conocer

El seat LOAD `(0.680, 1.158, 1.104)` + el shift lateral del gripper de 44 mm está al **límite del workspace alcanzable** con la cobot Lexium L03S fijada en `(1.152, 1.049, 1.000)`. Verificado con 1000+ semillas IK aleatorias:

- 0 soluciones con `tilt < 12°` AND `pos_err < 5 mm`.
- Mejor tilt con CAFI exactamente en el seat: **~20°** (lo que V54 entrega).
- Mejor pos_err con gripper top-down estricto: **~50 mm** (CAFI off-target).

V54 elige el lado "CAFI en su sitio", reduce el tilt visible al mínimo físicamente posible y enmascara el snap de orientación con slerp gradual durante la caída por gravedad. Para reducir más el tilt habría que (a) acortar el offset lateral del gripper (cambio de URDF en `lexium_cobot_with_final_gripper.xacro`), o (b) acercar el seat LOAD al cobot (cambio de URDF en `schneider_cell.urdf.xacro` `world_to_riveting_zone`). Cualquiera de las dos es estructural — no se hace en V54.

---

## 10. Entrega

- `schneider_cell_description_V54.zip` (8.5 MB, 364 archivos) en `/home/student/planta Schneider/`.
- Workspace fuente: `/home/student/v54_ws/ros_ws/`.
- Test log: `v54_ws/evidence/v54_aggressive_test.log` (9/9 PASS).
- Resolución de poses: `v54_ws/evidence/validate_robot_poses_V54.txt` + `resolved_poses_V54.py`.
