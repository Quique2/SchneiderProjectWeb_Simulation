# schneider_cell_description V62 — cobot V61 con poses recalculadas y zero colisiones

## Por qué V62

V61 dejó el `URDF DEFINITIVO` (link_elbow_connector extraído, joint_3
re-anclado en frame del elbow con `rpy=(0, π, 0)`) como el cobot oficial
de la simulación. Pero las poses V60 fueron resueltas contra la cadena
**vieja** — feedeando esos joint vectors a la nueva FK el grasp center
caía a decenas de mm del target en muchas poses, y la interpolación
entre poses barría disco / bins / cámara.

V62 corrige eso: re-cinemática + re-resolución de poses + verificación
de colisiones contra todos los obstáculos de la planta.

## Cambios

1. **`lexium_kinematics.py`** — inserta el fijo
   `Trpy(ELBOW_CONNECTOR_XYZ, (0, π, 0))` entre `Rz(q[1])` y la
   translación a joint_3, y mueve `JOINT3_XYZ` a `(0.002560, 0.001375,
   0.113592)` (en frame del elbow_connector). Resto del chain (joint_4
   ... tool0) byte-for-byte intacto.
   `fk_joint_origins_world` expone también el origen del elbow.

2. **`v61_polish_poses.py`** (nuevo) — polidor que warm-startea cada
   pose con la config V60 y polishea contra la FK V61. Para casos
   problemáticos (vision close, bins) hace fallback con 120
   perturbaciones aleatorias alrededor de `prev_q`. Ranking ordena por:
   - mesa-safety (joints >= 1.005 m)
   - colisiones estáticas + interpolación a prev_q + interpolación a HOME
   - joints no-en-limite
   - distancia joint-space a prev_q
   - error cartesiano
   Esto produce soluciones IK que son alcanzables, no tocan obstáculos
   estáticamente, y sus interpolaciones a poses vecinas tampoco.

3. **`resolved_poses.py`** — regenerado. **19/19 poses pasan** con
   error cartesiano típicamente < 0.2 mm (worst ~ 8 mm en
   `DROP_REJECT_BIN`). Mesa clearance >= 1.005 m en todos los joints
   de todas las poses (worst min_joint_Z = 1.0466 m).

4. **`validate_robot_poses.txt`** — regenerado con la tabla V61.

## Pruebas — todas pasan

```
$ python3 v61_polish_poses.py    → 19/19 PASS, mesa OK
$ python3 v56_geom_test.py       → FK err 0.04 mm                            PASS
$ python3 v57_collision_test.py  → 0 violaciones (>=3 cm a todo obstáculo)   PASS
                                   (19 poses + 27 segmentos × 10 pasos
                                    interpolados verificados contra los
                                    18 obstáculos del plant)
$ python3 v55_aggressive_test.py → 52 CAFIs, 0 faults, 0 watchdogs           PASS
$ python3 v61_max_cafi_test.py   → TEST A 200 + TEST B 400, 0 issues         PASS
```

Cero colisiones con **nada** en la planta: mesa, conveyor, suministro,
sensores SICK, disco/fixtures rivet, fixture visión, ambos bins,
indicador post, postes de cabina, motor del conveyor, cámara Cognex
(body + columna).

## Archivos cambiados

```
src/schneider_cell_description/scripts/lexium_kinematics.py    (V61 chain)
src/schneider_cell_description/scripts/resolved_poses.py       (regenerado V62)
src/schneider_cell_description/scripts/validate_robot_poses.txt (regenerado V62)
src/schneider_cell_description/scripts/v61_polish_poses.py     (NUEVO)
evidence/v62/*.log                                              (NUEVO)
```
