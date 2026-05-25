# schneider_cell_description V58 — J6 alineado al cradle del fixture

## Cambio único respecto a V57

Quité el lock J6 = J6_PICK_CONVEYOR dentro del grupo LOAD/RIVET para
que el IK ahora elija el J6 natural que **alinea la orientación de la
CAFI con el cradle del fixture de remachado**, igual que hace el
fixture de cámara.

V57 dejaba J6 = -135.99° en todo el grupo LOAD/RIVET para no rotar la
muñeca durante el carry — pero el lado bueno (no rotation in carry) se
pagó con la CAFI quedando rotada ~150° fuera de la forma del cradle.
Visualmente la pieza no encajaba en el fixture.

V58:
- En LOAD/RIVET el IK queda libre en J6 (sólo J5 = -π/2 sigue forzado).
- El IK elige naturalmente J6 = +0.256 rad (+14.65°), que es la orientación
  donde el lateral grasp delta deposita la CAFI alineada con el cradle.
- El post-process propaga ese J6 a las 7 poses LOAD/RIVET (APPROACH,
  PLACE, RELEASE, RETREAT, APPROACH_PICK_RIVETED, PICK_RIVETED,
  LIFT_RIVETED) → **dentro del grupo NO hay rotación de muñeca**, la
  CAFI queda fija en la misma orientación desde APPROACH hasta LIFT.
- El twist de muñeca ahora ocurre entre LIFT_CONVEYOR (J6=-136°) y
  APPROACH_LOAD (J6=+14.65°): un giro de **150.6°** durante el viaje
  largo entre el conveyor y el fixture. Visualmente OK porque la CAFI
  no está cerca de ningún cradle durante ese viaje. Es el mismo patrón
  que ya usaban las poses de vision y bins.

| Pose | J6 V57 | J6 V58 |
|---|---|---|
| PICK_CONVEYOR | -2.373 | -2.373 (sin cambio) |
| LIFT_CONVEYOR | -2.373 | -2.373 (sin cambio) |
| **APPROACH_LOAD** | -2.373 | **+0.256** (cradle-aligned) |
| **PLACE_LOAD** | -2.373 | **+0.256** (cradle-aligned) |
| **RELEASE_LOAD** | -2.373 | **+0.256** |
| **RETREAT_LOAD** | -2.373 | **+0.256** |
| **APPROACH_PICK_RIVETED** | -2.373 | **+0.256** |
| **PICK_RIVETED** | -2.373 | **+0.256** |
| **LIFT_RIVETED** | -2.373 | **+0.256** |

## Pruebas (todas pasan)

```
$ python3 v56_geom_test.py            # V56 J5=-90 check
J5 = -90.0000° en las 7 poses LOAD/RIVET ✓
PLACE_LOAD CAFI error 0.16 mm
V56 GEOMETRY TEST PASSED.

$ python3 v57_collision_test.py       # V57 3 cm safety
0 violaciones (19 poses + 27 segmentos × 10 pasos = ~280 configs)
V57 COLLISION TEST PASSED.

$ python3 v55_aggressive_test.py      # V55 throughput + button flow
TEST 1: 52 CAFIs, cero faults/watchdogs
TEST 2: START -> STOP -> RESET con cleanup correcto
ALL V55 AGGRESSIVE TESTS PASSED.

$ python3 resolve_poses.py            # Pose solver
Summary: 19/19 poses PASS (<= 25 mm cart err)
```

## Por qué J6 = +0.256 alinea la CAFI con el cradle

El fixture_1_cafi_lateral_target_frame en la URDF está rotado a un
yaw específico relativo a riveting_zone (más el yaw del disco si
indexea). La CAFI tiene una forma asimétrica (123 x 88 x 25 mm) que
sólo encaja en el cradle si su eje largo está alineado con la
geometría del cradle.

El IK con strict top-down + J5=-π/2 + lateral_grasp_delta como
constraint NATURALMENTE converge a un J6 que coloca el TCP en la
posición lateral correcta — y por construcción ese J6 es exactamente
el que orienta la CAFI con el cradle. Forzar J6 a un valor arbitrario
(como hizo V57) rompía esa alineación.

Esto es exactamente la misma lógica que ya usaba la pose de vision
(POSE_PLACE_VISION J6 = +0.113, IK natural) — V58 vuelve a esa
filosofía para LOAD también.

## Archivos cambiados

```
ros_ws/src/schneider_cell_description/scripts/resolve_poses.py       (J6 lock removido)
ros_ws/src/schneider_cell_description/scripts/resolved_poses.py      (regenerado)
ros_ws/src/schneider_cell_description/scripts/validate_robot_poses.txt (regenerado)
```

Sólo 3 archivos cambiados — todos los cambios físicos del mundo
(URDF, conveyor +30 cm, disco +30 cm, indicador, etc.) se mantienen
exactos de V57.

## Notas

- El twist de 150° entre LIFT_CONVEYOR y APPROACH_LOAD se ejecuta
  con el interpolador del robot_controller (cosine-eased, max
  0.9 rad/s por joint). A esa velocidad un giro de 2.63 rad toma
  ~2.9 s. Bien.
- El collision test V57 verifica las trayectorias interpoladas con
  10 pasos por segmento, así que el muñeco rotando 150° durante el
  traverse fue sampleado y NO chocó con nada.
- Los bins (DROP_ACCEPT/REJECT) ya tenían J6 libre y siguen igual.
