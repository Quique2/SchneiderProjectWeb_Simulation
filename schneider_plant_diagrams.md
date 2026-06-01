# Diagramas de flujo — Planta Schneider

Documentación visual de la lógica de control de la planta automatizada con cobot, mesa giratoria, conveyor y sistema de inspección por visión.

---

## Diagramas incluidos

### 1. Flujo principal de producción

Ciclo completo desde que el operador presiona START hasta que un CAFI termina en el bin de aceptado o rechazado.

**Etapas principales:**
- Planta arranca en IDLE; pasa a RUNNING al presionar START
- Dispensador suelta CAFI → conveyor lo mueve al sensor fotoeléctrico
- Cobot hace pick del conveyor y coloca en fixture externo
- Mesa gira 180° → CAFI pasa a zona de remachado
- Remachado dura 30 segundos
- Cobot recoge CAFI remachado (prioridad absoluta) y lo lleva a inspección
- Cámara determina PASS o FAIL → cobot deposita en bin correspondiente
- Ciclo se repite

**Condiciones de bloqueo principales:**
- Conveyor no avanza si sensor está ocupado o hay 2 CAFIs esperando
- Cobot no hace pick si existe prioridad de CAFI remachado pendiente
- Mesa no gira si cobot está en zona de riesgo o remachado está activo

---

### 2. Seguridad: STOP, RESUME y RESTART

Lógica de pausa y recuperación ante intervención del operador.

| Acción | Resultado |
|---|---|
| STOP | Planta → PAUSED. Todo se detiene. |
| RESUME | Reanuda desde el estado anterior (si es seguro). |
| RESTART | Cancela el ciclo. El cobot deposita pieza en rechazo (si la tiene) y regresa a HOME. |

**Estado CLEANING_REQUIRED:**  
Se activa cuando tras un RESTART quedan CAFIs dentro de la planta. El operador debe retirarlos manualmente y confirmar antes de que la planta pueda volver a IDLE.

Si hay CAFI atrapado en el disco/remachado, la mesa entra en `TABLE_RECOVERY_POSITIONING` para facilitar el retiro manual.

---

### 3. Reglas de bloqueo e interlocks

Condiciones que deben cumplirse para que cada subsistema pueda actuar.

#### Conveyor puede avanzar si:
- No hay STOP / FAULT / CLEANING activo
- Sensor de conveyor no está ocupado
- No hay ya 2 CAFIs esperando

#### Cobot puede hacer pick del conveyor si:
- Sensor de conveyor detecta CAFI
- Fixture externo está libre (sensor)
- Cobot está libre y sin pieza en gripper
- Mesa está detenida y zona es segura
- No hay prioridad de CAFI remachado pendiente
- No hay STOP / FAULT / CLEANING activo

#### Mesa puede girar si:
- Cobot no está en zona de riesgo
- Cobot no está colocando/retirando pieza
- Remachado no está activo
- No hay STOP / FAULT / CLEANING activo
- Hay presencia confirmada en fixture externo

#### Remachado puede iniciar si:
- Mesa está en limit switch WORK (180°)
- Sensor de fixture de remachado detecta CAFI
- Cobot está fuera de la zona
- Mesa está detenida
- No hay STOP / FAULT / CLEANING activo

#### START permitido si:
- Planta está en IDLE
- Sensores están limpios y celda lista
- No hay STOP / FAULT / CLEANING activo

---

### 4. Estados del CAFI (máquina de estados)

```
DISPENSED
  └─→ ON_CONVEYOR_WAITING
        └─→ AT_SENSOR
              └─→ IN_GRIPPER
                    └─→ IN_OUTSIDE_FIXTURE
                          └─→ IN_RIVET_FIXTURE
                                └─→ RIVETING
                                      └─→ RIVETED
                                            └─→ IN_GRIPPER
                                                  └─→ IN_INSPECTION
                                                        ├─→ INSPECTED_PASS → ACCEPTED_BIN → DONE
                                                        └─→ INSPECTED_FAIL → REJECTED_BIN → DONE
```

Cualquier estado antes de RIVETING puede transicionar a `MANUAL_REMOVAL_REQUIRED` si se ejecuta un RESTART.

---

### 5. Estados de la planta

```
IDLE ──START──→ RUNNING ──STOP──→ PAUSED
                                    ├──RESUME──→ RUNNING
                                    └──RESTART──→ RESTARTING
                                                    └──→ CLEANING_REQUIRED
                                                              └──→ IDLE

RUNNING / PAUSED / RESTARTING ──error──→ FAULT
FAULT ──→ CLEANING_REQUIRED ──→ IDLE
FAULT ──→ IDLE (si no hay piezas y operador confirma)
```

---

### 6. Secuencia HMI

Diagrama de interacción entre actores del sistema en un ciclo normal:

```
Operador → Conveyor → Sensor → Cobot → Fixture → Mesa → Remachado → Cámara → Bins
```

1. Operador presiona START
2. Coloca CAFI en conveyor
3. Sensor detecta → conveyor se detiene
4. Cobot hace pick y coloca en fixture
5. Mesa gira 180°
6. Remachado activo 30 seg
7. Mesa posiciona CAFI para retiro
8. Cobot lleva a cámara
9. Cámara inspecciona → PASS/FAIL
10. Cobot deposita en bin → ciclo terminado

---

## Notas de implementación

- El **cobot** tiene prioridad absoluta cuando tiene un CAFI remachado: ningún otro pick se ejecuta hasta depositarlo en inspección.
- El **sensor del fixture externo** es crítico: si no confirma presencia tras el place del cobot, la planta entra en FAULT y bloquea el giro de la mesa.
- El **límite de 2 CAFIs** esperando en el conveyor es un interlock de seguridad para evitar colisiones o apilamiento.
- El **estado CLEANING_REQUIRED** no se puede saltar: requiere confirmación manual del operador con todos los sensores libres.
