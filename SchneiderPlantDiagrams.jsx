import { useState, useEffect, useRef } from "react";

const DIAGRAMS = [
  {
    label: "Flujo principal",
    code: `flowchart TD
A([Inicio planta]) --> B{¿Operador presionó START?}
B -- No --> IDLE[Planta en IDLE]
IDLE --> B
B -- Sí --> C[Planta en RUNNING]
C --> D{¿Hay CAFI disponible?}
D -- No --> WAIT_CAFI[Esperar CAFI]
WAIT_CAFI --> D
D -- Sí --> E{¿Conveyor puede aceptar CAFI?}
E -- No --> E_BLOCK[Conveyor bloqueado]
E_BLOCK --> D
E -- Sí --> F[Dispensador suelta CAFI]
F --> G[Conveyor mueve CAFI al sensor]
G --> H{¿Sensor detecta CAFI?}
H -- No --> G
H -- Sí --> I[Conveyor se detiene]
I --> J{¿Fixture externo libre?}
J -- No --> J_WAIT[CAFI espera en conveyor]
J_WAIT --> J
J -- Sí --> K{¿Cobot libre y seguro?}
K -- No --> K_WAIT[Esperar cobot]
K_WAIT --> K
K -- Sí --> L{¿Prioridad CAFI remachado?}
L -- Sí --> PRIORITY[Atender remachado primero]
PRIORITY --> J
L -- No --> M[Señal: CAFI listo para pick]
M --> N[Cobot ejecuta PICK]
N --> O[Cobot toma CAFI]
O --> P[Sensor conveyor queda libre]
P --> Q[Conveyor espera 2 seg]
Q --> R{¿Otro CAFI esperando?}
R -- Sí --> G
R -- No --> S[Conveyor en espera]
O --> T[Cobot lleva CAFI al fixture externo]
T --> U{¿Mesa detenida?}
U -- No --> U_WAIT[Esperar mesa]
U_WAIT --> U
U -- Sí --> V[Cobot coloca CAFI]
V --> W{¿Sensor confirma CAFI?}
W -- No --> FAULT[FAULT]
W -- Sí --> X[CAFI confirmado]
X --> Y{¿Puede girar la mesa?}
Y -- No --> Y_WAIT[Esperar condiciones]
Y_WAIT --> Y
Y -- Sí --> Z[Mesa gira 180°]
Z --> AA[Fixtures cambian posición]
AA --> AB{¿Sensor remachado detecta CAFI?}
AB -- No --> NEXT{¿CAFI remachado accesible?}
AB -- Sí --> AC[Iniciar remachado]
AC --> AD[Esperar 30 segundos]
AD --> AE[Remachado terminado]
AE --> AF[Mesa posiciona CAFI para retiro]
AF --> AG{¿Cobot libre?}
AG -- No --> AG_WAIT[Esperar cobot]
AG_WAIT --> AG
AG -- Sí --> AH[Cobot recoge CAFI remachado]
AH --> AI[CAFI = prioridad absoluta]
AI --> AJ[Cobot lleva a inspección]
AJ --> AK[Coloca en zona de visión]
AK --> AL{¿Sensor visión detecta CAFI?}
AL -- No --> FAULT
AL -- Sí --> AM[Cámara inspecciona]
AM --> AN[Esperar resultado]
AN --> AO{¿Resultado?}
AO -- PASS --> AP[Cobot → bin aceptado]
AO -- FAIL --> AQ[Cobot → bin rechazado]
AP --> AR[CAFI DONE]
AQ --> AR
AR --> AS{¿Planta en RUNNING?}
AS -- No --> IDLE
AS -- Sí --> AT{¿CAFI en conveyor?}
AT -- Sí --> J
AT -- No --> WAIT_CAFI
NEXT -- Sí --> AH
NEXT -- No --> AT`,
  },
  {
    label: "STOP / RESUME",
    code: `flowchart TD
A([Planta en operación]) --> B{¿Operador presiona STOP?}
B -- No --> A
B -- Sí --> C[STOP activo]
C --> D[Conveyor, Mesa, Cobot, Remachado: detenidos]
D --> E[Planta en PAUSED]
E --> F{¿RESUME o RESTART?}
F -- RESUME --> G{¿Es seguro continuar?}
G -- No --> G_BLOCK[RESUME bloqueado]
G_BLOCK --> F
G -- Sí --> H[Reanudar desde estado anterior]
H --> I[Planta vuelve a RUNNING]
F -- RESTART --> J[Cancelar ciclo automático]
J --> K{¿Cobot sostiene CAFI?}
K -- Sí --> L[COBOT_RECOVERY_TO_REJECT]
L --> M[Cobot va lento a bin rechazo]
M --> N[Deposita pieza, gripper abre]
N --> O[Cobot regresa a HOME]
K -- No --> P[COBOT_RECOVERY_HOME_SLOW]
P --> O
O --> Q{¿CAFIs dentro de planta?}
Q -- No --> R[Planta puede ir a IDLE]
Q -- Sí --> S[CLEANING_REQUIRED]
S --> T{¿CAFI en disco/remachado?}
T -- Sí --> U[TABLE_RECOVERY_POSITIONING]
U --> V[Mesa posiciona para retiro manual]
T -- No --> W[Esperar retiro manual]
V --> W
W --> X[Operador retira CAFIs]
X --> Y{¿Operador confirma limpieza?}
Y -- No --> W
Y -- Sí --> R
R --> Z[Planta en IDLE]`,
  },
  {
    label: "Interlocks",
    code: `flowchart TD
A([Solicitud de acción]) --> B{¿Qué acción?}
B -->|Conveyor avanzar| C1{¿STOP/FAULT/CLEANING?}
C1 -- Sí --> CB[Bloquear conveyor]
C1 -- No --> C2{¿Sensor ocupado?}
C2 -- Sí --> CB
C2 -- No --> C3{¿Ya hay 2 CAFIs esperando?}
C3 -- Sí --> CB
C3 -- No --> C_OK[Conveyor avanza]
B -->|Cobot pick| D1{¿Sensor conveyor ocupado?}
D1 -- No --> DB[Bloquear pick]
D1 -- Sí --> D2{¿Fixture externo libre?}
D2 -- No --> DB
D2 -- Sí --> D3{¿Cobot libre?}
D3 -- No --> DB
D3 -- Sí --> D4{¿Mesa detenida?}
D4 -- No --> DB
D4 -- Sí --> D5{¿Prioridad remachado?}
D5 -- Sí --> DB
D5 -- No --> D6{¿STOP/FAULT?}
D6 -- Sí --> DB
D6 -- No --> D_OK[Cobot pick ejecuta]
B -->|Mesa girar| E1{¿Cobot en zona riesgo?}
E1 -- Sí --> EB[Bloquear giro]
E1 -- No --> E2{¿Remachado activo?}
E2 -- Sí --> EB
E2 -- No --> E3{¿STOP/FAULT?}
E3 -- Sí --> EB
E3 -- No --> E4{¿Presencia en fixture externo?}
E4 -- No --> EB
E4 -- Sí --> E_OK[Mesa gira 180°]
B -->|Remachado| F1{¿Mesa en limit WORK?}
F1 -- No --> FB[Bloquear remachado]
F1 -- Sí --> F2{¿Sensor fixture detecta CAFI?}
F2 -- No --> FB
F2 -- Sí --> F3{¿Cobot fuera de zona?}
F3 -- No --> FB
F3 -- Sí --> F4{¿STOP/FAULT?}
F4 -- Sí --> FB
F4 -- No --> F_OK[Remachado activo 30s]
B -->|START| G1{¿Planta en IDLE?}
G1 -- No --> GB[Bloquear START]
G1 -- Sí --> G2{¿Sensores limpios?}
G2 -- No --> GB
G2 -- Sí --> G3{¿STOP/FAULT?}
G3 -- Sí --> GB
G3 -- No --> G_OK[Planta a RUNNING]`,
  },
  {
    label: "Estados CAFI",
    code: `stateDiagram-v2
[*] --> DISPENSED
DISPENSED --> ON_CONVEYOR_WAITING
ON_CONVEYOR_WAITING --> AT_SENSOR : llega al sensor
AT_SENSOR --> IN_GRIPPER : cobot pick
IN_GRIPPER --> IN_OUTSIDE_FIXTURE : cobot place
IN_OUTSIDE_FIXTURE --> IN_RIVET_FIXTURE : mesa gira 180°
IN_RIVET_FIXTURE --> RIVETING : inicia remachado
RIVETING --> RIVETED : 30 segundos
RIVETED --> IN_GRIPPER : cobot recoge
IN_GRIPPER --> IN_INSPECTION : coloca en visión
IN_INSPECTION --> INSPECTED_PASS : cámara PASS
IN_INSPECTION --> INSPECTED_FAIL : cámara FAIL
INSPECTED_PASS --> ACCEPTED_BIN : cobot aceptado
INSPECTED_FAIL --> REJECTED_BIN : cobot rechazado
ACCEPTED_BIN --> DONE
REJECTED_BIN --> DONE
DISPENSED --> MANUAL_REMOVAL_REQUIRED : RESTART
ON_CONVEYOR_WAITING --> MANUAL_REMOVAL_REQUIRED : RESTART
AT_SENSOR --> MANUAL_REMOVAL_REQUIRED : RESTART
IN_OUTSIDE_FIXTURE --> MANUAL_REMOVAL_REQUIRED : RESTART
IN_RIVET_FIXTURE --> MANUAL_REMOVAL_REQUIRED : RESTART
IN_INSPECTION --> MANUAL_REMOVAL_REQUIRED : RESTART
MANUAL_REMOVAL_REQUIRED --> [*] : retiro manual
DONE --> [*]`,
  },
  {
    label: "Estados planta",
    code: `stateDiagram-v2
[*] --> IDLE
IDLE --> RUNNING : START
RUNNING --> PAUSED : STOP
PAUSED --> RUNNING : RESUME si seguro
PAUSED --> RESTARTING : RESTART
RESTARTING --> CLEANING_REQUIRED : recuperación terminada
CLEANING_REQUIRED --> IDLE : operador confirma
RUNNING --> FAULT : error crítico
PAUSED --> FAULT : error crítico
RESTARTING --> FAULT : error crítico
FAULT --> CLEANING_REQUIRED : reset requiere limpieza
FAULT --> IDLE : sin piezas y condición segura`,
  },
  {
    label: "Secuencia HMI",
    code: `sequenceDiagram
participant OP as Operador/HMI
participant CONV as Conveyor
participant SENS as Sensor
participant COBOT as Cobot
participant FIX as Fixture externo
participant TABLE as Mesa
participant RIVET as Remachado
participant VISION as Cámara
participant BIN as Bins
OP->>CONV: START
CONV->>CONV: habilitar
OP->>CONV: Colocar CAFI
CONV->>SENS: mover CAFI
SENS-->>CONV: CAFI detectado
CONV->>CONV: detener
SENS-->>COBOT: CAFI listo
FIX-->>COBOT: fixture libre
COBOT->>SENS: pick CAFI
SENS-->>CONV: sensor libre
CONV->>CONV: esperar 2 seg
COBOT->>FIX: colocar CAFI
FIX-->>TABLE: presencia confirmada
TABLE->>TABLE: girar 180°
TABLE-->>RIVET: CAFI en remachado
RIVET->>RIVET: remachar 30 seg
RIVET-->>TABLE: terminado
TABLE->>TABLE: posicionar para retiro
COBOT->>TABLE: recoger remachado
COBOT->>VISION: colocar en inspección
VISION->>VISION: inspeccionar
VISION-->>COBOT: PASS o FAIL
COBOT->>BIN: colocar en bin
BIN-->>OP: ciclo terminado`,
  },
];

function MermaidDiagram({ code, id }) {
  const ref = useRef(null);

  useEffect(() => {
    let cancelled = false;
    async function render() {
      try {
        const mermaid = (await import("https://esm.sh/mermaid@11/dist/mermaid.esm.min.mjs")).default;
        const dark = window.matchMedia("(prefers-color-scheme: dark)").matches;
        mermaid.initialize({
          startOnLoad: false,
          theme: "base",
          fontFamily: "system-ui, sans-serif",
          themeVariables: {
            darkMode: dark,
            fontSize: "13px",
            lineColor: dark ? "#9c9a92" : "#73726c",
            textColor: dark ? "#c2c0b6" : "#3d3d3a",
            primaryColor: dark ? "#3C3489" : "#EEEDFE",
            primaryTextColor: dark ? "#CECBF6" : "#3C3489",
            primaryBorderColor: dark ? "#534AB7" : "#AFA9EC",
            secondaryColor: dark ? "#085041" : "#E1F5EE",
            tertiaryColor: dark ? "#444441" : "#F1EFE8",
          },
        });
        const { svg } = await mermaid.render(id, code);
        if (!cancelled && ref.current) {
          ref.current.innerHTML = svg;
        }
      } catch (e) {
        if (!cancelled && ref.current) {
          ref.current.innerHTML = `<p style="color:#888;font-size:13px;padding:8px">Error al renderizar: ${e.message}</p>`;
        }
      }
    }
    render();
    return () => { cancelled = true; };
  }, [code, id]);

  return (
    <div
      ref={ref}
      style={{ width: "100%", overflowX: "auto" }}
    />
  );
}

export default function SchneiderPlantDiagrams() {
  const [active, setActive] = useState(0);

  return (
    <div style={{
      fontFamily: "system-ui, sans-serif",
      background: "var(--bg, #fff)",
      borderRadius: 12,
      overflow: "hidden",
      border: "1px solid rgba(0,0,0,0.08)",
      maxWidth: 960,
      margin: "0 auto",
    }}>

      {/* Header */}
      <div style={{
        padding: "16px 20px 0",
        borderBottom: "1px solid rgba(0,0,0,0.08)",
      }}>
        <p style={{
          margin: "0 0 12px",
          fontSize: 11,
          fontWeight: 600,
          letterSpacing: "0.08em",
          textTransform: "uppercase",
          color: "#888",
        }}>
          Planta Schneider
        </p>
        <div style={{ display: "flex", gap: 4, flexWrap: "wrap" }}>
          {DIAGRAMS.map((d, i) => (
            <button
              key={i}
              onClick={() => setActive(i)}
              style={{
                padding: "6px 14px",
                borderRadius: 8,
                border: "none",
                cursor: "pointer",
                fontSize: 13,
                fontFamily: "inherit",
                fontWeight: active === i ? 500 : 400,
                background: active === i ? "#1a1a1a" : "transparent",
                color: active === i ? "#fff" : "#555",
                transition: "background 0.15s, color 0.15s",
                marginBottom: 8,
              }}
            >
              {d.label}
            </button>
          ))}
        </div>
      </div>

      {/* Diagram panel */}
      <div style={{ padding: 20, minHeight: 300 }}>
        {DIAGRAMS.map((d, i) => (
          <div key={i} style={{ display: active === i ? "block" : "none" }}>
            <MermaidDiagram code={d.code} id={`mermaid-diagram-${i}`} />
          </div>
        ))}
      </div>
    </div>
  );
}
