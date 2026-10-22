# Plan: integración del chat con eve

Estado (2026-10-22): **fase 1 implementada y testeada con fixtures; NO verificada contra eve en vivo**
(el MCP del GCS estaba caído, así que no hubo una aprobación real de punta a punta). Ver §12.
Rama `feat/eve-agent-integration`. Fecha del plan: 2026-10-21.

Complementa `docs/ChatProtocol.md` (contrato de eventos, aún "proposed") y la sección
"eve Agent Bridge" de `server/CLAUDE.md`, que está desactualizada (ver §9).

---

## 1. Por qué

El bridge actual (`server/models/chat/eveOrchestrator.js`, `eveClient.js`) trata el stream
de eve como request/response: lo drena hasta `session.waiting` y devuelve el último
`message.completed` como texto. Eve es un log de eventos durable, no un request/response.
Consecuencias verificadas contra capturas reales (eve 0.81.1):

| Problema | Evidencia |
|---|---|
| **Se cuelga con una aprobación pendiente.** El stream termina en `input.requested` → `turn.waiting {on:"input"}` y NO emite `session.waiting`. El `for await` no termina y `emitChatBusy(false)` nunca corre: el chat queda bloqueado. | `turn1.ndjson` (cola) y `turn2.ndjson` |
| Sin visibilidad de tool calls ni de tasks. | 7 `actions.requested`, 1 `task.started` descartados por el bridge |
| `message.completed` interino: 7 de 9 en un turno llevan `finishReason:"tool-calls"`, y hay **dos** con `"stop"` seguidos. `finishReason` NO identifica la respuesta final; sí lo hace "el último antes de `turn.completed`". | `turn1.ndjson` |
| Sin streaming de texto (hay 387 `message.appended` por turno). | `turn1.ndjson` |
| Nada de aprobaciones: las herramientas de vuelo (`start_mission`, `load_mission_to_uav`) quedan pausadas en eve y el GCS no puede responder. | `connections/multiuav-gcs.ts:31` en `gcs_eevee_assistant` |

## 2. Decisiones tomadas

1. **El servidor es intermediario, pero transporta; no colapsa.** Abre el stream de eve,
   persiste cursor y estado, audita lo que cruza una frontera de seguridad (aprobaciones),
   y reenvía/proyecta eventos. No reimplementa el loop de herramientas ni los subagentes.
   Alternativas descartadas: navegador directo a eve (pierde la DB unificada y la auditoría de
   vuelo, y no está resuelta la auth fuera de Vercel); parchar el drain (cada capacidad de
   eve sería una traducción nueva a mano).
2. **Una sola tarjeta de aprobación, generalizada** (`ToolApprovalGate.jsx`), no una nueva
   para eve. Dos tarjetas = dos lugares donde arreglar un bug de seguridad.
3. **Eve es la fuente de verdad de la aprobación** en chats eve; nosotros guardamos una copia
   de auditoría. No reimplementamos su semántica (una respuesta stale nunca autoriza,
   un solo ganador).
4. **El legacy no se reescribe sobre eve.** Converge después con un adaptador que emita los
   mismos eventos (§8, fase 3). Razón: el legacy soporta modelos locales (Ollama, llama.cpp);
   no está verificado que eve lo haga.

## 3. Contrato de eve (verificado)

Fuentes: docs y tipos de `node_modules/eve` (0.81.1), más capturas reales en
`/tmp/claude-1000/-home-grvc-work-px4-gcs-eevee-assistant/f34aad0e-520b-4aba-9373-1ec6b6c467a3/scratchpad/`
(`turn1.ndjson`, `turn2.ndjson`, `turn2b.ndjson`; **copiar a `server/test/fixtures/eve/`**,
`/tmp` no es duradero).

**Rutas.** `dev:assistant` → `/eve/v1/*` (:2000). `dev:all` → `/eve/assistant/v1/*` (:3300).
Stream: `GET .../session/:id/stream?startIndex=N` (NDJSON, versión 26 en `x-eve-stream-version`).

**Envelope.** `{type, data, meta:{id:"evt_<ULID>", at}}`. Los eventos de turno traen
`turnId`, `stepIndex`, `sequence`. Un retry re-emite con ids nuevos y los mismos tres campos:
**deduplicar por `meta.id`**.

**Eventos que importan:**

| Evento | Uso |
|---|---|
| `message.appended` | delta de texto (fase 2) |
| `message.completed` | texto autoritativo; hay uno por step |
| `actions.requested` / `action.result` | tool calls (`toolName`, `callId`, `input`) |
| `action.input.appended` | **descartar**: 240 por turno, ruido |
| `task.started` / `task.settled` | tasks. El planner es `kind:"tool"`, `name:"request_mission_plan"`; **no hay `agent.started`** en ese caso |
| `input.requested` | aprobación / pregunta / límite de sesión |
| `input.resolved`, `approval.settled` | resolución |
| `turn.waiting {on:"input"\|"tasks"}` | `input` = esperando humano; `tasks` = esperando tasks |
| `turn.completed\|failed\|cancelled` | **fin de turno** |
| `session.waiting` | después del fin de turno; NO llega si hay aprobación pendiente |

**`input.requested.data.requests[]`** (real): `{kind, requestId, prompt, display, allowFreeform,
options:[{id:"approve"},{id:"cancel"}], action:{kind:"tool-call", callId, toolName, input}}`.
No hay hash ni expiración (el límite es `limits.sessionTimeoutMs`, 30 días).

**Responder:** `POST .../session/:id` con `{"inputResponses":[{requestId, optionId, text?}]}`
(estricto: `message` o `inputResponses`, nunca ambos). `cancel` + `text` → el texto llega al
modelo y a `input.resolved`; la herramienta queda `not_run` (`TOOL_EXECUTION_DENIED`).
Texto libre ante una aprobación **la cancela** (`ignored`), no la aprueba.

**Cancelar:** `POST .../session/:id/cancel` (`{turnId?}`), 202 asíncrono, se confirma con
`turn.cancelled`.

**Errores:** `409 session_not_ready` (reintentar con backoff), `409 session_stranded` (sesión
inalcanzable tras upgrade de eve; solo lectura con `follow=false`), `409 session_not_active`.

**Aprobaciones de un hijo** aparecen en el stream del padre (`callId` y `taskId`) y se
responden por la sesión padre. `request.action.callId` conserva el callId del hijo.

### 3.1 Hallazgos de la segunda ronda (verificados en docs y capturas)

Capturas: `p3.ndjson` (padre) y `child.ndjson` (hijo `planner_sandbox`, ~120 eventos), en el
mismo scratchpad que las anteriores (copiar a `server/test/fixtures/eve/`).

- **Stream del hijo.** `agent.started.data` trae `sessionId` y `streamPath`, pero el
  `streamPath` sale **sin el prefijo del agente** (`/eve/v1/session/<id>/stream`) y en
  `dev:all` da 404; con `/eve/assistant/v1/...` da 200. **Regla: tomar solo el `sessionId` y
  armar la URL con la base configurada**, nunca usar `streamPath` tal cual. Causa: eve fija
  `EVE_ROUTE_PREFIX = /eve/v1` (`dist/src/protocol/routes.js`) y arma `streamPath` con eso; el
  prefijo de montaje (`/eve/<agente>`) lo agrega el cliente o el proxy. Es diseño, no un bug.
  En `dev:assistant` coincide; en `dev:all` no. No se probó si otra versión de `vercel dev`
  lo cambia. El hijo es una
  sesión normal con su propio cursor `startIndex` y el mismo host/credenciales. Sin tools ni
  HITL en la corrida capturada. Las aprobaciones del hijo, según la doc, suben al padre.
- **Retención (crítico).** Por defecto la retención depende de la política de trazas OTel.
  Para audiencia `private` o `unknown` fuera de desarrollo pide retención `0`, y a `0`
  *"completed session transcripts become unreadable"*: leer devuelve stream vacío o
  `410 session_expired` (`agent-config.md:489`, `otel.mdx:45`, `channels/eve.mdx:206`). Un GCS
  autenticado cae en `private`, así que **en producción el historial de turnos completados
  desaparecería** salvo que se fije `experimental.workflow.retention: "default"` en `agent.ts`
  (campo experimental). No se sabe cuánto dura la retención "default" (remite a la doc de
  workflow-sdk). Mientras la sesión está activa o en espera los datos siguen. `sessionTimeoutMs`
  (30 días) no borra datos.
- **Lectura.** El stream solo acepta `startIndex`, `follow` e `includeTailIndex`. `?types=` y
  `?exclude=` se ignoran (probado). No hay historial proyectado ni listado ni DELETE de
  sesiones; `reset` retira el id, `clear` borra solo el historial del modelo. Filtrar del
  lado nuestro (descartar `message.appended`, `reasoning.appended`, `action.input.appended`).
  `eve/client` trae un reducer que quizá se pueda importar en el servidor (no probado).
- **Lectores concurrentes:** dos lecturas con `follow=false` en paralelo funcionan sin estado
  por lector. No probado: lectores `follow` simultáneos.
- **Identidad.** `input.resolved` NO trae principal; sí `approval.settled.data.responderPrincipalId`
  (en dev vale `local-dev`). Es el `principalId` que devuelva el `AuthFn`. Para que sea el
  operador y no el servidor del GCS: `AuthFn` propio (p. ej. JWT HMAC corto por request, con
  `principalId` del operador) o `forwardedPrincipal` con `trustedForwarders`. La policy
  `approval.response` restringe quién aprueba, pero **no se verificó que una connection MCP
  acepte la forma `{request, response}`** (la doc la muestra para `defineTool`).
- **Auth fuera de Vercel.** Mínimo: `eveChannel({ auth: [jwtHmac(...) | httpBasic(...), localDev()] })`
  en `channels/eve.ts` de **cada** agente (assistant y monitor). Hoy el scaffold es
  `[vercelOidc(), localDev(), placeholderAuth()]` y rechaza todo en producción. No hay
  helper de token estático (se escribe un `AuthFn`). Health es público. La auth de ruta no
  impone dueño de sesión: quien pasa puede leer o aprobar cualquier sesión.
- **Herramientas de vuelo.** `FLIGHT_CRITICAL_TOOLS = [load_mission_to_uav, start_mission,
  send_command]` (`connections/multiuav-gcs.ts`), con `endsWith` por el prefijo
  `multiuav-gcs__`. `send_command` no está en `ALLOWED_TOOLS`, así que hoy es inalcanzable
  (entrada defensiva). Es una `const` local sin export: no hay fuente única.
- **Modelos locales.** Sí: `shared/models.ts` ya tiene provider `llamacpp` (chat completions)
  y la doc de eve documenta Ollama, LM Studio y vLLM vía `@ai-sdk/openai-compatible`. Pero el
  código advierte que el tier `high` (los planners) con un modelo local chico produce planes
  que el validador rechaza en bucle. Soportado el transporte; la calidad no está medida.
- **Reinicio con aprobación pendiente:** la doc dice que la pausa HITL es durable y se reanuda
  sin re-emitir. Un step interrumpido se reejecuta (hasta 4 intentos) con ids nuevos y mismos
  `turnId/stepIndex/sequence`. Una sesión creada con otra versión queda `session_stranded` tras
  un upgrade: fijar la versión.

## 4. Lo que NO está verificado

- Reinicio real de eve con una aprobación pendiente y con un `message` en vuelo (solo doc).
- Duración de la retención `"default"` del World local y de producción.
- Que una connection MCP acepte `approval: {request, response}`.
- Que `clientContext` aparezca en el stream.
- Lectores `follow` concurrentes; importar `eve/client` en Node.
- Que `EVE_ENABLE=true` esté en `server/.env` (no se pudo leer).
- Qué tan lejos está el legacy de emitir eventos: no se leyó `handlers/`.
- Forma de los items persistidos de `subagent_result` (necesaria para proyectar `task.*`).
- Si borrar un chat borra sus `ChatToolApproval`.
- Calidad de los planners con un modelo local.

## 5. Arquitectura objetivo

```
UI (ChatPanel / ToolApprovalGate)
   ▲  WS: assistant message, busy, tool_approval_requested/resolved (+ eventos nuevos)
   │
EveRelay  (server/models/chat/eveRelay.js)  ── un lector por chat, en background
   │  reduce eventos → estado de turno, persiste cursor, proyecta a items de chat
   ▼
eveClient.js ── HTTP ──► eve  (/eve/v1/session…)
```

**El lector corre en background**, no dentro de `processMessage`. Con aprobaciones un turno
puede durar horas; una request HTTP/WS no puede esperar eso. `processMessage` solo envía el
mensaje y asegura que haya un lector activo (registro por `chatId`, como `subAgentRegistry`).

**Máquina de estados de un turno** (función pura, testeable con los NDJSON):

```
running ──input.requested──► awaiting_input ──input.resolved──► running
running ──turn.completed──► completed        running ──turn.failed──► failed
running|awaiting_input ──turn.cancelled──► cancelled
```

`turn.waiting {on:"tasks"}` NO es fin ni espera humana: sigue `running` (mostrar "planificando…").

**Cursor.** `chat.metadata.eveStreamIndex` se persiste a medida que se procesan eventos
(no solo al final del turno, como hoy), para que un reinicio del GCS no relea ni pierda.
Dedupe por `meta.id` como segunda red.

**Reanudación tras reinicio del GCS.** Al arrancar, los chats eve con turno abierto
(`metadata.eveTurnOpen`) reabren el stream desde su cursor. Una aprobación pendiente se
reconstruye del stream (último `input.requested` sin su `input.resolved`).

## 6. Proyección a la UI

Para que el historial siga funcionando por REST y `ChatMessages.jsx` renderice sin cambios,
el servidor proyecta a los **items de chat existentes**:

| Evento eve | Item persistido / emitido |
|---|---|
| `message.completed` (último del turno) | `assistant` / `text` |
| `message.completed` interino | no se persiste como respuesta (opcional: reasoning) |
| `actions.requested` | `function_call` |
| `action.result` | `function_call_output` (con `status:'error'` si `isError`) |
| `task.started` / `task.settled` | `subagent_result` o estado transitorio (verificar forma, §4) |
| `input.requested` | `CHAT_TOOL_APPROVAL_REQUESTED` con `InputRequest` generalizado |
| `input.resolved` | `CHAT_TOOL_APPROVAL_RESOLVED` |
| `turn.*` | `CHAT_BUSY` |

**`InputRequest` generalizado** (extiende `toInputRequest` de `toolApprovalStore.js`):

```js
{ requestId, kind: 'tool-approval' | 'session-limit' | 'question', prompt?, chatId,
  options: [{ id, label, style? }],           // NUEVO: leído del evento; legacy = approve/deny
  action: { kind:'tool-call', callId, toolName, input, inputHash? },  // inputHash solo legacy
  expiresAt?,                                 // solo legacy
  source: 'legacy' | 'eve' }
```

`ToolApprovalGate.jsx` hoy asume `expiresAt` (sin él muestra `NaN:NaN`) y hardcodea
`approve`/`deny` (eve usa `cancel`). Cambios: opciones desde `request.options`, cuenta
regresiva e `inputHash` solo si existen, y `kind:'session-limit'` (Continue/Stop).
**Mantener la regla del cliente:** nunca texto libre; solo `{requestId, optionId}`
(`common/util/toolApproval.js`).

## 7. Riesgos y reglas

1. **No reutilizar filas de `ChatToolApproval` para eve sin proteger el sweeper.**
   `approvalSweeper` expira y *reanuda turnos legacy*; si ve filas de eve las expiraría o
   intentaría reanudar un turno que no es suyo. Decisión: tabla de auditoría
   append-only aparte, **para los dos motores** (ver §7.1.1). Sin lógica de estado.

### 7.1.1 `ChatInputAudit` (legacy + eve)

`ChatToolApproval` sigue siendo el estado del legacy (guarda `resumeContext`, `iteration`,
`status`, que se sobrescribe). `ChatInputAudit` es solo escritura, una fila por evento, sin FK
a `Chat` (debe sobrevivir al borrado del chat).

Columnas: `engine` (`legacy|eve`), `chatId`, `requestId`, `callId`, `toolName`,
`kind` (`requested|decided|executed`), `source` (`operator|system|eve`),
`outcome` (`approved|denied|expired|cancelled|ignored|invalid`), `input` (el que vio el
operador, en `requested`), `responderPrincipalId`, `text`, `at`, `inputHash?` (solo legacy),
`eveEventId?` (`meta.id`). Índice único `(engine, requestId, kind, source)` con insert-ignore
para que el relay no duplique filas al releer eventos.

| Punto de escritura | kind / source |
|---|---|
| legacy `chat.js:370` `createPending` | `requested` |
| legacy `chat.js:640` `resolve` | `decided` / `operator` |
| legacy `chat.js:595` `expireStale`, `cancelPending` | `decided` / `system` |
| legacy `chat.js:571` `claimExecution` | `executed` |
| eve `input.requested` | `requested` |
| eve POST de `inputResponses` (aquí se conoce al operador) | `decided` / `operator` |
| eve `input.resolved` + `approval.settled` | `decided` / `eve` (puede diferir: `ignored`, `invalid`); el principal sale de `approval.settled.data.responderPrincipalId` |
| eve `action.result` | `executed` (`completed` o `not_run`) |

Notas: `expired` solo existe en el legacy e `ignored` solo en eve; se normalizan al guardar.
`responderPrincipalId` queda vacío o como pista mientras el WS no tenga auth (igual en los
dos motores). Sin transacciones hoy en el legacy: ver política de fallo en §10.2.
2. **El enforcement cambia de dueño.** En eve la herramienta se ejecuta dentro de eve; el GCS
   solo transporta la decisión y no puede re-verificar un hash. Mitigación: la auditoría
   guarda el `action.input` exacto que se mostró al operador.
3. **Dos listas de herramientas de vuelo** (`TOOL_APPROVAL_REQUIRED` en el GCS, por defecto
   2; `FLIGHT_CRITICAL_TOOLS` en eve, 3 con `send_command`). Hoy no hay fuente única ni export.
   Test de contrato con la relación correcta: **toda herramienta de `TOOL_APPROVAL_REQUIRED`
   debe estar en la lista de eve** (que tenga de más es aceptable, es defensivo); falla si no.
   Requiere que el test lea el archivo del repo eve o que se extraiga la lista a un módulo
   compartido.
4. **Quién aprueba.** Eve por defecto solo deja aprobar a quien inició el turno; con varios
   operadores eso se decide en el servidor. Hoy el WS no tiene auth (`chat.js`:
   `responderPrincipalId` es solo una pista de auditoría). No cambia con esta integración,
   pero el chat eve hereda el hueco: documentarlo.
5. **Auth de eve en producción.** El scaffold actual rechaza todo fuera de dev. Hay que
   configurar `eveChannel({auth})` en cada agente (§3.1). Es auth de **servicio** (el GCS se
   identifica ante eve, p. ej. un `AuthFn` con bearer desde env), no de operador: no hay
   usuarios. El responder de eve será siempre el servidor del GCS.
6. **Versión.** Fijar la misma versión de eve en cliente y servidor del repo eve.
7. **Un solo lector por chat.** Registro con guardia; dos lectores duplicarían mensajes.
8. **`turn.waiting {on:"tasks"}` ≠ fallo.** Un `task.settled {status:"failed"}` puede ir
   seguido de un turno que termina bien.
9. **Retención 0 por defecto en producción** (§3.1): sin `experimental.workflow.retention:
   "default"`, los turnos completados se vuelven ilegibles (`410 session_expired`). El relay
   debe manejar ese `410` como "historial ya no disponible", no como error de sesión, y por
   eso **nuestra copia proyectada no puede tratarse como desechable** (ver §10.2b).
10. **El `streamPath` del hijo no sirve tal cual.** Construir la URL con la base configurada y
    el `sessionId`.

## 8. Fases

### Fase 1: mínimo seguro (primer commit lógico)

Objetivo: que un chat eve **nunca se cuelgue**, muestre tool calls y permita aprobar/rechazar.

Servidor:
- `config.js`: de una sola `EVE_URL` a un **registro de agentes** (eve no tiene endpoint que
  los liste; `/eve`, `/eve/agents` y `/.well-known/eve` dan 404). Una entrada por agente con
  nombre y base URL: `assistant` → `http://localhost:3300/eve/assistant` (`dev:all`) o
  `http://localhost:2000/eve` (`dev:assistant`, sin nombre de agente en el path). El cliente
  agrega `/v1/...`. Al arrancar, `GET <base>/v1/health` por agente (vivo o no, público) y
  `GET <base>/v1/info` (tools, subagentes, conexiones; pasa por la auth del canal: en dev lo
  deja `localDev()`, en producción necesita la auth real). Cada chat eve guarda en
  `metadata` el **nombre del agente** además del `sessionId`. En `dev:all` responden
  `assistant`, `monitor` y `planner_sandbox`. Producción propia: `eve build` + `eve start`
  por agente desde su directorio, un servidor por agente. Cambio de config: documentarlo.
- `eveClient.js`: `sendInputResponses`, `cancelSession`, manejo de `409` (backoff en
  `session_not_ready`) y de `410 session_expired`, `getHealth`, y un `streamUrl(sessionId)`
  que arme la URL desde la base configurada (también para sesiones hijas).
- Producción (antes de desplegar, no bloquea dev): `channels/eve.ts` con auth propia y
  `retention: "default"` en el repo eve (§3.1, §7.9).
- `eveRelay.js` (nuevo): máquina de estados pura + lector en background + registro por chat +
  cursor persistente + dedupe por `meta.id` + proyección de §6.
- `eveOrchestrator.js`: `processMessage` solo envía y arranca el lector; nueva
  `respondToInput(chatId, responses)` para aprobaciones; `cancel(chatId)`.
- `chat.js` (controller): enrutar la respuesta de aprobación a eve cuando
  `_resolveEngine` = 'eve' (hoy va a `ToolApprovalStore`).
- Auditoría de aprobaciones (§7.1).
- Rehidratar lectores y pendientes al arrancar.

Cliente:
- `ToolApprovalGate.jsx` generalizada (§6).
- Estado mínimo transitorio por chat: `running | awaiting_input | planning`, para el chip
  y para no bloquear la entrada. Sin reducer completo de eventos.

Tests (`node:test`, con los NDJSON como fixtures):
- Máquina de estados: turno normal, turno con aprobación (aprobada y rechazada), task
  fallida, retry (mismo `turnId/stepIndex/sequence`, distinto `meta.id`).
- Regresión del bug del cursor (follow-up no relee el turno anterior).
- Regresión del cuelgue (stream que termina en `turn.waiting {on:"input"}` libera el lector
  y deja `awaiting_input`, no `busy`).
- Contrato de herramientas de vuelo (§7.3).
- Cliente: la tarjeta con `options` sin `expiresAt` ni `inputHash`.

Docs: corregir `server/CLAUDE.md` (§9).

### Fase 2: experiencia

- Deltas de texto (`message.appended`) con coalescencia en el servidor (387 por turno;
  no reenviar tal cual).
- Progreso de tasks/planner en la UI.
- Stream hijo (`agent.started.streamPath`): **verificar primero** con una captura real que
  fuerce un subagente.
- Cancelación desde la UI (`cancel`).
- Prompt de `session-limit` (Continue/Stop) con costo/uso acumulado (`turn.waiting.usage`).

### Fase 3: convergencia con el legacy

- Reducer de eventos único en el cliente (cumple `docs/ChatProtocol.md`).
- Adaptador que hace que el orquestador legacy emita los mismos eventos, sin reescribir su
  loop. Requiere decidir `ChatProtocol.md` (cambia el esquema de persistencia).
- Antes: leer `handlers/` para medir el costo.

## 9. Documentación a corregir

`server/CLAUDE.md` › "eve Agent Bridge":
- `subagent.called` / `childSessionId` **no existen** (eliminados, CHANGELOG de eve línea 495).
- El repo eve está en `../gcs_eevee_assistant` (relativo a `llm_planner_gcs/`), no `../../`.
- "No client UI wired" es falso desde `acac46b` (hay botón de chat eve y chip `eve · beta`).
- La versión de eve es 0.81.1, no 0.53 ni 0.69.
- `EVE_URL` con prefijo (fase 1).

## 10. Decisiones abiertas

1. **Fase 1: ¿proyectar a items existentes o emitir eventos al cliente?** El plan proyecta a los
   items existentes porque el historial por REST los necesita igual y `ChatMessages` ya los
   renderiza. Contra: se retrasa el reducer único (fase 3) y se pierde información (deltas,
   `turnId`). Alternativa: persistir eventos crudos y proyectar en el cliente: más fiel a
   `ChatProtocol.md`, pero obliga a decidir ese documento antes.
2. **Auditoría:** DECIDIDO tabla mínima `ChatInputAudit` para legacy y eve (§7.1.1);
   `metadata` descartada. DECIDIDO política de fallo: **fail-closed solo para `decided` en el
   camino eve** (se escribe antes del `POST` a eve; si falla, no se envía y la aprobación
   queda pendiente con un error visible para reintentar). Todo lo demás es best-effort
   (se loguea): `requested`, `executed` (ya corrió, no se puede deshacer) y todo el legacy,
   donde `ChatToolApproval` ya es la compuerta (`resolve` atómico + `claimExecution`), así
   que no cambia su comportamiento. Sin verificar: si borrar un chat borra sus
   `ChatToolApproval`.
2b. **Historial de eve:** DECIDIDO proyectar a items existentes. **Corrección:** la idea de
   "caché desechable" depende de la retención de eve. Con la retención por defecto de
   producción (0 para audiencia `private`) los turnos completados son ilegibles, y entonces
   nuestra copia proyectada es la **única** copia. Dos caminos, no excluyentes: (a) fijar
   `experimental.workflow.retention: "default"` en `agent.ts` (experimental, duración
   desconocida) y tratar la copia como caché; (b) tratar nuestra copia como registro
   durable de los turnos completados, proyectándola al terminar cada turno. Recomendado: (b)
   siempre, (a) además. Sin filtros en el stream, el costo de releer es alto (~228 KB por
   turno), otra razón para (b).
7. **Qué agentes expone la UI.** Hoy el botón crea siempre un chat con el asistente. Con un
   registro de agentes, ¿se ofrece solo `assistant`, o también `monitor`? No se sabe qué hace
   `monitor` ni si es apto para chat de operador. `planner_sandbox` es un subagente de
   depuración, no debería ofrecerse a operadores.
6. **Identidad del operador:** DECIDIDO que no se implementa. El GCS no tiene usuarios ni los
   tendrá, así que `responderPrincipalId` queda nulo en los dos motores y no se hace JWT por
   operador. La auditoría registra qué se mostró, qué se decidió y cuándo, no quién. Sigue
   haciendo falta la auth de servicio GCS→eve en producción (§7.5), que es otra cosa.
3. **Quién puede aprobar** con varios operadores, mientras no haya auth en el WS.
4. **Modelos locales:** eve los soporta como transporte (`llamacpp`, openai-compatible). Lo
   abierto es la calidad de los planners con un modelo chico, que no se midió. Mientras no se
   mida, el legacy no se retira.
5. **Reinicio de `eve dev`:** la doc dice que la pausa HITL sobrevive; no se probó. Se
   verifica creando una sesión con aprobación pendiente, reiniciando `dev:all` y leyendo con
   `startIndex=-1`.

## 11. Cómo verificar lo pendiente

```bash
# levantar eve (en gcs_eevee_assistant, Node 24)
source "$NVM_DIR/nvm.sh" && nvm use && npm run dev:assistant
curl -s localhost:2000/eve/v1/health        # workflowId
curl -s localhost:2000/eve/v1/info          # config efectiva y auth
curl -N "localhost:2000/eve/v1/session/<id>/stream?startIndex=0"
# GCS
cd server && npm test                       # nunca `node --test` pelado (usa la DB real)
```

## 12. Estado de la fase 1 (2026-10-22)

**Hecho** (todo en `server/` salvo el cliente):

| Pieza | Dónde |
|---|---|
| Registro de agentes (`EVE_AGENTS`, `EVE_URL` compatible, `EVE_DEFAULT_AGENT`) | `config/config.js` |
| Cliente HTTP (reintento de `session_not_ready`, `410`, `inputResponses`, `cancel`, URL propia) | `models/chat/eveClient.js` |
| Reducer puro de eventos | `models/chat/eveEvents.js` |
| Lector en background con cursor, reconexión y dedupe | `models/chat/eveRelay.js` |
| Orquestador (no espera), `respondToInput`, `cancel`, `resumeOpenTurns` | `models/chat/eveOrchestrator.js` |
| Auditoría para los dos motores | `ChatInputAudit` + `models/chat/inputAudit.js`; hooks en `chat.js` |
| Eventos nuevos: `CHAT_ACTIVITY`, `CHAT_TOOL_APPROVAL_FAILED` | `eventBus.js`, `chatEvents.js`, `websocketSubscriber.js` |
| Tarjeta única generalizada, actividad, fallos de aprobación | `client/` (`ToolApprovalGate.jsx`, `inputRequest.js`, `store/chat.js`) |
| Tests | `test-eve-{events,relay,client,orchestrator,flight-tools-contract}.js`, `test-chat-input-audit.js`, `test-input-request-client.js` |

**Diferencias respecto a lo planeado:**

- **Fallo de aprobación visible.** No estaba en el plan: con la tarjeta quitada al hacer clic, un fail-closed
  dejaba al operador sin tarjeta y sin saber que no se envió nada. La tarjeta de eve ahora espera el
  `input.resolved`, y `CHAT_TOOL_APPROVAL_FAILED` la mantiene con el error.
- **El cliente no consumía `chatBusy`** (y `sendingMessage` se apaga en el mismo tick que se prende), así que
  el indicador de actividad es propio (`state.chat.activity`), no un arreglo del existente.
- **`ENUM` de Sequelize no valida en SQLite.** El modelo de auditoría agrega `validate.isIn` explícito.
- **`HelpOutline` no existe en `@mui/icons-material` v9** (lo detectó el linter): se usa `HelpOutlineOutlined`.
- La política de fallo (§10.2) quedó: fail-closed SOLO en `decided` de eve.

**Pendiente / sin verificar:**

1. **Prueba en vivo con aprobación real.** Los fixtures `*.synthetic.ndjson` están armados a mano con las
   formas que reportó eve. Reemplazar por capturas reales (`gcs_eevee_assistant/test/fixtures/gcs-bridge/`)
   cuando el MCP esté arriba. En particular se asume `action.result.data.status === "rejected"` para un rechazo
   (el tipo y el CHANGELOG lo dicen; no se vio en una captura viva).
2. **Texto libre durante una aprobación** (`ignored`): fixture sintético, sin captura.
3. **Dos fallos de la suite ajenos a este cambio:** `test-mcp-client.js` (el MCP en `:3001` está caído) y
   `test-definition-resolver.js` › windTurbine (lee datos de `models/markers`, que no se tocaron).
4. Retención de producción (§7.9) y auth de servicio (§7.5): configuración del repo eve, no hecha.
5. Fase 2: deltas de texto, stream hijo, botón de cancelar (existe `EveOrchestrator.cancel`, falta ruta y UI),
   actividad tras recargar la página, preguntas con texto libre.
6. `server/.env.example` no se tocó (el permiso lo impide): documentar `EVE_AGENTS` / `EVE_DEFAULT_AGENT` ahí.
