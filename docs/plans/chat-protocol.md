# ADR — Contrato de mensajes del chat LLM (server ↔ cliente)

**Estado:** Propuesto — pendiente de aprobación antes de implementar
**Fecha:** 2026-09-18
**Alcance:** `server/models/chat/`, `server/subscribers/websocketSubscriber.js`,
`client/src/store/chat.js`, `client/src/components/chat/`

---

## 1. Contexto

El chat LLM de este GCS no es decorativo: el modelo invoca herramientas que cargan y
arrancan misiones de vuelo reales (`load_mission_to_uav`, `start_mission`). El contrato
de mensajes entre server y cliente es, por lo tanto, parte del camino crítico de
seguridad del sistema, no solo de la UI.

Hoy ese contrato no está definido en ningún lado. Lo que existe es el resultado
acumulado de decisiones puntuales, y tiene fallas estructurales verificadas en código.

### 1.1 Estado actual (verificado, no supuesto)

**No hay identidad estable de mensaje.**
La tabla `ChatMessage` tiene PK autoincremental real
(`server/schemas/database/chatMessage.model.js:20-26`), pero ese `id` nunca sale del
backend: ni `ChatHistoryManager.loadHistory` (`server/models/chat/chatHistoryManager.js:317-322`,
que proyecta solo `chatId`/`from`/`timestamp`/`message`) ni el emit de WebSocket
(`server/subscribers/websocketSubscriber.js:36-43`) lo incluyen. El cliente fabrica un
sustituto en `client/src/store/chat.js:87`:

```js
id: message.id || `msg_${Date.now()}_${Math.random()}`,
```

Como `message.id` nunca llega, ese fallback es siempre el id real. Consecuencia: el mismo
mensaje lógico tiene una clave en vivo y otra distinta después de recargar.

**Hay tres relojes para el mismo mensaje.**

| Momento | Dónde | Destino |
| --- | --- | --- |
| Envío del usuario | `client/src/services/sendChatMessage.js:15` | Descartado: `chat.js:319-322` (`_applyInput`) nunca lo usa |
| Persistencia | `server/models/chat/chatHistoryManager.js:139` | Queda en la DB |
| Emit por WS | `server/subscribers/websocketSubscriber.js:41` | Es el que ve el cliente |

El timestamp que ve el cliente en vivo **no** es el que quedó en la DB. Y
`prependMessages` (`client/src/store/chat.js:120-131`) deduplica comparando `timestamp`,
porque no tiene un id con el que comparar. Esa deduplicación no puede funcionar de forma
confiable.

**Las imágenes viajan y se almacenan como base64.**
El store guarda el objeto `message` completo sin truncar, y el data URI se reconstruye por
concatenación en `client/src/components/chat/ChatMessages.jsx:790-792`.

**El fallback de tipo desconocido corrompe el contenido.**
Hay dos embudos en cascada: `convertMsg` (`ChatMessages.jsx:175-184`) convierte cualquier
tipo no contemplado en `{type:'text', content: JSON.stringify(...)}` sin pretty-print, y el
`default:` del switch de render (`ChatMessages.jsx:1012`) lo manda a `TextMessageBlock`.
Ahí, en la línea 942, el contenido pasa por `ReactMarkdown` con `remarkGfm`: un payload con
`_`, `*`, `#`, backticks o pipes se reinterpreta como formato.

El resultado es un artefacto del cliente **visualmente indistinguible de un mensaje real del
asistente, y además deformado**. Para un operador que decide si mandar un UAV a volar, eso
no es un bug cosmético.

> No hay vulnerabilidad de inyección: no se usa `rehype-raw`, así que `react-markdown` no
> renderiza HTML crudo. El problema es de corrupción de formato y de confusión de origen.

**La falta de id forzó una estrategia de caché.**
El comentario en `ChatMessages.jsx:265-285` lo documenta solo:

> *"Keying on a message id would not work here — the server sends no stable id, and the
> client-side fallback differs between a live message and the same message after a reload."*

De ahí sale `memoizeByIdentity` (WeakMap sobre la identidad del objeto). El mismo comentario
registra el costo en render: `convertMsg` puede correr tres `JSON.parse` anidados,
`formatMcpContent` encadena cuatro regex sobre strings de ~145 KB, y ambos terminan en un
`JSON.stringify(..., null, 2)` — trabajo que debería hacerse una sola vez, en el server, al
persistir.

**El cliente adivina semántica olfateando el payload.**
`getResultSummary` (`ChatMessages.jsx:240-254`) hace duck-typing sobre la salida de las
herramientas: `content.mission || content.missionId` → `"Created Mission"`. Quien define la
herramienta sabe qué hizo; el cliente no debería deducirlo del shape.

**No existe ningún gate de aprobación humana.**
El loop de herramientas de `server/models/chat/chat.js` ejecuta `load_mission_to_uav` /
`start_mission` directamente. El único "gate" es prosa en los prompts `.md` de los agentes,
no código.

### 1.2 Lo que sí está bien y no se toca

El server ya normaliza el protocolo de cada proveedor a un vocabulario interno común antes
de persistir o emitir (`server/models/chat/handlers/antropicHandler.js:158-185` y sus pares
de OpenAI/Gemini/Ollama). El cliente nunca ve bloques crudos de Anthropic ni de OpenAI. Esa
capa de abstracción funciona y queda intacta.

### 1.3 Diagnóstico de raíz

Los problemas de arriba no son independientes. Son síntomas de **dos pipelines**:
`loadHistory` (REST) y el emit de WS producen shapes distintos por caminos distintos, y
convergen solo por accidente. Arreglar únicamente el id tapa el síntoma más visible y deja
la falla estructural intacta.

### 1.4 Referencias externas consultadas

Se consultaron dos proyectos con arquitecturas comparables, contrastando contra sus tipos
reales:

- **llama.cpp** (`tools/server` + `tools/ui`): server sin estado de conversación, historial
  client-side en IndexedDB. Aporta la separación de ids en capas que nunca se pisan. Su
  `content: string | ContentPart[]` polimórfico (con colapso a string cuando queda una sola
  parte de texto) es **el anti-patrón** que origina nuestro `convertMsg`.
- **eve** (Vercel, 0.53.1): log de eventos append-only durable + proyección por reducer en
  el cliente. Es la referencia principal de este documento. Se le pidió además una revisión
  adversarial del contrato propuesto, cuyos hallazgos están incorporados en §3.

---

## 2. Decisión

Adoptar un **contrato de eventos** con proyección en el cliente, y separar explícitamente
lo que hoy está mezclado: identidad, posición, tiempo, presentación y progreso.

### 2.1 Envelope

Tres campos, invariante para todos los tipos de evento. Nada semántico en la raíz.

```js
{
  type: "message.appended" | "action.requested" | "action.result" | ...,
  data: { ...payload, turnId, stepIndex, callId?, requestId?, sequence },
  meta: { id: "evt_<ULID>", at: "<ISO-8601>", attemptId?: "<ULID>" }
}
```

### 2.2 Los tres identificadores, con un trabajo cada uno

Ningún identificador cumple doble función. Esta es la regla que hoy se viola.

| Campo | Lo genera | Trabajo | Persiste |
| --- | --- | --- | --- |
| `clientRequestId` | Cliente | Correlaciona sus eventos sintéticos `client.*` (render optimista) | No |
| `eventId` (ULID) | Server, al persistir | Identidad: dedupe, clave de React, clave de ingest | Sí |
| `sequence` | Server, contador absoluto por chat | Cursor de paginación y detección de huecos | Sí |

`eventId` es ULID (timestamp + bits aleatorios), estampado **una sola vez, inmediatamente
antes de escribir**. Reconectar o releer devuelve el mismo id, lo que lo habilita como clave
de idempotencia (`on conflict (id) do nothing`).

**`eventId` es identidad, no posición.** Un ULID es ordenable en el tiempo pero no da orden
total entre procesos con relojes distintos. Para ordenar y paginar se usa `sequence`, nunca
el ULID.

`attemptId` cubre los reintentos: dos intentos del mismo paso emiten eventos con el mismo
`turnId`/`stepIndex`/`sequence`, y sin este campo no hay forma de distinguirlos. (eve no lo
tiene y lo señaló como su propio agujero.)

### 2.3 Un solo timestamp

`meta.at` se estampa en el mismo instante que `meta.id`, al hacer append. Se eliminan los
otros dos relojes. Todo lo demás lo lee; nadie lo regenera.

### 2.4 Jerarquía de correlación

```
session
 └─ turnId      unidad de cancelación y de auditoría causal
    └─ stepIndex   identidad del bloque de texto en construcción
       └─ callId      ciclo de vida de una tool call
          └─ requestId   un pedido de intervención humana
sequence   ordena y pagina
```

Se adoptan los cuatro niveles. `stepIndex` merece justificación porque el nombre engaña: no
es principalmente un contador de llamadas al modelo, es **la identidad del bloque que se
está acumulando**. Los deltas no llevan `messageId`; la tupla `(turnId, stepIndex)` es la
identidad. Sin él haría falta un `blockId` explícito — más trabajo y menos general — y se
fusionarían en un solo bloque los casos de texto → tool call → texto dentro del mismo turno.

La correlación es **siempre por id, nunca por posición en un array**.

### 2.5 Proyección al cliente

Siempre array. Nunca polimórfico. Este es el punto que elimina `convertMsg` por completo.

```js
ChatMessage { id, role, parts: Part[] }

Part = { type: "text",      text, state: "streaming" | "done" }
     | { type: "reasoning", text, state }
     | { type: "file",      blobId, mediaType, size, filename? }   // nunca bytes
     | ToolPart
```

`ToolPart` es una máquina de estados discriminada:

```
input-streaming ──► input-available ──┬──► output-available
                                      ├──► output-error
                                      └──► approval-requested ──► approval-responded
                                                                   ├──► output-available
                                                                   └──► output-denied
```

Los campos imposibles en cada estado se tipan como inalcanzables, de modo que sea
estructuralmente imposible renderizar la salida de una herramienta que no corrió.

**El switch de render no lleva cláusula `default`.** La exhaustividad se verifica en
compilación: agregar una variante debe romper el build, no degradar en runtime. Si un tipo
desconocido llega igual, se renderiza como **placeholder explícitamente marcado como no
soportado** — nunca como burbuja de texto normal, y nunca a través del renderer de markdown.

### 2.6 Un reducer, dos fuentes

El render optimista no crea una entidad paralela que después haya que reconciliar. El
cliente inyecta eventos sintéticos **al mismo reducer**:

```
client.message.submitted    ← pinta ya
client.message.failed
client.input.responded      ← feedback inmediato; input.resolved confirma después
```

Local pinta, durable confirma. Un solo pipeline: en vivo y tras recargar corre el mismo
reducer sobre el mismo log.

### 2.7 Presentación separada del dato

Las etiquetas y estilos viajan en su propio sub-objeto, declarados por quien define la
herramienta:

```js
{ actions: [...], presentation: { "<callId>": { label?, style? } } }
```

El cliente no mantiene listas de nombres de herramientas peligrosas ni deduce resúmenes del
shape del payload. Esto elimina `getResultSummary`.

### 2.8 Dos planos: log durable y progreso efímero

| Plano | Contenido | Garantía |
| --- | --- | --- |
| **Log durable** | Hechos: empezó, terminó, resultado, falló, se canceló, se aprobó | Persistido, auditable, append-only |
| **Progreso efímero** | `"iteración 47, gap 3.2%, 28s"` | Best-effort, batcheado, se pierde sin consecuencias |

**Criterio de decisión:** *si este evento se pierde, ¿cambia lo que el operador puede
reconstruir después?* Si no, es progreso.

Esto importa concretamente para el planner MIP (servicio FastAPI externo, decenas de
segundos por resolución). Meter su progreso en el log durable no es un problema de espacio:
es que en una investigación posterior **la señal se ahoga en ruido de progreso**.

### 2.9 Estados terminales distintos

`rejected` ≠ `failed` ≠ `cancelled`.

- **`rejected`**: denegado en el gate de aprobación. Nunca ejecutó. Ni éxito ni fallo.
- **`failed`**: ejecutó y falló.
- **`cancelled`**: evento terminal propio, sin código de error.

Colapsarlos destruye el registro de auditoría: deja de poder distinguirse *"el operador dijo
que no"* de *"el UAV no respondió"*.

Los errores usan `{code, message, details?}` uniforme en todos los niveles, para tener un
solo renderer de error.

### 2.10 Binarios por referencia

El mensaje lleva `{type:"file", blobId, mediaType, size, filename?, sha256?}`. Nunca bytes.
El ciclo de vida se ata al chat, y la autorización se resuelve con **la misma policy que el
propio chat**: si podés leer el chat, podés leer sus blobs. No se construye un sistema de
autorización paralelo para binarios.

> Nota: llama.cpp acepta base64 inline en el wire, pero internamente lo separa de inmediato
> a buffers binarios y deja un marcador en el texto. Ni ellos lo usan como representación
> interna; es una concesión de transporte que existe porque su server no tiene concepto de
> usuario con el que acotar un endpoint de blobs. Nosotros sí lo tenemos.

---

## 3. Requisitos de seguridad para la aprobación humana (HITL)

Esta sección es normativa. Los puntos 3.1 y 3.2 son los que, si no están, hacen que todo lo
demás sea decoración.

### 3.1 La aprobación se liga al input congelado, no a la intención

**El riesgo concreto:**

1. El modelo pide `start_mission({uav:3, missionId:"A", altitude:40})`
2. El turno se parkea. Se muestra al operador: *"¿Arrancar misión A en UAV 3 a 40 m?"*
3. El operador aprueba.
4. Si el loop se reconstruye **re-corriendo el modelo**, el modelo emite
   `start_mission({uav:3, missionId:"A", altitude:120})`
5. Se ejecuta. El operador aprobó 40 m. El UAV vuela a 120 m.

Un LLM con `temperature > 0` no devuelve los mismos argumentos. Con `temperature = 0`
tampoco está garantizado si cambió el proveedor, la versión del modelo, o si el contexto se
compactó en el medio.

**Norma:** el pedido de aprobación lleva el input completo y serializado adentro, y se
ejecuta **ese valor**, nunca uno recalculado.

```js
{
  requestId,
  kind: "tool-approval",
  prompt,
  action: {
    kind: "tool-call",
    callId,
    toolName,
    input,                    // el valor, no un puntero a la intención
    inputHash: "sha256:..."   // del input canonicalizado (claves ordenadas)
  },
  options?, display?, allowFreeform?, expiresAt
}
```

Antes de ejecutar, se recomputa el hash del input a ejecutar y se compara con el aprobado.
**Si no coincide, no se ejecuta** y se emite un error tipado.

### 3.2 El texto libre no aprueba nada

Si el operador escribe *"dale, sí"*, eso inicia un turno normal y la aprobación **queda
pendiente**. Solo una respuesta estructurada con `requestId` resuelve:

```js
{ requestId, optionId?, text? }
```

Con varios pedidos pendientes, no se adivina a cuál se refiere el texto libre. Esto no es
una preferencia de UX.

### 3.3 Las aprobaciones vencen, y el mundo se revalida al ejecutar

El hash protege contra cambios en el input. **No protege contra cambios en el mundo.**

Sin expiración, un operador puede aprobar `start_mission` cuarenta minutos después, con el
UAV ya aterrizado, otra batería, otro viento y otro drone en ese espacio aéreo.

- `expiresAt` en el pedido. Vencido = no aprobable; se proyecta como `expired` y el modelo
  debe volver a pedirlo.
- **Revalidación del estado del mundo en el momento de ejecutar**, no de aprobar: ¿el UAV
  sigue armado y en el estado esperado? ¿la misión sigue cargada? ¿sigue siendo el mismo
  espacio aéreo? Si no, se aborta con error tipado.

> La aprobación humana autoriza una acción **bajo unas condiciones**. Si las condiciones
> cambiaron, la autorización no vale. (eve no tiene expiración en sus tool approvals: es un
> hueco de su diseño para nuestro caso de uso, no un patrón a copiar.)

### 3.4 Identidad de quien aprueba

El evento de resolución lleva `responderPrincipalId`. No es opcional en un GCS
multi-operador:

- Para la auditoría post-incidente, *"alguien aprobó"* no sirve.
- **El server rechaza aprobaciones de principals sin el rol.** Si esa comprobación vive solo
  en la UI (botón deshabilitado), no existe.
- Debe fijarse la política de carrera: primero gana, o doble aprobación para herramientas
  críticas. Ponerlo en el protocolo ahora es barato; agregarlo después es cambiar el
  contrato.

### 3.5 Idempotencia de la ejecución

`requestId` da idempotencia de la **aprobación** (la segunda llega y el pedido ya está
resuelto). No da idempotencia de la **ejecución** si el proceso cae entre "marqué aprobado"
y "ejecuté", ni cubre el doble click ni el reintento del POST por timeout.

Hace falta un registro durable con `callId` como clave: *"`callId` X ya se ejecutó,
resultado Y"*.

### 3.6 El log es el registro de vuelo

- **Append-only real**: sin `UPDATE` ni `DELETE`, aplicado a nivel de base de datos
  (permisos del rol de la aplicación o trigger), no por convención del equipo.
- **Hash-chaining** recomendado: cada evento guarda el hash del anterior. En investigación
  de accidentes, eso es la diferencia entre evidencia y anécdota.
- **Retención explícita y separada** de la del chat: los eventos con consecuencia de vuelo
  probablemente deban durar más.

### 3.7 Qué se persiste para parkear y resumir

No alcanza con el array de mensajes:

1. El array de mensajes normalizados hasta el punto de pausa.
2. **La tool call pendiente congelada**: `callId`, `toolName`, input serializado,
   `inputHash`.
3. **La configuración del modelo de ese turno**: `modelId`, proveedor, `temperature`,
   versión del system prompt, versión del set de herramientas. Si se resume sobre otro
   modelo porque alguien deployó en el medio, el comportamiento cambia en silencio.
4. `turnId` y el `stepIndex` siguiente, para no romper la numeración.
5. El binding `requestId` ↔ `callId`.
6. Las herramientas disponibles en ese momento, si el set es dinámico.
7. El `expiresAt`.

**Al resumir no se vuelve a llamar al modelo.** Se toma el input congelado, se ejecuta la
herramienta, se appendea el resultado y recién ahí sigue el loop normal. El modelo ve la
tool call y su resultado consecutivos; nunca se entera de que hubo una pausa. Por eso esto
es barato de implementar sin durable execution: no se re-ejecuta nada.

### 3.8 Por qué el modelo de eventos no es opcional aquí

"Las aprobaciones pendientes sobreviven al reload" **no es una feature que haya que
escribir**. Cae sola de log durable + reducer determinístico: `input.requested` es un evento
más; al recargar se reproduce el log, el reducer vuelve a proyectar `approval-requested`, y
sigue pendiente porque nunca llegó `input.resolved`.

Sin log de eventos, esa feature se escribe a mano: tabla de aprobaciones pendientes,
sincronización con el estado del mensaje, qué pasa si el server reinició con una aprobación
abierta, reconciliación entre dos tablas. Es decir, **dos mecanismos de persistencia que
pueden desincronizarse, en el camino crítico de armar un drone.**

---

## 4. Transporte: WebSocket

Se mantiene WebSocket. No se migra a HTTP + NDJSON.

**Justificación:** la propiedad de "un solo pipeline" es del **modelo de datos**, no del
transporte. Sale de *hay un log, el cliente tiene un cursor, la proyección es un fold
determinístico*. Nada de eso exige HTTP. WS aporta push del server, bidireccionalidad y una
conexión para N chats.

Lo que WS no da gratis y hay que construir:

### 4.1 Persist-then-emit, nunca al revés

Si se emite por el socket antes de que el evento esté commiteado y el proceso muere, el
cliente tiene un evento que no existe en el log y su cursor avanzó a un `sequence` que el
server nunca va a poder replayar. **Ese cliente queda permanentemente desincronizado.**

Con un stream que *es* el log, ese bug no puede existir. Nosotros tenemos dos caminos (DB y
socket) y hay que ordenarlos a mano: **escribir primero, emitir después, siempre.**

### 4.2 Resume y detección de huecos

- Handshake de reconexión con `resume(chatId, fromSequence)`; el server replaya desde ahí.
- Detección de huecos por contigüidad de `sequence` **por chat**.
- **El cursor avanza cuando se aplica, no cuando se recibe.** Si el cliente mueve
  `lastSequence` al recibir del socket y cae entre recibir y aplicar, perdió ese evento para
  siempre y nunca lo va a pedir.
- El socket multiplexa N chats con contadores independientes: el resume manda los cursores
  de **todos los chats abiertos**, no solo el activo.

### 4.3 `sequence` necesita un único escritor

Con dos procesos Node (o workers), dos eventos concurrentes del mismo chat pueden pedir el
mismo `sequence` — por ejemplo, dos resultados de herramienta llegando juntos. Se resuelve
con secuencia de Postgres por chat, o `INSERT ... RETURNING` con constraint único sobre
`(chatId, sequence)` y reintento, o lock por chat.

Hay que resolverlo **ahora que es un solo proceso**. El día que se escale es un bug de
corrupción de datos, no de rendimiento.

### 4.4 Backpressure

Con HTTP + NDJSON, un cliente lento genera backpressure de TCP y el server se frena solo.
Con WS, el buffer del socket crece sin límite en el server: un cliente en una laptop con
mala red consume memoria **del proceso que comanda drones**.

Se define un umbral de buffer; al cruzarlo se deja de pushear a ese cliente, se lo marca
como atrasado y se pone al día por replay. Degradar a replay es correcto; bufferear infinito
es una caída.

---

## 5. Subagentes

Se mantiene el aplanado en el stream del padre (la UI lo agradece), **con la condición de
re-emitir, no reenviar**.

El log del padre tiene **un solo escritor: el padre**. Cuando el hijo produce algo, el padre
escribe un evento nuevo en su log, con su `eventId` y su `sequence`. El `sequence` del hijo
nunca entra al contador del padre; sus coordenadas viajan como datos:

```js
{ type: "subagent.event",
  data: { callId, subagentName, childEventId, childSequence, event: {...} },
  meta: { id: "evt_...", at: "..." } }        // ids del PADRE
```

Así el cursor, el dedupe y el orden quedan intactos.

**El planner MIP se modela como tarea con id propio, no como tramo del turno del padre.** Si
tarda decenas de segundos y el operador manda otro mensaje, el turno del padre ya cerró:
atribuir los eventos del hijo al turno viejo es appendear a un turno cerrado, y al nuevo es
mentir sobre la causalidad. El evento de arranque declara la tarea, los de progreso llevan
`taskId`, y el resultado puede caer en otro turno — el `taskId` lo correlaciona.

Costo conocido de aplanar, aceptado: cancelar solo al hijo requiere un canal aparte. Con un
MIP de decenas de segundos, *"cancelar el planner sin matar la conversación"* es una feature
previsible.

---

## 6. Compactación y auditoría

**El log de eventos y el historial del modelo son dos cosas distintas.** La compactación
toca solo el segundo.

- El **log de eventos** es append-only y tiene todo. Es lo que el operador vio.
- El **historial del modelo** es lo que se manda en el prompt. Eso es lo que se compacta.

La compactación no borra nada del log: emite dos eventos **más** (`compaction.requested` /
`compaction.completed`). **El marcador visual es un evento**: la UI ve
`compaction.completed` en el medio y dibuja la separación. No hace falta un campo especial.

Se agrega sobre el diseño de referencia: `compaction.completed` lleva el rango compactado
(`fromSequence`, `toSequence`) y un hash del resumen, para que la reconstrucción sea exacta
y no inferida.

Esto da un requisito de auditoría que conviene explicitar: **se puede reconstruir no solo
qué pasó, sino qué información tenía el modelo a la vista cuando lo decidió.** Es la
diferencia entre *"el LLM se mandó una macana"* y *"el LLM decidió razonablemente con lo que
le quedaba después de compactar"*.

**Nunca compactar con blockers abiertos.** El input congelado protege la ejecución, pero el
modelo puede perder el contexto de por qué pidió eso. La compactación se hace en un
boundary de turno limpio y sin aprobaciones pendientes.

---

## 7. Migración de la base de datos

**Decisión: tabla de eventos nueva.** La tabla `ChatMessage` actual queda como archivo de
solo lectura, y un proyector convierte sus filas a eventos sintéticos al leerlas.

**Alternativas descartadas:**

*Reinterpretar las filas existentes como eventos.* Suena elegante — cada `messageData` ya es
*casi* un evento `*.completed` — pero ese "casi" es donde vive el dolor: quedan filas que son
eventos de verdad y filas que son mensajes viejos disfrazados, en la misma tabla, con los
mismos índices y las mismas queries, y cada consulta tiene que saber de qué época es cada
fila. Para siempre.

*No migrar a log de eventos: mantener mensajes y exponer un solo serializer para REST y WS.*
Más barato, y defendible si no hiciera falta reproducir estado. Pero como se explica en §3.8,
con aprobaciones humanas esta opción obliga a escribir a mano, con bugs propios, exactamente
la feature más crítica del sistema.

La tabla nueva mantiene la frontera nítida: el proyector es feo pero está en un solo lugar,
tiene tests, y el día que el historial viejo deje de importar se borra de una.

**Backfill:** los ULID de las filas históricas se generan con **el timestamp de la fila**, no
con `Date.now()`, o el orden lexicográfico miente (un bloque de ids con fecha de hoy para
conversaciones del año pasado).

**Sobre el momento:** el campo de identidad va en la primera versión o deja un agujero
permanente — en el diseño de referencia, los eventos escritos antes de cierta versión
quedaron sin id y no se pueden deduplicar nunca. Aquí se llega a tiempo: hay PK en todas las
filas existentes, así que el backfill es determinístico.

---

## 8. Qué desaparece del código actual

| Ubicación | Qué pasa |
| --- | --- |
| `client/src/components/chat/ChatMessages.jsx:50-185` — `convertMsg` | Desaparece. El server envía `parts[]` ya proyectados |
| `ChatMessages.jsx:175-184` — fallback `JSON.stringify` | Reemplazado por placeholder marcado, fuera del renderer de markdown |
| `ChatMessages.jsx:240-254` — `getResultSummary` | Desaparece. La etiqueta viene en `presentation` |
| `ChatMessages.jsx:1012` — `default:` del switch | Desaparece. Exhaustividad verificada en compilación |
| `ChatMessages.jsx:265-285` — `memoizeByIdentity` | Deja de ser un parche: el parseo caro se hace una vez en el server |
| `server/subscribers/websocketSubscriber.js:41` | Deja de regenerar el timestamp; pasa el persistido |
| `client/src/store/chat.js:87` | Deja de fabricar ids; usa `eventId` |
| `client/src/store/chat.js:120-131` — `prependMessages` | Dedupe por `eventId`, paginación por `sequence` |

Sobre la exhaustividad en el cliente: omitir el `default` solo protege si el compilador puede
verificar que todas las ramas retornan. Hay que anotar el tipo de retorno de la función de
render o activar `noImplicitReturns`; con `strict: true` a secas, TypeScript deja pasar la
rama faltante devolviendo `undefined` en silencio.

---

## 9. Orden de implementación

Los dos primeros son de seguridad. Sin ellos, el resto es decoración.

1. **Input congelado + `inputHash` verificado antes de ejecutar** (§3.1)
2. **`responderPrincipalId` + autorización server-side** (§3.4)
3. Persist-then-emit y `sequence` con único escritor (§4.1, §4.3)
4. Tabla de eventos nueva (§7)
5. `expiresAt` + revalidación del mundo al ejecutar (§3.3)
6. Idempotencia de ejecución por `callId` (§3.5)
7. Plano de progreso separado del log (§2.8)
8. Append-only aplicado en DB + hash chain (§3.6)
9. Backpressure en el WebSocket (§4.4)

Los arreglos de render (fallback marcado, fuera del markdown) son baratos e independientes:
pueden hacerse antes que todo lo demás para cortar la corrupción visual de inmediato.

---

## 10. Consecuencias

**A favor:**

- Un solo pipeline: en vivo y tras recargar corre el mismo reducer sobre el mismo log. La
  clase de bug "diverge después del reload" deja de ser posible.
- Las aprobaciones pendientes sobreviven al reload sin código dedicado.
- El registro de vuelo queda auditable y reconstruible, incluyendo qué información tenía el
  modelo a la vista.
- El cliente deja de parsear, adivinar y reconstruir; el trabajo caro se hace una vez.

**En contra:**

- Es un refactor grande que toca persistencia, transporte y render.
- Convivencia permanente (hasta que se descarte el historial viejo) con un proyector de
  filas legacy.
- Más eventos en el wire que el diseño actual: se paga verbosidad a cambio de que el
  consumidor no tenga que decidir nada.

**Riesgo principal:** implementar el modelo de eventos y dejar la aprobación humana para
"después". El valor de §3 depende de §2; hacerlos en orden inverso, o hacer solo uno,
produce lo peor de ambos.
