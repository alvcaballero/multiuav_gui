---
title: GCS (Ground  control Station MultiUAV) v2.3.1
language_tabs:
  - shell: Shell
  - http: HTTP
  - javascript: JavaScript
  - ruby: Ruby
  - python: Python
  - php: PHP
  - java: Java
  - go: Go
toc_footers:
  - <a href="https://grvc.us.es/">Find more about the project</a>
includes: []
search: true
highlight_theme: darkula
headingLevel: 2

---

<!-- Generator: Widdershins v4.0.1 -->

<h1 id="gcs-ground-control-station-multiuav-">GCS (Ground  control Station MultiUAV) v2.3.1</h1>

> Scroll down for code samples, example requests and responses. Select a language for code samples from the tabs above or the mobile navigation menu.

Descripcion de API de Alto nivel para comunicacion de Ros  con UAV.
some usefull link:
- [Repositorio](https://github.com/alvcaballero/multiuav_gui/tree/react)

Base URLs:

* <a href="https://localhost:4000/api/">https://localhost:4000/api/</a>

Email: <a href="mailto:arpoma167@gmail.com">Support</a> 

<h1 id="gcs-ground-control-station-multiuav--devices">Devices</h1>

Management of UAV-devices

## get__devices

`GET /devices`

*Fetch a list of Devices*

Return the list of devices

<h3 id="get__devices-parameters">Parameters</h3>

|Name|In|Type|Required|Description|
|---|---|---|---|---|
|token|header|string|false|Token de autenticacion|

> Example responses

> 200 Response

```json
[
  {
    "id": 0,
    "name": "string",
    "status": "string",
    "lastUpdate": "2019-08-24T14:15:22Z",
    "category": "string"
  }
]
```

<h3 id="get__devices-responses">Responses</h3>

|Status|Meaning|Description|Schema|
|---|---|---|---|
|200|[OK](https://tools.ietf.org/html/rfc7231#section-6.3.1)|(ok) list of devices|Inline|
|400|[Bad Request](https://tools.ietf.org/html/rfc7231#section-6.5.1)|No permission|None|

<h3 id="get__devices-responseschema">Response Schema</h3>

Status Code **200**

|Name|Type|Required|Restrictions|Description|
|---|---|---|---|---|
|*anonymous*|[[Device](#schemadevice)]|false|none|none|
|» id|integer|false|none|none|
|» name|string|false|none|none|
|» status|string|false|none|none|
|» lastUpdate|string(date-time)|false|none|in IS0 8601 format. eg. `1963-11-22T18:30:00Z`|
|» category|string|false|none|none|

<aside class="success">
This operation does not require authentication
</aside>

## post__devices


`POST /devices`

*Create a Device*

> Body parameter

```json
{
  "uav_id": "string",
  "uav_type": "string"
}
```

<h3 id="post__devices-parameters">Parameters</h3>

|Name|In|Type|Required|Description|
|---|---|---|---|---|
|token|header|string|false|Token de autenticacion|
|body|body|[BodyAddUAV](#schemabodyadduav)|true|none|

> Example responses

> 200 Response

```json
{
  "uav_id": "string",
  "uav_type": "string"
}
```

<h3 id="post__devices-responses">Responses</h3>

|Status|Meaning|Description|Schema|
|---|---|---|---|
|200|[OK](https://tools.ietf.org/html/rfc7231#section-6.3.1)|OK|[BodyAddUAV](#schemabodyadduav)|

<aside class="success">
This operation does not require authentication
</aside>

## delete__devices_{id}

`DELETE /devices/{id}`

*Delete a Device*

<h3 id="delete__devices_{id}-parameters">Parameters</h3>

|Name|In|Type|Required|Description|
|---|---|---|---|---|
|id|path|integer|true|none|

> Example responses

<h3 id="delete__devices_{id}-responses">Responses</h3>

|Status|Meaning|Description|Schema|
|---|---|---|---|
|204|[No Content](https://tools.ietf.org/html/rfc7231#section-6.3.5)|No Content|None|

<h3 id="delete__devices_{id}-responseschema">Response Schema</h3>

<aside class="success">
This operation does not require authentication
</aside>

<h1 id="gcs-ground-control-station-multiuav--positions">Positions</h1>

Obtains raw data from the UAV

## get__postitions


`GET /postitions`

*Fetches a list of Positions*

We strongly recommend using Websocket instead of periodically polling positions endpoint. Without any params, it returns a list of last known positions for all the user's Devices. _from_ and _to_ fields are not required with _id_.

> Example responses

> 200 Response

```json
[
  {
    "deviceId": 0,
    "deviceTime": "2019-08-24T14:15:22Z",
    "latitude": 0,
    "longitude": 0,
    "altitude": 0,
    "speed": 0,
    "course": 0,
    "attributes": {
      "protocol": "string",
      "mission_state": "string",
      "wp_reached": 0,
      "uav_state": "string",
      "landed_state": "string",
      "alarm": "string"
    }
  }
]
```

<h3 id="get__postitions-responses">Responses</h3>

|Status|Meaning|Description|Schema|
|---|---|---|---|
|200|[OK](https://tools.ietf.org/html/rfc7231#section-6.3.1)|OK|Inline|

<h3 id="get__postitions-responseschema">Response Schema</h3>

Status Code **200**

|Name|Type|Required|Restrictions|Description|
|---|---|---|---|---|
|*anonymous*|[[Position](#schemaposition)]|false|none|none|
|» deviceId|integer|false|none|none|
|» deviceTime|string(date-time)|false|none|in IS0 8601 format. eg. `1963-11-22T18:30:00Z`|
|» latitude|number|false|none|none|
|» longitude|number|false|none|none|
|» altitude|number|false|none|none|
|» speed|number|false|none|in knots|
|» course|number|false|none|none|
|» attributes|object|false|none|none|
|»» protocol|string|false|none|none|
|»» mission_state|string|false|none|none|
|»» wp_reached|integer|false|none|none|
|»» uav_state|string|false|none|none|
|»» landed_state|string|false|none|none|
|»» alarm|string|false|none|none|

<aside class="success">
This operation does not require authentication
</aside>

<h1 id="gcs-ground-control-station-multiuav--missions">Missions</h1>

Management the mission

## put__loadmission_{id}


`PUT /loadmission/{id}`

*Load mission to devices*

without any parameter,load the mission all UAV's

<h3 id="put__loadmission_{id}-parameters">Parameters</h3>

|Name|In|Type|Required|Description|
|---|---|---|---|---|
|id|path|integer|false|none|

> Example responses

<h3 id="put__loadmission_{id}-responses">Responses</h3>

|Status|Meaning|Description|Schema|
|---|---|---|---|
|204|[No Content](https://tools.ietf.org/html/rfc7231#section-6.3.5)|No Content|None|

<h3 id="put__loadmission_{id}-responseschema">Response Schema</h3>

<aside class="success">
This operation does not require authentication
</aside>

## put__commandmission_{id}


`PUT /commandmission/{id}`

*Command to exec mission*

without any parameter,load the mission all UAV's

<h3 id="put__commandmission_{id}-parameters">Parameters</h3>

|Name|In|Type|Required|Description|
|---|---|---|---|---|
|id|path|integer|false|none|

> Example responses

<h3 id="put__commandmission_{id}-responses">Responses</h3>

|Status|Meaning|Description|Schema|
|---|---|---|---|
|204|[No Content](https://tools.ietf.org/html/rfc7231#section-6.3.5)|No Content|None|

<h3 id="put__commandmission_{id}-responseschema">Response Schema</h3>

<aside class="success">
This operation does not require authentication
</aside>

<h1 id="gcs-ground-control-station-multiuav--commands">Commands</h1>

Send command the UAV

## post__comands_send



`POST /comands/send`

*Send command to devices*

Send command to  devices

> Body parameter

```json
{
  "deviceId": 0,
  "attributes": {}
}
```

<h3 id="post__comands_send-parameters">Parameters</h3>

|Name|In|Type|Required|Description|
|---|---|---|---|---|
|body|body|[Command](#schemacommand)|true|none|

> Example responses

> 200 Response

```json
{
  "deviceId": 0,
  "attributes": {}
}
```

<h3 id="post__comands_send-responses">Responses</h3>

|Status|Meaning|Description|Schema|
|---|---|---|---|
|200|[OK](https://tools.ietf.org/html/rfc7231#section-6.3.1)|Command sent|[Command](#schemacommand)|
|400|[Bad Request](https://tools.ietf.org/html/rfc7231#section-6.5.1)|Command no match|None|

<h3 id="post__comands_send-responseschema">Response Schema</h3>

<aside class="success">
This operation does not require authentication
</aside>

# Schemas

<h2 id="tocS_BodyAddUAV">BodyAddUAV</h2>
<!-- backwards compatibility -->
<a id="schemabodyadduav"></a>
<a id="schema_BodyAddUAV"></a>
<a id="tocSbodyadduav"></a>
<a id="tocsbodyadduav"></a>

```json
{
  "uav_id": "string",
  "uav_type": "string"
}

```

### Properties

|Name|Type|Required|Restrictions|Description|
|---|---|---|---|---|
|uav_id|string|false|none|none|
|uav_type|string|false|none|none|

<h2 id="tocS_Device">Device</h2>
<!-- backwards compatibility -->
<a id="schemadevice"></a>
<a id="schema_Device"></a>
<a id="tocSdevice"></a>
<a id="tocsdevice"></a>

```json
{
  "id": 0,
  "name": "string",
  "status": "string",
  "lastUpdate": "2019-08-24T14:15:22Z",
  "category": "string"
}

```

### Properties

|Name|Type|Required|Restrictions|Description|
|---|---|---|---|---|
|id|integer|false|none|none|
|name|string|false|none|none|
|status|string|false|none|none|
|lastUpdate|string(date-time)|false|none|in IS0 8601 format. eg. `1963-11-22T18:30:00Z`|
|category|string|false|none|none|

<h2 id="tocS_Position">Position</h2>
<!-- backwards compatibility -->
<a id="schemaposition"></a>
<a id="schema_Position"></a>
<a id="tocSposition"></a>
<a id="tocsposition"></a>

```json
{
  "deviceId": 0,
  "deviceTime": "2019-08-24T14:15:22Z",
  "latitude": 0,
  "longitude": 0,
  "altitude": 0,
  "speed": 0,
  "course": 0,
  "attributes": {
    "protocol": "string",
    "mission_state": "string",
    "wp_reached": 0,
    "uav_state": "string",
    "landed_state": "string",
    "alarm": "string"
  }
}

```

### Properties

|Name|Type|Required|Restrictions|Description|
|---|---|---|---|---|
|deviceId|integer|false|none|none|
|deviceTime|string(date-time)|false|none|in IS0 8601 format. eg. `1963-11-22T18:30:00Z`|
|latitude|number|false|none|none|
|longitude|number|false|none|none|
|altitude|number|false|none|none|
|speed|number|false|none|in knots|
|course|number|false|none|none|
|attributes|object|false|none|none|
|» protocol|string|false|none|none|
|» mission_state|string|false|none|none|
|» wp_reached|integer|false|none|none|
|» uav_state|string|false|none|none|
|» landed_state|string|false|none|none|
|» alarm|string|false|none|none|

<h2 id="tocS_Command">Command</h2>
<!-- backwards compatibility -->
<a id="schemacommand"></a>
<a id="schema_Command"></a>
<a id="tocScommand"></a>
<a id="tocscommand"></a>

```json
{
  "deviceId": 0,
  "attributes": {}
}

```

### Properties

|Name|Type|Required|Restrictions|Description|
|---|---|---|---|---|
|deviceId|integer|false|none|none|
|attributes|object|false|none|none|

<h2 id="tocS_Notification">Notification</h2>
<!-- backwards compatibility -->
<a id="schemanotification"></a>
<a id="schema_Notification"></a>
<a id="tocSnotification"></a>
<a id="tocsnotification"></a>

```json
{
  "id": 0,
  "type": "string",
  "attributes": {}
}

```

### Properties

|Name|Type|Required|Restrictions|Description|
|---|---|---|---|---|
|id|integer|false|none|none|
|type|string|false|none|none|
|attributes|object|false|none|none|

