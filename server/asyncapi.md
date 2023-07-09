# GCS Websockets API for multiUAV 1.8.0 documentation

In addition to the REST API, the GCS has a WebSocket endpoint for live updates.

This only consists in the fact that when a connection is established, the server sends UPDATE messages to the client consecutively to update the list of devices and their telemetry. 

## Table of Contents

* [Servers](#servers)
  * [public](#public-server)
* [Operations](#operations)
  * [SUB /](#sub--operation)
* [Schemas](#schemas)

## Servers

### `public` Server

* URL: `localhost:4000/api/socket`
* Protocol: `wss`

Public server available without authentication



## Operations

### SUB `/` Operation

* Operation ID: `sendMessage`

Messages that you receive from the websocket

#### Message `update`

*array with serverstate,  device and positions*

element of the array contains a Json type object of devices, positions.
If a message does not contain any object of a certain type, the key will not be included in the JSON structure. In most cases, a message contains only one type of objects.

##### Payload

| Name | Type | Description | Value | Constraints | Notes |
|---|---|---|---|---|---|
| devices | array(object>) | - | - | - | - |
| devices.id | integer | - | - | - | - |
| devices.name | string | - | - | - | - |
| devices.status | string | - | - | - | - |
| devices.lastUpdate | string | in IS0 8601 format. eg. `1963-11-22T18:30:00Z` | - | format (`date-time`) | - |
| devices.category | string | - | - | - | - |
| position | array(object) | - | - | - | - |
| position.deviceId | integer | - | - | - | - |
| position.deviceTime | string | in IS0 8601 format. eg. `1963-11-22T18:30:00Z` | - | format (`date-time`) | - |
| position.latitude | number | - | - | - | - |
| position.longitude | number | - | - | - | - |
| position.altitude | number | - | - | - | - |
| position.speed | number | in knots | - | - | - |
| position.course | number | - | - | - | - |
| position.attributes | object | - | - | - | **additional properties are allowed** |
| position.attributes.protocol | string | - | - | - | - |
| position.attributes.mission_state | string | - | - | - | - |
| position.attributes.wp_reached | integer | - | - | - | - |
| position.attributes.uav_state | string | - | - | - | - |
| position.attributes.landed_state | string | - | - | - | - |
| position.attributes.alarm | string | - | - | - | - |
| RosState | boolean | - | - | - | - |

> Examples of payload _(generated)_

```json
{
  "devices": [
    {
      "id": 0,
      "name": "string",
      "status": "string",
      "lastUpdate": "2019-08-24T14:15:22Z",
      "category": "string"
    }
  ],
  "position": [
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
  ],
  "RosState": true
}
```
# Schemas

<h2 id="tocS_Device">Update</h2>
<!-- backwards compatibility -->
<a id="schemadevice"></a>
<a id="schema_Device"></a>
<a id="tocSdevice"></a>
<a id="tocsdevice"></a>

```json
{
  "devices": array(Object),
  "position": array(Object),
  "RosState": "Bolean",
}

```

### Properties

|Name|Type|Required|Restrictions|Description|
|---|---|---|---|---|
|devices|array(Device)|false|none|List of added devices |
|position|array(Position)|false|none|list of telemetry of addad devices|
|RosState|bool|false|none|State of Ros conexion|



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





