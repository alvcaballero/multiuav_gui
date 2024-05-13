import { IncomingMessage } from "http";
import WebSocket from "ws";
import { flatbuffers } from "flatbuffers"; // do not remove; needed by generated code
import { fb } from "./schema_generated";

export class Logger {
  name: string;
  loggedOnce: Set<string>;

  constructor(name: string) {
    this.name = name;
    this.loggedOnce = new Set();
  }
  
  log(msg: string) {
    console.log(`[${this.name}] ${msg}`);
  }
  
  logOnce(msg: string) {
    if (this.loggedOnce.has(msg)) {
      return;
    }
    this.log(msg);
    this.loggedOnce.add(msg);
  }
}

// get IP from IncomingMessage, with support for proxied requests
export function getIp(req: IncomingMessage) {
  // https://github.com/websockets/ws#how-to-get-the-ip-address-of-the-client
  // https://nodejs.org/api/net.html#net_socket_remoteaddress
  const forwardedForValues = req.headers["x-forwarded-for"];
  if (typeof forwardedForValues === "undefined") {
    return req.socket.remoteAddress;
  }
  const forwardedFor = Array.isArray(forwardedForValues) ? forwardedForValues[0] : forwardedForValues;
  return forwardedFor.split(/\s*,\s*/)[0];
}

export function getByteBuffer(data: WebSocket.Data) {
  if (data instanceof Buffer) {
    return new flatbuffers.ByteBuffer(data);
  }
  if (data instanceof ArrayBuffer) {
    return new flatbuffers.ByteBuffer(new Uint8Array(data));
  }
  return null;
}

export function getRobofleetMetadata(buf: flatbuffers.ByteBuffer) {
  const msg = fb.MsgWithMetadata.getRootAsMsgWithMetadata(buf);
  return msg._metadata();
}
