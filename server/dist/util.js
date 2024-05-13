"use strict";
Object.defineProperty(exports, "__esModule", { value: true });
exports.getRobofleetMetadata = exports.getByteBuffer = exports.getIp = exports.Logger = void 0;
const flatbuffers_1 = require("flatbuffers"); // do not remove; needed by generated code
const schema_generated_1 = require("./schema_generated");
class Logger {
    constructor(name) {
        this.name = name;
        this.loggedOnce = new Set();
    }
    log(msg) {
        console.log(`[${this.name}] ${msg}`);
    }
    logOnce(msg) {
        if (this.loggedOnce.has(msg)) {
            return;
        }
        this.log(msg);
        this.loggedOnce.add(msg);
    }
}
exports.Logger = Logger;
// get IP from IncomingMessage, with support for proxied requests
function getIp(req) {
    // https://github.com/websockets/ws#how-to-get-the-ip-address-of-the-client
    // https://nodejs.org/api/net.html#net_socket_remoteaddress
    const forwardedForValues = req.headers["x-forwarded-for"];
    if (typeof forwardedForValues === "undefined") {
        return req.socket.remoteAddress;
    }
    const forwardedFor = Array.isArray(forwardedForValues) ? forwardedForValues[0] : forwardedForValues;
    return forwardedFor.split(/\s*,\s*/)[0];
}
exports.getIp = getIp;
function getByteBuffer(data) {
    if (data instanceof Buffer) {
        return new flatbuffers_1.flatbuffers.ByteBuffer(data);
    }
    if (data instanceof ArrayBuffer) {
        return new flatbuffers_1.flatbuffers.ByteBuffer(new Uint8Array(data));
    }
    return null;
}
exports.getByteBuffer = getByteBuffer;
function getRobofleetMetadata(buf) {
    const msg = schema_generated_1.fb.MsgWithMetadata.getRootAsMsgWithMetadata(buf);
    return msg._metadata();
}
exports.getRobofleetMetadata = getRobofleetMetadata;
