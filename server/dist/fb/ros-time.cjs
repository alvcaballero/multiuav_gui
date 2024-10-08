"use strict";
// automatically generated by the FlatBuffers compiler, do not modify
Object.defineProperty(exports, "__esModule", { value: true });
exports.RosTime = void 0;
var RosTime = /** @class */ (function () {
    function RosTime() {
        this.bb = null;
        this.bb_pos = 0;
    }
    RosTime.prototype.__init = function (i, bb) {
        this.bb_pos = i;
        this.bb = bb;
        return this;
    };
    RosTime.prototype.secs = function () {
        return this.bb.readUint32(this.bb_pos);
    };
    RosTime.prototype.nsecs = function () {
        return this.bb.readUint32(this.bb_pos + 4);
    };
    RosTime.sizeOf = function () {
        return 8;
    };
    RosTime.createRosTime = function (builder, secs, nsecs) {
        builder.prep(4, 8);
        builder.writeInt32(nsecs);
        builder.writeInt32(secs);
        return builder.offset();
    };
    return RosTime;
}());
exports.RosTime = RosTime;
