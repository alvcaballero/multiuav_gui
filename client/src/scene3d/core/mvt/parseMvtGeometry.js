/**
 * Decodes MVT geometry commands into rings of [x, y, x, y, ...] flat arrays.
 * Commands: MoveTo=1, LineTo=2, ClosePath=7
 * Values are zigzag-encoded deltas.
 */
const zigzag = (n) => (n >> 1) ^ -(n & 1);

export const parseMvtGeometry = (geometry) => {
  const rings = [];
  let i = 0;
  let cx = 0;
  let cy = 0;
  let current = null;

  while (i < geometry.length) {
    const cmdInt = geometry[i++];
    const cmd = cmdInt & 0x7;
    const count = cmdInt >> 3;

    if (cmd === 1) {
      // MoveTo — starts a new ring
      for (let c = 0; c < count; c++) {
        cx += zigzag(geometry[i++]);
        cy += zigzag(geometry[i++]);
        if (current && current.length > 0) rings.push(current);
        current = [cx, cy];
      }
    } else if (cmd === 2) {
      // LineTo — continues current ring
      for (let c = 0; c < count; c++) {
        cx += zigzag(geometry[i++]);
        cy += zigzag(geometry[i++]);
        current.push(cx, cy);
      }
    } else if (cmd === 7) {
      // ClosePath — closes polygon ring (repeat first point)
      if (current && current.length >= 4) {
        current.push(current[0], current[1]);
        rings.push(current);
        current = null;
      }
    }
  }

  if (current && current.length > 0) rings.push(current);
  return rings;
};
