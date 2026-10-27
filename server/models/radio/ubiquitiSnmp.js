import snmp from 'net-snmp';

// UBNT-AirMAX-MIB › ubntStaTable: one row per station associated to the access point, indexed by
// `<wlan index>.<6 bytes of the MAC>`. Columns checked against an AirOS 8 Bullet AC station (2026-10).
const STA_ENTRY = '1.3.6.1.4.1.41112.1.4.7.1';
const COLUMN = {
  2: 'name',
  3: 'signal', // dBm
  4: 'noise', // dBm
  5: 'distance', // m
  10: 'ip',
  11: 'txRate', // bit/s
  12: 'rxRate', // bit/s
  15: 'connectedS',
  16: 'cinr', // dB
  17: 'txCapacity', // kbit/s
  18: 'rxCapacity', // kbit/s
};
// Not read: column 1 (the MAC, taken from the index), 6 (CCQ: meaningless on AirOS 8 AC) and 13/14 (byte
// counters, Counter64: SNMPv1 has none and they come back as garbage).

const asText = (value) => (Buffer.isBuffer(value) ? value.toString('utf8') : String(value));

/**
 * The station table out of a walk of `STA_ENTRY`. A row with no signal or noise (half walked) is dropped:
 * it cannot say anything about the link.
 * @param {{oid: string, value: any}[]} varbinds
 * @returns {{mac: string, name: string, ip: string, signal: number, noise: number}[]} (and the other columns)
 */
export function parseStations(varbinds) {
  const rows = new Map();
  for (const { oid, value } of varbinds) {
    if (!oid.startsWith(`${STA_ENTRY}.`)) continue;
    const [column, , ...mac] = oid.slice(STA_ENTRY.length + 1).split('.');
    const field = COLUMN[column];
    if (!field || mac.length !== 6) continue;
    const key = mac.join('.');
    let row = rows.get(key);
    if (!row) {
      row = { mac: mac.map((b) => Number(b).toString(16).padStart(2, '0')).join(':') };
      rows.set(key, row);
    }
    row[field] = field === 'name' || field === 'ip' ? asText(value) : Number(value);
  }
  return [...rows.values()].filter((row) => Number.isFinite(row.signal) && Number.isFinite(row.noise));
}

/**
 * Walks the AP's station table. One session per call: it is UDP, there is nothing to keep, and a poll that hangs
 * cannot leave a half-open session behind.
 * @returns {Promise<ReturnType<typeof parseStations>>}
 */
export function fetchStations({ host, community, version = '1', timeoutMs = 2000 }) {
  return new Promise((resolve, reject) => {
    const session = snmp.createSession(host, community, {
      version: version === '2c' ? snmp.Version2c : snmp.Version1,
      timeout: timeoutMs,
      retries: 1,
    });
    const varbinds = [];
    session.on('error', (err) => reject(err));
    session.subtree(
      STA_ENTRY,
      20,
      (batch) => {
        for (const vb of batch) if (!snmp.isVarbindError(vb)) varbinds.push(vb);
      },
      (err) => {
        session.close();
        if (err) reject(err);
        else resolve(parseStations(varbinds));
      }
    );
  });
}
