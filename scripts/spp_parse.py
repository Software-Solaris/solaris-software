#!/usr/bin/env python3
"""
spp_parse.py — Solaris Packet Protocol (SPP) binary dump parser.

Usage
-----
    python spp_parse.py <dump.bin> [options]

Options
-------
    --skip N        Bytes a saltar al inicio del archivo (equivale al skip de dd).
    --count N       Parsear como mucho N paquetes (0 = todos).
    --csv FILE      Exportar a CSV.
    --no-color      Desactivar colores ANSI.

Packet layout (64 bytes, little-endian)
---------------------------------------
    Offset  Size  Field
      0       1   version       (= 1)
      1       2   apid
      3       2   seq
      5       2   payloadLen
      7       4   timestampMs
     11       1   dropCounter
     12      50   payload
     62       2   crc           (0 = no computado)

APIDs conocidos
---------------
    0x0001  LOG
    0x0002  FSM state
    0x0004  BMP390  — 3 x float32: temp(°C), pressure(Pa), altitude(m)
    0x0008  ICM20948 — 12 x float32:
               accel_x/y/z, gyro_x/y/z, mag_x/y/z, quat_w, altitude_m, pressure_Pa
"""

import argparse
import csv
import os
import struct
import sys

K_SPP_PKT_VERSION    = 1
K_SPP_TOTAL_PKT_SIZE = 64
K_SPP_PKT_PAYLOAD_MAX = 50

_PRI_FMT  = "<BHHH"
_PRI_SIZE = struct.calcsize(_PRI_FMT)   # 7

_SEC_FMT  = "<IB"
_SEC_SIZE = struct.calcsize(_SEC_FMT)   # 5

_CRC_FMT  = "<H"
_CRC_SIZE = struct.calcsize(_CRC_FMT)  # 2

APID_NAMES = {
    0x0001: "LOG",
    0x0002: "FSM",
    0x0004: "BMP390",
    0x0008: "ICM20948",
}

_USE_COLOR = True

def _c(code, text):
    if not _USE_COLOR:
        return text
    return f"\033[{code}m{text}\033[0m"

def red(t):     return _c("91", t)
def green(t):   return _c("92", t)
def yellow(t):  return _c("93", t)
def cyan(t):    return _c("96", t)
def bold(t):    return _c("1",  t)
def magenta(t): return _c("95", t)
def white(t):   return _c("97", t)

def _crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc

class SPPPacket:
    def __init__(self, raw, file_offset):
        self.offset = file_offset
        pri = struct.unpack_from(_PRI_FMT, raw, 0)
        self.version, self.apid, self.seq, self.payload_len = pri
        sec = struct.unpack_from(_SEC_FMT, raw, _PRI_SIZE)
        self.timestamp_ms, self.drop_counter = sec
        self.payload = raw[_PRI_SIZE + _SEC_SIZE : K_SPP_TOTAL_PKT_SIZE - _CRC_SIZE]
        (self.crc,) = struct.unpack_from(_CRC_FMT, raw, K_SPP_TOTAL_PKT_SIZE - _CRC_SIZE)
        if self.crc != 0:
            self.crc_ok = (_crc16(raw[:K_SPP_TOTAL_PKT_SIZE - _CRC_SIZE]) == self.crc)
        else:
            self.crc_ok = True

    @property
    def apid_name(self):
        return APID_NAMES.get(self.apid, f"0x{self.apid:04X}")

    @property
    def timestamp_s(self):
        return self.timestamp_ms / 1000.0

    def decode_payload(self):
        plen = min(self.payload_len, K_SPP_PKT_PAYLOAD_MAX)
        raw  = self.payload[:plen]
        if self.apid == 0x0004 and len(raw) >= 12:
            a, p, t = struct.unpack_from("<fff", raw)
            return {"altitude_m": a, "pressure_Pa": p, "temp_C": t}
        if self.apid == 0x0008:
            if len(raw) >= 48:  # packet1: full IMU
                vals = struct.unpack_from("<12f", raw)
                keys = ["accel_x", "accel_y", "accel_z",
                        "gyro_x", "gyro_y", "gyro_z",
                        "quat_w", "quat_x", "quat_y", "quat_z",
                        "accuracy", "reserved"]
                return dict(zip(keys, vals))
            if len(raw) >= 8:   # packet2 (si se divide en nuevo firmware)
                qz, acc = struct.unpack_from("<ff", raw)
                return {"quat_z": qz, "accuracy": acc}
        if self.apid == 0x0002 and len(raw) >= 1:
            return {"fsm_state": raw[0]}
        if self.apid == 0x0001:
            return {"log": raw.rstrip(b"\x00").decode("utf-8", errors="replace")}
        return {}

def parse_dump(path, skip, count):
    parsed = 0
    with open(path, "rb") as fh:
        fh.seek(skip)
        offset = skip
        while True:
            if count and parsed >= count:
                break
            raw = fh.read(K_SPP_TOTAL_PKT_SIZE)
            if len(raw) < K_SPP_TOTAL_PKT_SIZE:
                break
            if raw in (b"\xff" * K_SPP_TOTAL_PKT_SIZE, b"\x00" * K_SPP_TOTAL_PKT_SIZE):
                offset += K_SPP_TOTAL_PKT_SIZE
                continue
            pkt = SPPPacket(raw, offset)
            if pkt.version == K_SPP_PKT_VERSION:
                yield pkt
                parsed += 1
            offset += K_SPP_TOTAL_PKT_SIZE

def find_apogee(packets):
    candidates = [p for p in packets if p.apid == 0x0004] or \
                 [p for p in packets if p.apid == 0x0008]
    best, best_alt = None, float("-inf")
    for pkt in candidates:
        alt = pkt.decode_payload().get("altitude_m")
        if alt is not None and alt > best_alt:
            best_alt, best = alt, pkt
    return (best, best_alt) if best else (None, None)

_CW = {"offset": 10, "time": 9, "apid": 10, "seq": 6, "drop": 5, "data": 58}

def _hdr():
    h = (f"{'Offset':<{_CW['offset']}}{'Time(s)':<{_CW['time']}}"
         f"{'APID':<{_CW['apid']}}{'Seq':<{_CW['seq']}}{'Drop':<{_CW['drop']}}"
         f"{'Payload':<{_CW['data']}}")
    sep = "\u2500" * len(h)
    print(bold(cyan(sep))); print(bold(white(h))); print(bold(cyan(sep)))

def display_packet(pkt, apogee_pkt):
    fields    = pkt.decode_payload()
    is_apog   = apogee_pkt is not None and pkt.offset == apogee_pkt.offset
    crc_warn  = red(" [BAD CRC]") if not pkt.crc_ok else ""
    if pkt.apid == 0x0004:
        t   = fields.get("temp_C", float("nan"))
        p   = fields.get("pressure_Pa", float("nan"))
        alt = fields.get("altitude_m", float("nan"))
        ds  = f"T={t:+7.2f}\u00b0C  P={p:9.1f}Pa  Alt={alt:8.2f}m"
        ds  = magenta(bold(f"\u2605 APOGEO \u2605  {ds}")) if is_apog else green(ds)
    elif pkt.apid == 0x0008:
        if "accel_x" in fields:
            ax,ay,az = fields.get("accel_x",0), fields.get("accel_y",0), fields.get("accel_z",0)
            gx,gy,gz = fields.get("gyro_x",0),  fields.get("gyro_y",0),  fields.get("gyro_z",0)
            qw,qx,qy,qz = fields.get("quat_w",0), fields.get("quat_x",0), fields.get("quat_y",0), fields.get("quat_z",0)
            ds = cyan(f"Acc=({ax:+.2f},{ay:+.2f},{az:+.2f})g  "
                      f"Gyr=({gx:+.1f},{gy:+.1f},{gz:+.1f})°/s  "
                      f"Q=({qw:.3f},{qx:.3f},{qy:.3f},{qz:.3f})")
        else:
            qz  = fields.get("quat_z",  float("nan"))
            acc = fields.get("accuracy", float("nan"))
            ds  = cyan(f"qz={qz:.3f}  accuracy={acc:.3f}")

    elif pkt.apid == 0x0002:
        ds = yellow(f"FSM state={fields.get('fsm_state','?')}")
    elif pkt.apid == 0x0001:
        ds = white(f"LOG: {fields.get('log','')}")
    else:
        ds = red(f"APID=0x{pkt.apid:04X}  raw={pkt.payload[:8].hex(' ')}...")
    prefix = bold(magenta("\u25b6 ")) if is_apog else "  "
    print(f"{prefix}{pkt.offset:<{_CW['offset']}}{pkt.timestamp_s:<{_CW['time']}.3f}"
          f"{pkt.apid_name:<{_CW['apid']}}{pkt.seq:<{_CW['seq']}}{pkt.drop_counter:<{_CW['drop']}}"
          f"{ds}{crc_warn}")

def write_csv(packets, apogee_pkt, path_out):
    fields = ["offset","timestamp_ms","apid","apid_name","seq","drop_counter","crc_ok","is_apogee",
              "temp_C","pressure_Pa","altitude_m",
              "accel_x","accel_y","accel_z",
              "gyro_x","gyro_y","gyro_z",
              "mag_x","mag_y","mag_z",
              "quat_w","quat_x","quat_y","quat_z","accuracy",
              "fsm_state","log"]

    with open(path_out, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=fields, extrasaction="ignore")
        w.writeheader()
        for pkt in packets:
            row = {"offset": pkt.offset, "timestamp_ms": pkt.timestamp_ms,
                   "apid": f"0x{pkt.apid:04X}", "apid_name": pkt.apid_name,
                   "seq": pkt.seq, "drop_counter": pkt.drop_counter,
                   "crc_ok": pkt.crc_ok,
                   "is_apogee": apogee_pkt is not None and pkt.offset == apogee_pkt.offset}
            row.update(pkt.decode_payload())
            w.writerow(row)
    print(bold(green(f"\n  CSV guardado en: {path_out}")))

def print_summary(packets, apogee_pkt, apogee_alt):
    by_apid = {}
    for p in packets:
        by_apid[p.apid_name] = by_apid.get(p.apid_name, 0) + 1
    bad_crc = sum(1 for p in packets if not p.crc_ok)
    sep = "\u2550" * 70
    print("\n" + bold(cyan(sep)))
    print(bold(white("  RESUMEN DEL VUELO")))
    print(bold(cyan(sep)))
    print(f"  Paquetes totales : {bold(str(len(packets)))}")
    for name, cnt in sorted(by_apid.items(), key=lambda x: -x[1]):
        print(f"    {name:<12}: {cnt}")
    if bad_crc:
        print(f"  {red(f'CRC erroneos : {bad_crc}')}")
    if apogee_pkt and apogee_alt is not None:
        f = apogee_pkt.decode_payload()
        print(f"\n   APOGEO DETECTADO  ")
        print(f"     Altitud     : {bold(green(f'{apogee_alt:.2f} m'))}")
        print(f"     Timestamp   : {bold(green(f'{apogee_pkt.timestamp_s:.3f} s'))}")
        print(f"     Offset dump : {bold(str(apogee_pkt.offset))} bytes")
        print(f"     Seq         : {bold(str(apogee_pkt.seq))}")
        if "temp_C" in f:     print(f"     Temperatura : {f['temp_C']:.2f} \u00b0C")
        if "pressure_Pa" in f: print(f"     Presion     : {f['pressure_Pa']:.1f} Pa")
    else:
        print(f"\n  {yellow('No se encontraron datos de altitud.')}")
    print(bold(cyan(sep)) + "\n")

def main():
    global _USE_COLOR
    ap = argparse.ArgumentParser(description="Parsea un dump binario SPP de la SD del cohete Solaris.")
    ap.add_argument("dump")
    ap.add_argument("--skip",     type=int, default=0)
    ap.add_argument("--count",    type=int, default=0)
    ap.add_argument("--csv",      metavar="FILE")
    ap.add_argument("--no-color", action="store_true")
    args = ap.parse_args()
    if args.no_color or not sys.stdout.isatty():
        _USE_COLOR = False
    if not os.path.isfile(args.dump):
        print(red(f"Error: no se encuentra '{args.dump}'"), file=sys.stderr); sys.exit(1)
    sz = os.path.getsize(args.dump)
    print(bold(f"\n  SPP Parser \u2014 Solaris Telemetry"))
    print(f"  Archivo : {args.dump}  ({sz/1024/1024:.2f} MiB)\n")
    print("  Leyendo paquetes...", end="", flush=True)
    packets = list(parse_dump(args.dump, args.skip, args.count))
    print(f" {len(packets)} paquetes validos.\n")
    apogee_pkt, apogee_alt = find_apogee(packets)
    _hdr()
    for pkt in packets:
        display_packet(pkt, apogee_pkt)
    print_summary(packets, apogee_pkt, apogee_alt)
    if args.csv:
        write_csv(packets, apogee_pkt, args.csv)

if __name__ == "__main__":
    main()
