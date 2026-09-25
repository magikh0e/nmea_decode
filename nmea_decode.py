#!/usr/bin/env python3
"""
nmea_decode.py - NMEA 0183 sentence decoder (stdlib only; pyserial optional)
by: magikh0e

Sources:
  nmea_decode.py log.nmea                 # file
  cat log.nmea | nmea_decode.py -         # stdin
  nmea_decode.py serial:/dev/ttyUSB0@9600 # serial (needs: pip install pyserial)
  nmea_decode.py serial:COM3@4800         # serial on Windows
  nmea_decode.py tcp:192.168.1.50:10110   # NMEA over TCP (gpsd raw, gateways)
  nmea_decode.py udp:10110                # UDP broadcast (OpenCPN, marine gateways)
  nmea_decode.py -s '$GPGGA,...*47'       # single sentence

Output modes:
  (default)   pretty per-sentence decode
  --json      one JSON object per line
  --fix       consolidated fix summary per epoch (position, speed, sats, DOP)
Options:
  --only GGA,RMC   filter by sentence type
  --strict         drop sentences with bad/missing checksums
  --stats          print sentence counts / errors at the end (stderr)

Decoded: GGA RMC GSA GSV VTG GLL ZDA GNS GST HDT HDG TXT, plus AIS (!AIVDM/!AIVDO) envelope.
Anything else is split into raw fields.
"""
import argparse
import json
import socket
import sys
from collections import Counter

# --------------------------------------------------------------------------- lookup tables

TALKERS = {
    "GP": "GPS", "GL": "GLONASS", "GA": "Galileo", "GB": "BeiDou", "BD": "BeiDou",
    "GQ": "QZSS", "QZ": "QZSS", "GI": "NavIC", "GN": "Multi-GNSS", "II": "Integrated Instrument",
    "IN": "Integrated Navigation", "HC": "Compass (magnetic)", "HE": "Gyro (north seeking)",
    "AI": "AIS", "EC": "ECDIS", "SD": "Depth Sounder", "WI": "Weather Instrument",
    "YX": "Transducer", "P": "Proprietary",
}

GGA_QUALITY = {
    0: "Invalid", 1: "GPS fix", 2: "DGPS fix", 3: "PPS fix", 4: "RTK fixed",
    5: "RTK float", 6: "Estimated (DR)", 7: "Manual input", 8: "Simulation",
}

FAA_MODE = {
    "A": "Autonomous", "D": "Differential", "E": "Estimated (DR)", "F": "RTK float",
    "M": "Manual", "N": "Not valid", "P": "Precise", "R": "RTK fixed", "S": "Simulator",
}

NAV_STATUS = {"S": "Safe", "C": "Caution", "U": "Unsafe", "V": "Not valid"}

GNSS_SYSTEM_ID = {1: "GPS", 2: "GLONASS", 3: "Galileo", 4: "BeiDou", 5: "QZSS", 6: "NavIC"}

SENTENCE_NAMES = {
    "GGA": "Fix Data", "RMC": "Recommended Minimum", "GSA": "DOP & Active Satellites",
    "GSV": "Satellites in View", "VTG": "Course & Speed over Ground",
    "GLL": "Geographic Position", "ZDA": "Time & Date", "GNS": "GNSS Fix Data",
    "GST": "Pseudorange Error Statistics", "HDT": "Heading (True)",
    "HDG": "Heading, Deviation & Variation", "TXT": "Text Message",
    "VDM": "AIS (other vessels)", "VDO": "AIS (own vessel)",
}

# --------------------------------------------------------------------------- field helpers


def f_(v):
    try:
        return float(v) if v not in ("", None) else None
    except ValueError:
        return None


def i_(v):
    try:
        return int(v) if v not in ("", None) else None
    except ValueError:
        try:
            return int(float(v))
        except (ValueError, TypeError):
            return None


def coord(value, hemi):
    """ddmm.mmmm / dddmm.mmmm + N/S/E/W -> signed decimal degrees."""
    if not value or not hemi:
        return None
    try:
        dot = value.find(".")
        if dot == -1:
            dot = len(value)
        deg = int(value[: dot - 2])
        mins = float(value[dot - 2:])
    except ValueError:
        return None
    d = deg + mins / 60.0
    return round(-d if hemi in ("S", "W") else d, 7)


def to_dms(dec, is_lat):
    if dec is None:
        return None
    hemi = ("N" if dec >= 0 else "S") if is_lat else ("E" if dec >= 0 else "W")
    a = abs(dec)
    d = int(a)
    m = int((a - d) * 60)
    s = (a - d - m / 60) * 3600
    return f"{d}°{m:02d}'{s:06.3f}\"{hemi}"


def utc_time(v):
    if not v or len(v) < 6:
        return None
    return f"{v[0:2]}:{v[2:4]}:{v[4:]}"


def ddmmyy(v):
    if not v or len(v) != 6:
        return None
    yy = int(v[4:6])
    year = 2000 + yy if yy < 80 else 1900 + yy
    return f"{year:04d}-{v[2:4]}-{v[0:2]}"


def lookup(table, key):
    if key in (None, ""):
        return None
    return f"{key} ({table.get(key, 'Unknown')})"


# --------------------------------------------------------------------------- framing


def nmea_checksum(body):
    c = 0
    for ch in body:
        c ^= ord(ch)
    return c


def parse_frame(line):
    """Split a raw line into talker/type/fields and verify checksum. Returns dict or None."""
    starts = [p for p in (line.find("$"), line.find("!")) if p != -1]
    if not starts:
        return None
    s = line[min(starts):].strip()
    start_char = s[0]
    body, star, cs = s[1:].partition("*")
    checksum_ok = None
    given = None
    if star:
        given = cs[:2].upper()
        try:
            checksum_ok = int(given, 16) == nmea_checksum(body)
        except ValueError:
            checksum_ok = False
    fields = body.split(",")
    addr = fields[0]
    if addr.startswith("P"):
        talker, stype = "P", addr[1:]
    else:
        talker, stype = addr[:2], addr[2:]
    return {
        "raw": s,
        "start": start_char,
        "talker": talker,
        "type": stype,
        "fields": fields[1:],
        "checksum_ok": checksum_ok,
        "checksum": given,
        "checksum_calc": f"{nmea_checksum(body):02X}",
    }


# --------------------------------------------------------------------------- decoders
# Each takes the field list (after the address) and returns an ordered dict.


def pad(f, n):
    return f + [""] * (n - len(f))


def d_gga(f):
    f = pad(f, 14)
    lat, lon = coord(f[1], f[2]), coord(f[3], f[4])
    q = i_(f[5])
    return {
        "time_utc": utc_time(f[0]),
        "lat": lat, "lon": lon,
        "fix_quality": f"{q} ({GGA_QUALITY.get(q, 'Unknown')})" if q is not None else None,
        "satellites_used": i_(f[6]),
        "hdop": f_(f[7]),
        "altitude_msl_m": f_(f[8]),
        "geoid_separation_m": f_(f[10]),
        "dgps_age_s": f_(f[12]),
        "dgps_station": f[13] or None,
    }


def d_rmc(f):
    f = pad(f, 13)
    var = f_(f[9])
    if var is not None and f[10] == "W":
        var = -var
    kn = f_(f[6])
    return {
        "time_utc": utc_time(f[0]),
        "status": {"A": "A (Active/valid)", "V": "V (Void/warning)"}.get(f[1], f[1] or None),
        "lat": coord(f[2], f[3]), "lon": coord(f[4], f[5]),
        "speed_kn": kn,
        "speed_kmh": round(kn * 1.852, 2) if kn is not None else None,
        "course_true_deg": f_(f[7]),
        "date": ddmmyy(f[8]),
        "mag_variation_deg": var,
        "mode": lookup(FAA_MODE, f[11]),
        "nav_status": lookup(NAV_STATUS, f[12]),
    }


def d_gsa(f):
    f = pad(f, 18)
    ft = i_(f[1])
    sysid = i_(f[17])
    return {
        "selection": {"M": "Manual", "A": "Automatic"}.get(f[0], f[0] or None),
        "fix_type": {1: "No fix", 2: "2D", 3: "3D"}.get(ft, ft),
        "sats_used": [int(p) for p in f[2:14] if p.strip().isdigit()],
        "pdop": f_(f[14]), "hdop": f_(f[15]), "vdop": f_(f[16]),
        "system": f"{sysid} ({GNSS_SYSTEM_ID.get(sysid, 'Unknown')})" if sysid else None,
    }


def d_gsv(f):
    f = pad(f, 3)
    rest = f[3:]
    signal = None
    if len(rest) % 4 == 1:  # NMEA 4.1 signal ID at the end
        signal = rest[-1]
        rest = rest[:-1]
    sats = []
    for k in range(0, len(rest), 4):
        g = pad(rest[k:k + 4], 4)
        if not g[0]:
            continue
        sats.append({"prn": i_(g[0]), "elev": i_(g[1]), "az": i_(g[2]), "snr": i_(g[3])})
    return {
        "msg": f"{f[1]}/{f[0]}",
        "sats_in_view": i_(f[2]),
        "signal_id": signal or None,
        "satellites": sats,
    }


def d_vtg(f):
    f = pad(f, 9)
    if f[1] in ("T", ""):  # modern: val,T,val,M,val,N,val,K,mode
        return {
            "course_true_deg": f_(f[0]), "course_mag_deg": f_(f[2]),
            "speed_kn": f_(f[4]), "speed_kmh": f_(f[6]), "mode": lookup(FAA_MODE, f[8]),
        }
    return {  # legacy NMEA 1.x: true,mag,kn,kmh
        "course_true_deg": f_(f[0]), "course_mag_deg": f_(f[1]),
        "speed_kn": f_(f[2]), "speed_kmh": f_(f[3]),
    }


def d_gll(f):
    f = pad(f, 7)
    return {
        "lat": coord(f[0], f[1]), "lon": coord(f[2], f[3]),
        "time_utc": utc_time(f[4]),
        "status": {"A": "A (Valid)", "V": "V (Invalid)"}.get(f[5], f[5] or None),
        "mode": lookup(FAA_MODE, f[6]),
    }


def d_zda(f):
    f = pad(f, 6)
    date = None
    if f[1] and f[2] and f[3]:
        date = f"{int(f[3]):04d}-{int(f[2]):02d}-{int(f[1]):02d}"
    tz = None
    if f[4] or f[5]:
        tz = f"{i_(f[4]) or 0:+03d}:{abs(i_(f[5]) or 0):02d}"
    return {"time_utc": utc_time(f[0]), "date": date, "local_tz_offset": tz}


def d_gns(f):
    f = pad(f, 13)
    modes = f[5]
    systems = ["GPS", "GLONASS", "Galileo", "BeiDou", "QZSS", "NavIC"]
    mode_desc = None
    if modes:
        mode_desc = ", ".join(
            f"{systems[k] if k < len(systems) else '?'}={FAA_MODE.get(c, c)}"
            for k, c in enumerate(modes)
        )
    return {
        "time_utc": utc_time(f[0]),
        "lat": coord(f[1], f[2]), "lon": coord(f[3], f[4]),
        "mode": mode_desc,
        "satellites_used": i_(f[6]), "hdop": f_(f[7]),
        "altitude_msl_m": f_(f[8]), "geoid_separation_m": f_(f[9]),
        "dgps_age_s": f_(f[10]), "dgps_station": f[11] or None,
        "nav_status": lookup(NAV_STATUS, f[12]),
    }


def d_gst(f):
    f = pad(f, 8)
    return {
        "time_utc": utc_time(f[0]), "rms_range_m": f_(f[1]),
        "err_ellipse_major_m": f_(f[2]), "err_ellipse_minor_m": f_(f[3]),
        "err_ellipse_orient_deg": f_(f[4]),
        "lat_err_m": f_(f[5]), "lon_err_m": f_(f[6]), "alt_err_m": f_(f[7]),
    }


def d_hdt(f):
    f = pad(f, 2)
    return {"heading_true_deg": f_(f[0])}


def d_hdg(f):
    f = pad(f, 5)
    dev, var = f_(f[1]), f_(f[3])
    if dev is not None and f[2] == "W":
        dev = -dev
    if var is not None and f[4] == "W":
        var = -var
    return {"heading_mag_deg": f_(f[0]), "deviation_deg": dev, "variation_deg": var}


def d_txt(f):
    f = pad(f, 4)
    return {
        "msg": f"{f[1]}/{f[0]}",
        "severity": {"00": "Error", "01": "Warning", "02": "Notice", "07": "User"}.get(f[2], f[2]),
        "text": ",".join(f[3:]),
    }


def d_vdm(f):
    f = pad(f, 6)
    payload = f[4]
    msg_type = None
    if payload:
        v = ord(payload[0]) - 48
        msg_type = v - 8 if v > 40 else v
    return {
        "fragment": f"{f[1]}/{f[0]}",
        "seq_id": f[2] or None,
        "channel": f[3] or None,
        "ais_msg_type": msg_type,
        "payload": payload,
        "fill_bits": i_(f[5]),
        "note": "AIS payload not decoded (use pyais for full AIS decoding)",
    }


DECODERS = {
    "GGA": d_gga, "RMC": d_rmc, "GSA": d_gsa, "GSV": d_gsv, "VTG": d_vtg,
    "GLL": d_gll, "ZDA": d_zda, "GNS": d_gns, "GST": d_gst, "HDT": d_hdt,
    "HDG": d_hdg, "TXT": d_txt, "VDM": d_vdm, "VDO": d_vdm,
}


def decode(line):
    fr = parse_frame(line)
    if fr is None:
        return None
    fn = DECODERS.get(fr["type"]) if fr["talker"] != "P" else None
    try:
        fr["data"] = fn(fr["fields"]) if fn else {f"field_{n+1}": v for n, v in enumerate(fr["fields"])}
        fr["decoded"] = fn is not None
    except Exception as e:  # malformed sentence shouldn't kill a stream
        fr["data"] = {"error": f"{type(e).__name__}: {e}"}
        fr["decoded"] = False
    return fr


# --------------------------------------------------------------------------- output


def fmt_val(k, v):
    if k == "lat":
        return f"{v:.7f}  ({to_dms(v, True)})"
    if k == "lon":
        return f"{v:.7f}  ({to_dms(v, False)})"
    if isinstance(v, list):
        return ", ".join(str(x) for x in v) if v else "-"
    return str(v)


def print_pretty(fr, out=sys.stdout):
    talker = TALKERS.get(fr["talker"], fr["talker"])
    name = SENTENCE_NAMES.get(fr["type"], "Proprietary" if fr["talker"] == "P" else "Unrecognized")
    cs = {True: "ok", False: f"BAD (got {fr['checksum']}, calc {fr['checksum_calc']})",
          None: "missing"}[fr["checksum_ok"]]
    out.write(f"{fr['raw']}\n")
    out.write(f"  [{fr['talker']}{fr['type']}] {talker} · {name} · checksum {cs}\n")
    data = fr["data"]
    for k, v in data.items():
        if k == "satellites":
            continue
        if v is None or v == "":
            continue
        out.write(f"    {k:<24} {fmt_val(k, v)}\n")
    if data.get("lat") is not None and data.get("lon") is not None:
        out.write(f"    {'map':<24} https://maps.google.com/?q={data['lat']},{data['lon']}\n")
    sats = data.get("satellites")
    if sats:
        out.write("    PRN  Elev  Azim  SNR\n")
        for s in sats:
            row = [s["prn"], s["elev"], s["az"], s["snr"]]
            out.write("    " + "  ".join(f"{'-' if x is None else x:>4}" for x in row) + "\n")
    out.write("\n")


class FixTracker:
    """Merges sentences into one state per epoch (keyed on UTC time)."""

    def __init__(self, out=sys.stdout):
        self.out = out
        self.state = {}
        self.sats = {}      # (talker, prn) -> dict
        self.epoch = None

    def feed(self, fr):
        d = fr["data"]
        t = d.get("time_utc")
        if t and t != self.epoch:
            if self.epoch is not None:
                self.flush()
            self.epoch = t
        typ = fr["type"]
        for k in ("time_utc", "date", "lat", "lon", "altitude_msl_m", "satellites_used",
                  "hdop", "pdop", "vdop", "speed_kn", "speed_kmh", "course_true_deg",
                  "fix_quality", "fix_type", "status", "mode", "heading_true_deg"):
            if d.get(k) is not None:
                self.state[k] = d[k]
        if typ == "GSV":
            if d["msg"].startswith("1/"):
                for key in [k for k in self.sats if k[0] == fr["talker"]]:
                    del self.sats[key]
            for s in d["satellites"]:
                self.sats[(fr["talker"], s["prn"])] = s

    def flush(self):
        s = self.state
        if not s:
            return
        o = self.out
        o.write(f"── {s.get('date', '????-??-??')} {s.get('time_utc', '')} UTC " + "─" * 30 + "\n")
        if s.get("lat") is not None:
            o.write(f"  Position   {s['lat']:.7f}, {s['lon']:.7f}  "
                    f"({to_dms(s['lat'], True)} {to_dms(s['lon'], False)})\n")
        else:
            o.write("  Position   no fix\n")
        parts = []
        if "altitude_msl_m" in s: parts.append(f"alt {s['altitude_msl_m']} m")
        if "speed_kn" in s: parts.append(f"{s['speed_kn']} kn")
        if "course_true_deg" in s: parts.append(f"cog {s['course_true_deg']}°")
        if "heading_true_deg" in s: parts.append(f"hdg {s['heading_true_deg']}°")
        if parts: o.write("  Motion     " + ", ".join(parts) + "\n")
        q = [str(s[k]) for k in ("fix_quality", "fix_type", "status") if k in s]
        if q: o.write("  Fix        " + " | ".join(q) + "\n")
        dop = [f"{k.upper()} {s[k]}" for k in ("pdop", "hdop", "vdop") if k in s]
        if dop: o.write("  DOP        " + "  ".join(dop) + "\n")
        if self.sats:
            by = Counter(TALKERS.get(t, t) for t, _ in self.sats)
            tracked = sum(1 for v in self.sats.values() if v["snr"])
            o.write(f"  Sats       {s.get('satellites_used', '?')} used, {len(self.sats)} in view, "
                    f"{tracked} with signal  ("
                    + ", ".join(f"{k}:{v}" for k, v in sorted(by.items())) + ")\n")
        o.write("\n")
        o.flush()


# --------------------------------------------------------------------------- sources


def open_source(spec):
    if spec == "-":
        yield from sys.stdin
    elif spec.startswith("serial:"):
        try:
            import serial  # pyserial
        except ImportError:
            sys.exit("pyserial required for serial input: pip install pyserial")
        port, _, baud = spec[7:].partition("@")
        with serial.Serial(port, int(baud or 4800), timeout=1) as ser:
            while True:
                raw = ser.readline()
                if raw:
                    yield raw.decode("ascii", errors="replace")
    elif spec.startswith("tcp:"):
        host, _, port = spec[4:].rpartition(":")
        with socket.create_connection((host, int(port))) as sock:
            yield from sock.makefile("r", encoding="ascii", errors="replace")
    elif spec.startswith("udp:"):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("", int(spec[4:])))
        while True:
            data, _ = sock.recvfrom(65535)
            yield from data.decode("ascii", errors="replace").splitlines()
    else:
        with open(spec, "r", encoding="ascii", errors="replace") as fh:
            yield from fh


# --------------------------------------------------------------------------- main


def main():
    ap = argparse.ArgumentParser(description="Decode NMEA 0183 sentences.",
                                 formatter_class=argparse.RawDescriptionHelpFormatter,
                                 epilog=__doc__.split("Sources:")[1])
    ap.add_argument("source", nargs="?", default="-",
                    help="file, '-', serial:PORT@BAUD, tcp:HOST:PORT, udp:PORT")
    ap.add_argument("-s", "--sentence", action="append", help="decode sentence(s) given directly")
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("--json", action="store_true", help="JSON lines output")
    mode.add_argument("--fix", action="store_true", help="consolidated per-epoch fix summary")
    ap.add_argument("--only", help="comma list of sentence types, e.g. GGA,RMC")
    ap.add_argument("--strict", action="store_true", help="drop bad/missing checksums")
    ap.add_argument("--stats", action="store_true", help="print counts at end (stderr)")
    args = ap.parse_args()

    only = {t.strip().upper() for t in args.only.split(",")} if args.only else None
    lines = args.sentence if args.sentence else open_source(args.source)
    tracker = FixTracker() if args.fix else None
    counts, bad_cs, junk = Counter(), 0, 0

    try:
        for line in lines:
            if not line.strip():
                continue
            fr = decode(line)
            if fr is None:
                junk += 1
                continue
            if fr["checksum_ok"] is False:
                bad_cs += 1
            if args.strict and fr["checksum_ok"] is not True:
                continue
            if only and fr["type"] not in only:
                continue
            counts[fr["talker"] + fr["type"]] += 1
            if args.json:
                out = {k: fr[k] for k in ("talker", "type", "checksum_ok", "decoded", "data", "raw")}
                print(json.dumps(out, ensure_ascii=False), flush=True)
            elif tracker:
                tracker.feed(fr)
            else:
                print_pretty(fr)
                sys.stdout.flush()
    except KeyboardInterrupt:
        pass
    except BrokenPipeError:
        return
    finally:
        if tracker:
            tracker.flush()
        if args.stats:
            e = sys.stderr
            e.write(f"\n{sum(counts.values())} sentences, {bad_cs} bad checksums, "
                    f"{junk} non-NMEA lines\n")
            for k, v in counts.most_common():
                e.write(f"  {k:<8} {v}\n")


if __name__ == "__main__":
    main()
