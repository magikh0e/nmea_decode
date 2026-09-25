# nmea_decode

`nmea_decode.py` — a single-file **NMEA 0183** sentence decoder. Standard
library only; `pyserial` is optional and only needed for live serial input.

## What it decodes

`GGA RMC GSA GSV VTG GLL ZDA GNS GST HDT HDG TXT` across every talker (GPS,
GLONASS, Galileo, BeiDou, QZSS, NavIC, multi-GNSS), plus the AIS
`!AIVDM` / `!AIVDO` envelope. Checksums are verified. Anything without a
dedicated decoder is split into raw fields.

## Sources

```
nmea_decode.py log.nmea                 # file
cat log.nmea | nmea_decode.py -         # stdin
nmea_decode.py serial:/dev/ttyUSB0@9600 # serial (pip install pyserial)
nmea_decode.py serial:COM3@4800         # serial on Windows
nmea_decode.py tcp:192.168.1.50:10110   # NMEA over TCP (gpsd raw, gateways)
nmea_decode.py udp:10110                # UDP broadcast (OpenCPN, gateways)
nmea_decode.py -s '$GPGGA,...*47'       # single sentence
```

## Output modes

- *(default)* — pretty per-sentence decode
- `--json` — one JSON object per line
- `--fix` — consolidated fix summary per epoch (position, speed, sats, DOP)

## Options

- `--only GGA,RMC` — filter by sentence type
- `--strict` — drop sentences with bad / missing checksums
- `--stats` — print sentence counts / errors at the end (stderr)

## Examples

```
python nmea_decode.py log.nmea
python nmea_decode.py serial:/dev/ttyUSB0@9600 --fix
python nmea_decode.py udp:10110 --json
```

---

by [magikh0e](https://magikh0e.pl) — also listed at
[magikh0e.pl/pubCode](https://magikh0e.pl/pubCode/)
