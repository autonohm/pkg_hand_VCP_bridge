# PIB Hand Nucleo Bridge

Dieses ROS2-Paket verbindet die ROS-Schnittstelle mit dem VCP-Serial-Protokoll
der STM32-Nucleo-Handsteuerung. Die Bridge sendet ASCII-Kommandos ueber den
seriellen Port und wartet auf `OK` oder `ERR ...`.

Die Firmware ist fuer die eigentliche Trajektorie und Servo-Ansteuerung
verantwortlich. Eine erfolgreiche ROS-Antwort bedeutet daher nur, dass die
Firmware das Kommando angenommen hat. Sie bedeutet nicht, dass die Bewegung
bereits mechanisch abgeschlossen ist.

## ROS-Schnittstelle

Der Node laeuft unter dem Namespace `/hand/nucleo`:

| Service | Typ | VCP-Befehl |
|---|---|---|
| `/hand/nucleo/grip` | `datatypes/srv/NucleoGrip` | `G` |
| `/hand/nucleo/finger` | `datatypes/srv/NucleoFinger` | `F` |
| `/hand/nucleo/stop` | `datatypes/srv/NucleoSideCommand` | `STOP` |
| `/hand/nucleo/hold` | `datatypes/srv/NucleoSideCommand` | `HOLD` |
| `/hand/nucleo/status` | `datatypes/srv/NucleoStatus` | `GET:STATUS` |

Handseiten:

- `side: 0` = linke Hand
- `side: 1` = rechte Hand

Grip-IDs:

| ID | Griff |
|---:|---|
| `0` | offene Hand |
| `1` | Spitzgriff |
| `2` | Dreipunktgriff |
| `3` | Schluesselgriff |
| `4` | Zylindergriff |
| `5` | Hakengriff |
| `6` | sphaerischer Griff |
| `7` | Mittelfinger |

## Beispiele

Standardgriff auf der rechten Hand:

```bash
ros2 service call /hand/nucleo/grip datatypes/srv/NucleoGrip \
  "{side: 1, grip_id: 4, speed_mode: 0, speed_percent: 0, axis_speeds: []}"
```

Griff mit globalem Geschwindigkeitsfaktor von 50 Prozent:

```bash
ros2 service call /hand/nucleo/grip datatypes/srv/NucleoGrip \
  "{side: 1, grip_id: 4, speed_mode: 1, speed_percent: 50, axis_speeds: []}"
```

Griff mit individueller Geschwindigkeit fuer sechs Achsen:

```bash
ros2 service call /hand/nucleo/grip datatypes/srv/NucleoGrip \
  "{side: 1, grip_id: 4, speed_mode: 2, speed_percent: 0, axis_speeds: [100, 80, 60, 60, 80, 50]}"
```

Einzelne Achse auf Position 3000 bewegen:

```bash
ros2 service call /hand/nucleo/finger datatypes/srv/NucleoFinger \
  "{side: 1, finger: 1, position: 3000, use_speed: false, speed: 0}"
```

Einzelne Achse mit eigener Geschwindigkeit bewegen:

```bash
ros2 service call /hand/nucleo/finger datatypes/srv/NucleoFinger \
  "{side: 1, finger: 2, position: 4095, use_speed: true, speed: 100}"
```

Stopp, Halten und Status:

```bash
ros2 service call /hand/nucleo/stop datatypes/srv/NucleoSideCommand "{side: 1}"
ros2 service call /hand/nucleo/hold datatypes/srv/NucleoSideCommand "{side: 1}"
ros2 service call /hand/nucleo/status datatypes/srv/NucleoStatus "{}"
```

Bei `NucleoStatus` stehen die Firmware-Statuszeilen in `status_lines`. Die
vollstaendige Antwort befindet sich zusaetzlich in `raw_response`.

## Serielle Verbindung

Die Bridge verwendet die im VCP-Protokoll festgelegten Werte:

```text
460800 Baud, 8 Datenbits, keine Paritaet, 1 Stoppbit, kein Flow-Control
```

Standard-Port ist `/dev/ttyACM0`. Der Port kann ueber die Umgebungsvariable
`NUCLEO_SERIAL_PORT` geaendert werden:

```bash
NUCLEO_SERIAL_PORT=/dev/ttyACM1 ros2 launch hand_nucleo launch.py
```

Weitere ROS-Parameter:

| Parameter | Standard | Bedeutung |
|---|---:|---|
| `serial_port` | `/dev/ttyACM0` | Serieller Port |
| `baudrate` | `460800` | Baudrate |
| `response_timeout` | `1.0` | Antwort-Timeout in Sekunden |
| `reconnect_interval` | `1.0` | Reconnect-Intervall in Sekunden |
| `retry_delay` | `0.05` | Pause vor einem `ERR BUF`-Retry |

Zugriffe werden serialisiert, damit sich Antworten mehrerer ROS-Aufrufe nicht
vermischen. `ERR BUF` wird genau einmal wiederholt. Nach einem Timeout wird ein
Bewegungskommando nicht automatisch wiederholt, weil nicht sicher festgestellt
werden kann, ob die Firmware es bereits ausgefuehrt hat.

## Docker und Build

Das Paket wird im bestehenden `ros-motors`-Image gebaut und beim Start des
Motors-Launches automatisch neben den bisherigen Motor-Nodes gestartet.

```bash
docker compose --profile motors build ros-motors
docker compose --profile motors up ros-motors
```

Der Compose-Service reicht standardmaessig `/dev/ttyACM0` in den Container
durch. Bei einem anderen Port muessen `NUCLEO_SERIAL_PORT` und das Device
Mapping entsprechend angepasst werden.

## Tests

Die Protokoll- und Transporttests koennen im gebauten Image ausgefuehrt werden:

```bash
docker run --rm ros_motors \
  python3 -m pytest /app/ros2_ws/hand_nucleo/test/test_protocol.py
```

Die Tests verwenden einen Fake-Serial-Port und benoetigen keine angeschlossene
Nucleo.
