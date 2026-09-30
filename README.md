# Buddy 1 Rover

## Build

From the repository root:

```powershell
cmake -S . -B build "-DMQTT_BROKER_IP=192.168.137.1"
cmake --build build --target EmbeddedProj
```

Flash `build/rover_project/rover_project.uf2` to the Pico W. Wi-Fi credentials are supplied as CMake variables; do not commit real credentials to a shared repository.

## Local MQTT broker

Start Mosquitto with:

```powershell
& "C:\Program Files\Mosquitto\mosquitto.exe" `
  -c "$PWD\rover_project\mosquitto-local.conf" -v
```

The local configuration listens on:

- MQTT TCP: `192.168.137.1:1883`
- MQTT WebSockets: `192.168.137.1:9001`

For a private development hotspot it allows anonymous access. Use authentication and TLS before connecting the rover to an untrusted network.

## Local dashboard

In another terminal:

```powershell
python -m http.server 8000 --directory rover_project/dashboard
```

Open <http://localhost:8000>. The dashboard defaults to the local hotspot broker. To use a different broker, open `http://localhost:8000/?broker=HOST_OR_IP`.

## Topics

- `rover/commands`: JSON commands such as `{"cmd":"stop"}` or `{"cmd":"drive","left":30000,"right":30000}`
- Motion commands: `{"cmd":"move_forward","distance_mm":500,"max_pwm":20000}`, `move_backward`, `turn_left`, and `turn_right`
- `rover/telemetry`: sensor, encoder, motor, heading, and uptime data
- `rover/debug`: Buddy 3 raw/normalized IR values, line position, junction, and barcode state
- `rover/heartbeat`: periodic online heartbeat
- `rover/status`: retained offline Last Will

The firmware stops active motors after one second without a fresh command.

## Buddy 2 motion APIs

The reusable motion library is in `buddy2_motion.h` and `buddy2_motion.c`. It provides signed motor control, encoder odometry, speed estimation, PID correction, distance moves, and encoder-based turns. The default geometry and encoder values are placeholders; complete the measurements in `BUDDY2_CALIBRATION.md` before relying on physical distance or angle accuracy.
