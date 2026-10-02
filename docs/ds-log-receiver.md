# DS Log Receiver

The Driver Station's own logs (`.dslog`, `.dsevents`: trip time, packet loss, brownouts, the DS-side
event list) live on the DS laptop, and the robot's `.wpilog` lives on the robot. The DS log receiver
puts both in one place: right after a match the DS laptop's **MatchReview** helper pushes that
match's DS files to the robot over the field network, and the pit laptop later pulls them over SFTP
together with the robot logs.

Code: `frc.spectrumLib.telemetry.DSLogReceiver` (settings in its `Constants`), `DSLogHttpServer`
(HTTP), `DSLogFiles` (names, writes, folder cap). Tests: `DSLogFilesTest`, `DSLogHttpServerTest`.

## Why over the field network, and why port 5806

The field stays connected for roughly 30 to 90 s after the robot is disabled at the end of a match.
The field firewall passes only the team ports 5800-5810 to the robot, so the receiver listens on
**5806** (all interfaces). 5800 is WPILib's deploy-folder `WebServer`. Limelight USB port forwarding
(`LimelightHelpers.setupPortForwardingUSB`, not called in this code) would take 5800-5809 and
collide; do not turn it on without moving the receiver.

## Endpoints

|         Request         |                                         Answer                                          |
|-------------------------|-----------------------------------------------------------------------------------------|
| `PUT /dslog/<filename>` | `201 {"name":"...","size":N}` once the file is stored                                   |
| `GET /dslog/`           | `200 [{"name":"...","size":N,"mtime":MS}, ...]`, `mtime` in Unix epoch **milliseconds** |
| `GET /health`           | `200 {"ok":true,"enabled":false,"logDir":"/U/logs"}`                                    |

A `PUT` is checked in this order:

1. `<filename>` is URL-decoded and must match
   `^\d{4}_\d{2}_\d{2} \d{2}_\d{2}_\d{2} [A-Za-z]{3}\.(dslog|dsevents)$`
   (the DS's own naming, e.g. `2026_10_02 14_03_27 Fri.dslog`), else **400**.
2. Header `X-MatchReview-Token` must equal `DSLogReceiver.Constants.TOKEN` (default
   `spectrum-ds-3847`), else **401**. This only keeps stray traffic on the closed field network from
   writing files. It is in this public repo and goes over plain HTTP: it is not security.
3. The robot must be disabled, else **409**. An upload in progress is abandoned (also 409) if the
   robot enables.
4. `Content-Length` is required (**411**; chunked uploads are refused) and must be at most 25 MB
   (**413**).
5. Storing it must leave 200 MB free on the disk, else **507**.

Errors come back as `{"error":"..."}`. The body is streamed to `<logdir>/ds/<filename>.part`, synced
to disk, then renamed over `<logdir>/ds/<filename>`, so a cut-off upload never leaves a
complete-looking file. An existing file of the same name is replaced: MatchReview may push the same
match more than once, and each push is a larger snapshot. The oldest files in `ds/` are deleted to
keep it under 500 MB. A request that takes more than 120 s (the field cut the network mid-upload) is
dropped by the server.

Example (from the DS laptop, robot disabled):

```sh
curl -T "2026_10_02 14_03_27 Fri.dslog" -H "X-MatchReview-Token: spectrum-ds-3847" \
  "http://10.38.47.2:5806/dslog/2026_10_02%2014_03_27%20Fri.dslog"
curl http://10.38.47.2:5806/health
```

## Where the files go

`<logdir>` is `/U/logs` if a USB stick is in and writable, else `/home/systemcore/logs` (the same choice
DogLog makes for the `.wpilog`). It is decided per upload, so a stick that mounts late is still used.
`/health` reports the folder in use.

## Pulling them in the pit

The files sit in `<logdir>/ds/` next to the wpilogs. Copy that folder along with the robot logs, from
both places if the stick was in and out during the day:

```sh
scp -r "lvuser@10.38.47.2:/U/logs/ds" ./rio-logs/ds-usb
scp -r "lvuser@10.38.47.2:/home/systemcore/logs/ds" ./rio-logs/ds
```

Any SFTP client (WinSCP, FileZilla) works the same way. Files are only deleted by the 500 MB cap, so
clean the folder up after an event if you want the space back.

## Logging

What the receiver did is logged from the main loop (the server thread never calls DogLog):

|                         Key                         |                          Meaning                           |
|-----------------------------------------------------|------------------------------------------------------------|
| `DSLogReceiver/Running`                             | true once the port is bound; false if it failed to start   |
| `DSLogReceiver/LastFile`, `DSLogReceiver/LastSize`  | the last file stored                                       |
| `DSLogReceiver/Count`                               | files stored since boot                                    |
| `DSLogReceiver/Rejected`, `DSLogReceiver/LastError` | refused or failed uploads, and why                         |
| `DSLogReceiver/Dropped`                             | results lost because 64 were waiting (a flood of requests) |

Each result is also printed to the console.

## Robot safety

* Started once from `Robot.robotInit()`, on the real robot only (not in simulation).
* Its own low-priority daemon threads: a JDK `HttpServer` with a 1-thread executor (the JDK adds its
  dispatcher and two idle timer threads). It never calls WPILib or HAL; the main loop writes a
  volatile `enabled` flag each loop (`DriverStation.isEnabled()`) and the server reads it. The flag
  starts as "enabled", so nothing is accepted until the first loop has run.
* Every exception is caught. If the JVM lacks the `jdk.httpserver` module or the port is taken, the
  receiver logs the failure and the robot runs without it.
* Turn it off with `DSLogReceiver.Constants.ENABLED = false`.
