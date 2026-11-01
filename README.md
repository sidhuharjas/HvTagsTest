# Hive Tags

AprilTag detection for FTC that works with the camera you already have. One live dashboard for tuning, single tags and clusters, and a desktop simulator so you can try it without a robot. Built and tested by students and teams in the community.

| Camera | How it works | Pick it if | Setup |
|---|---|---|---|
| Limelight 3A | The camera detects tags on its own processor; the robot reads the result | You already run a Limelight and want its pose output | [Limelight](#limelight-3a) |
| USB webcam | Detection runs on the Control Hub through the FTC SDK, no coprocessor | You want to start today with hardware you have | [USB webcam](#usb-webcam) |
| No robot at all | The desktop simulator serves the same dashboard from your laptop | You want to see it work before touching hardware | [Simulator](#desktop-simulator) |

Have both cameras? Start with the Limelight.

## What you get

- Every tag in view reported on its own, with range, bearing, elevation and roll - a hive cell's four tags are four rows, not one
- The best scorable target highlighted, using the alliance and roll rules from the [FTC AprilTag Clusters tech tip](https://ftc-docs.firstinspires.org/en/latest/tech_tips/tech-tips/tech-tip-apriltag-clusters/tech-tip-apriltag-clusters.html)
- A tuning dashboard on port 8080 with the camera's live view on the same page: change a slider, watch the result, nothing redeployed
- **Push tag size to the Limelight**: writes your tag size into the camera's own pipeline over its REST API
- A smoke-test OpMode that prints detections to telemetry and nothing else

## Quick start

Clone the repo, then open `FtcRobotController/` in Android Studio and let Gradle sync finish.

### Limelight 3A

1. Name the Limelight `limelight` on the Driver Station (or type the real name on the dashboard).
2. Run the **AprilTag Dashboard** OpMode.
3. On the dashboard, set **source** to `LIMELIGHT`.

Point the camera at tags. If your Limelight is not at `limelight.local`, type its host or IP in the **host** box and the page and the REST calls both follow it.

Set your tag size once: drag the **tag size** slider, then press **push tag size to Limelight**. The dashboard shows the active pipeline and what the camera answered, for example `pipeline 0: tagSize 152.4 mm - uploaded`. If the pipeline file has no size key the camera understands, nothing is written and the message says what it found instead.

### USB webcam

1. Plug the webcam into the Control Hub and name it `Webcam 1` in the Driver Station.
2. Run the **AprilTag Dashboard** OpMode with **source** set to `WEBCAM`.

If the name is different, type it in the **camera name** box and the stream rebuilds.

### Open the dashboard

The OpMode prints its address on the Driver Station telemetry when it starts. It asks for 8080 and moves to the next free port (8082 and up) if that is taken - on a Control Hub it always is, because the Robot Controller app serves OnBot Java on 8080.

| Situation | URL |
|---|---|
| Laptop on the robot's network | the `Dashboard:` line on telemetry, e.g. `http://192.168.43.1:8082` |
| Control Hub over USB | `adb reverse tcp:8082 tcp:8082` (the port from that line), then open `http://localhost:8082` |

The desktop simulator prints the same line and normally gets 8080.

### Desktop simulator

Windows with a JDK 11 or newer (Android Studio's bundled JDK works). No robot, no Android:

```powershell
cd desktop-sim
powershell -ExecutionPolicy Bypass -File .\run-sim.ps1
```

The dashboard opens at http://localhost:8080 with synthetic targets that move, so every panel has something to show.

To use your laptop webcam as the robot camera instead (needs Python 3.9+ with `opencv-python` and `numpy`):

```powershell
powershell -ExecutionPolicy Bypass -File .\run-sim.ps1 -Webcam
```

Print a tag or hold a phone screen with one up to the lens. Detection runs in `webcam_tags.py`; the preview is at http://localhost:8090 and the numbers on the dashboard are live.

## Dashboard

| Group | What it does |
|---|---|
| source | Switch between the USB webcam and the Limelight while the OpMode runs |
| camera | Live view, webcam resolution, exposure, gain, camera name, Driver Station live view |
| detector | Webcam decimation and pose solver. Hidden in Limelight mode, where the camera owns its own pipeline |
| targeting | Alliance, roll limit, minimum cluster percent, minimum decision margin |
| Limelight | Device name, host, poll rate, active pipeline, link to the camera's own page on port 5801, push tag size |
| hive | Baseline pitch and tip threshold for the hive-cell check |

The page polls 10 times a second. If a row looks stale, the footer of the page shows the loop time and detection count.

The **Live view** card is the webcam itself, served as MJPEG by the OpMode on the dashboard's own port (`/stream.mjpg`), so it works over the robot's wifi and through `adb reverse` alike. Each detected tag is boxed and labelled with its id and range right on the picture. It encodes at most ~30 frames a second, and only while a page is watching. In Limelight mode the card is hidden - that camera has its own preview and settings page on :5801.

## Tests

```powershell
cd FtcRobotController
.\gradlew.bat :TeamCode:testDebugUnitTest
```

56 tests, no robot needed: config clamping and the JSON round trip, the dashboard's HTTP server and live view stream, both camera sources driven through SimLoop with fake hardware, the tag library that keeps every tag of a hive cell separate, and the tag-size patcher against realistic pipeline files.

## Project layout

| Path | What it is |
|---|---|
| `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/` | The detection code; the desktop simulator compiles the SDK-free files (`MjpegFrameGrabber` needs the SDK's camera stack and is skipped there) |
| `FtcRobotController/` | The FTC SDK 12 project: the same sources under `TeamCode/`, plus the tests |
| `desktop-sim/` | Dashboard host for a PC, plus `webcam_tags.py` (OpenCV detector that feeds it) |
| `desktop-sim/run-sim.ps1` | Compiles the sim and starts it |

To use it in your own robot project, copy the seven `.java` files from either `TeamCode/` folder into `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`. They only depend on the FTC SDK.

## Ports

| Port | What |
|---|---|
| 8080, then 8082-8089 | The dashboard. It takes the first free port, so a Control Hub ends up on 8082 or later |
| 8081 | Detections posted by `webcam_tags.py` |
| 8090 | MJPEG preview from the simulator's webcam script (the robot has no extra port: its live view is `/stream.mjpg` on the dashboard's port) |
| 5801 | The Limelight's own web page, linked from the dashboard |
| 5807 | The Limelight's REST API, used by push tag size |

Only one process can own a port, so stop the desktop simulator before running the tests.

## Troubleshooting

- **Page won't open** - the laptop and robot must be on the same network, and use the port from the telemetry line, not always 8080. Over USB the `adb reverse` line has to match that port too.
- **`no scorable target`** - the target card says why: it names the closest tag and the rule that rejected it (alliance, roll, cluster %, decision margin). Set `alliance` to `ANY` to see everything.
- **`BindException: Address already in use`** - every port the dashboard tries is busy. On a Control Hub 8080 is OnBot Java and that is expected, so the OpMode simply takes 8082; trust the telemetry line for the real URL. If it reports that no port was free, close whatever holds 8080-8089.
- **`camera 'Webcam 1' not found`** - the Driver Station camera name is different. Type the right name on the dashboard.
- **Live view says `camera stream not ready yet - retrying`** - the camera is not streaming yet; check the `cam` chip (`STARTING`, `CLOSED`). The page retries every 5 seconds and starts showing video once frames arrive. In Limelight mode there is no video here by design - use the Limelight's own page.
- **The OpMode stopped the moment a tag came into view** - an old bug: a detection the SDK could not localise carries no pose, and the state builder read it anyway. A poseless tag is now served with `"hasPose":false` and shows `no pose` in the Detections table. Update `AprilTagDashboardOpMode.java` if you still see it.
- **The OpMode ends on its own, whatever the cause** - as of this version a loop exception cannot end the run: the OpMode keeps ticking and the `OpMode error` card at the top of the page prints the exception. Paste that text into an issue.
- **Limelight shows nothing** - check the device name matches the Driver Station, and that the Limelight is on a pipeline with tags (`pipe_fiducial`). The dashboard prints the pipeline it is actually running.
- **Push tag size refused** - the message names the keys it found and the value it read. Paste it into an issue and it can usually be supported in one line.
- **Poses jitter** - lower exposure, try another pose solver, or raise decimation while keeping the smallest value that still sees the tags.

## License

The AprilTag code in this repo is MIT, see [LICENSE](LICENSE). The bundled `FtcRobotController/` is FIRST's SDK under its own BSD-3 license, see `FtcRobotController/LICENSE`.
