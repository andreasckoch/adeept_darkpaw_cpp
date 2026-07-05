# Spider Viewer

This is a Mac-side desktop viewer prototype for the Darkpaw telemetry stream.
It uses only Python standard-library modules and a browser window:

- `spider_viewer_server.py` receives `SPIDER_SENSOR_V1` UDP packets.
- The browser UI consumes those packets through server-sent events.
- The center panel shows a camera area or a configured video URL.
- The bottom panel renders live telemetry time-series.
- The lower-right overlay shows teleop controls.

Run from the repository root:

```bash
python3 desktop/spider_viewer/spider_viewer_server.py --open
```

Then stream mock telemetry:

```bash
./build-host/spider_sensor_telemetry_mock --host 127.0.0.1
```

On the Pi, stream available telemetry to the Mac address:

```bash
./build-pi/spider_sensor_streamer --host <mac-ip>
```

Camera video transport is intentionally separate from telemetry. For now the
viewer can show an HTTP-accessible video/image URL:

```bash
python3 desktop/spider_viewer/spider_viewer_server.py --open --video-url http://127.0.0.1:8080/video
```

The next iteration should replace that placeholder path with GStreamer/WebRTC or
another browser-displayable low-latency stream.
