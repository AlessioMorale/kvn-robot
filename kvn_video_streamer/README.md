# kvn_video_streamer

On-demand scaled H.264 camera stream as `foxglove_msgs/CompressedVideo` (`format: "h264"`) on
`/video/compressed`, for Lichtblick over the WiFi/VPN link (remote_controller design §4).

Pipeline: `source_pipeline ! videoscale ! videorate drop-only ! videoconvert ! caps(WxH@fps) ! queue ! encoder ! h264parse config-interval=-1 ! video/x-h264,stream-format=byte-stream,alignment=au ! appsink`.

Stream guarantees: Annex B, no B-frames, key frame at most every `keyframe_interval_s` (default 1 s) with SPS/PPS repeated before each IDR, bitrate capped by `bitrate_kbps`.

The pipeline runs only while the topic has subscribers (polled every `poll_period_ms`): it is set to NULL when the count drops to 0, and a GstForceKeyUnit event is sent when a subscriber joins a running stream. On pipeline error it stops and retries after `retry_delay_ms` while subscribers remain.

## WebRTC (optional)

`webrtc.enabled: true` (or `webrtc:=true` on the launch file) adds a second branch to the same pipeline, `webrtcsink` from gst-plugins-rs, so one camera feeds both:

`source ! scale ! rate ! convert ! caps ! tee` -> `queue ! valve ! encoder ! h264parse ! appsink` (Foxglove) and `queue ! webrtcsink` (H.264 forced, congestion control `homegrown`, bitrate between `webrtc.min_bitrate_kbps` and `webrtc.max_bitrate_kbps`).

* The pipeline and the camera stay open so viewers can connect; `webrtcsink` encodes only while one is connected. The Foxglove branch sits behind a valve that opens while the topic has subscribers (key frame requested on opening).
* Needs the `webrtcsink` plugin (`GST_PLUGIN_PATH`) and a signalling server (`gst-webrtc-signalling-server`, see `kvn_robot_bringup/systemd`).
* A WebRTC failure (no signalling server, plugin error) restarts the pipeline without that branch, so Foxglove keeps working; WebRTC is retried every `webrtc.retry_s` seconds while no Foxglove subscriber is connected.
* `webrtc.stun_server` empty keeps the plugin default, a public STUN server; set e.g. `stun://127.0.0.1:3478` to avoid contacting it (ZeroTier gives direct paths).

## Parameters

| Name | Default | Meaning |
|---|---|---|
| `source_pipeline` | `v4l2src device=/dev/video0` | gst-launch fragment producing raw video; `videotestsrc is-live=true` for tests; add `! image/jpeg ! jpegdec` for MJPEG cameras |
| `width`, `height` | 640, 480 | output size (even numbers) |
| `framerate` | 15 | output frame rate (frames dropped, never duplicated) |
| `bitrate_kbps` | 1000 | bitrate cap |
| `keyframe_interval_s` | 1.0 | key frame interval; `key-int-max = round(framerate * this)` |
| `encoder` | `""` | empty: `x264enc tune=zerolatency speed-preset=ultrafast bframes=0`. Otherwise a full encoder fragment (e.g. `v4l2h264enc ...`); placeholders `{bitrate_kbps}`, `{bitrate_bps}`, `{keyint}` are substituted |
| `raw_format` | `I420` | raw format forced before the encoder (`""` = negotiate) |
| `x264_profile` | `baseline` | profile caps after x264enc (`""` = none) |
| `frame_id` | `camera` | `CompressedVideo.frame_id` |
| `topic` | `/video/compressed` | output topic |
| `qos_depth` | 5 | publisher queue depth (reliable) |
| `poll_period_ms` | 200 | subscriber polling period |
| `retry_delay_ms` | 2000 | back-off after a pipeline error |
| `webrtc.enabled` | false | add the WebRTC branch |
| `webrtc.signaller_uri` | `ws://127.0.0.1:8443` | signalling server |
| `webrtc.stun_server` | `""` | `stun://host:port`, empty = plugin default |
| `webrtc.start_bitrate_kbps`, `min_`, `max_` | 1000, 200, 2000 | bitrate range for the congestion control |
| `webrtc.retry_s` | 10 | WebRTC retry interval after a failure |

## Run / test

```bash
ros2 launch kvn_video_streamer video_streamer.launch.py source_pipeline:="videotestsrc is-live=true"
colcon test --packages-select kvn_video_streamer
```

Tests: `test_h264_inspect` and `test_pipeline_config` (pure unit tests), `test_stream` (node with videotestsrc + x264enc: Annex B, no B slices, IDR spacing <= 1 s, SPS/PPS before each IDR, stop with no subscribers, fast key frame for a late subscriber; with `webrtcsink` installed also the valve gating and the fallback when WebRTC fails, skipped otherwise).
Needs GStreamer base/good/ugly (x264enc)/bad (h264parse) plugins.
