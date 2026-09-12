# xgc2-ros-image-rtp-adapter

Hardware-neutral ROS 1/ROS 2 image ingress for
[`xgc2-media-edge`](https://github.com/lxk36/xgc2-media-edge). It converts an
explicit `sensor_msgs/Image` or JPEG `sensor_msgs/CompressedImage` topic into
the same local source contract used by every other XGC2 camera source:

- H264/RTP payload type 96 on a fixed loopback UDP port;
- newline-delimited source control on an absolute Unix socket;
- stable `sourceId`, dimensions, rate, and optical frame metadata;
- independent `set-active`, keyframe, snapshot, and fresh-snapshot operations.

Topic names, ROS version, message type, pixel format, encoder backend, element
factories, and element properties are configuration. There is no Odin, B2,
camera brand, device path, Jetson model, or topic-name branch in the runtime.

## Unified media boundary

This package is one replaceable ingress, not the browser server:

```text
ROS 1 Image/CompressedImage ─┐
ROS 2 Image/CompressedImage ─┤  this adapter -> H264/RTP + Unix control
direct camera H264/RTSP ─────┤  native repacketizer/source
Gazebo world camera ─────────┘  native NVENC H264/RTP source
                                      |
                                      v
                            one xgc-media-edge service
                                      |
                                      v
                          WebRTC -> Experiment camera panel
```

The Gazebo world camera already implements the Edge source contract directly,
so its encoded output must not be routed through ROS and re-encoded. A direct
robot camera that already emits H264 should likewise use a bounded native
repacketizer. Raw/JPEG camera feeds and ROS topics use this encoder adapter.
All of them are configured as named sources in one Edge source roster and are
controlled independently by the Experiment Session.

## Shared ROS runtime

ROS 1 Noetic and ROS 2 Humble/Jazzy use the same Python modules for frame
validation, latest-frame buffering, encoder supervision, snapshots, and the
Edge control socket. The wrappers contain only the ROS-specific subscription,
parameters, logging, and timers.

| Input mode | ROS message | Encoder input |
| --- | --- | --- |
| `compressed` | `sensor_msgs/CompressedImage` | complete JPEG bytes |
| `raw` | `sensor_msgs/Image` | packed `rgb8`, `bgr8`, `rgba8`, `bgra8`, or `mono8` bytes |

Raw ROS row padding is removed once and the packed frame enters FFmpeg or
GStreamer directly. It is never converted to JPEG before H264 encoding.

For `compressed` input, the adapter also retains exactly one latest source JPEG
for snapshot transactions. `requireFresh=true` records the frame sequence when
the request arrives and blocks until a later message replaces it; the request
therefore never reuses a pre-request frame. JPEG-only snapshots return those
exact camera bytes as `source-jpeg-passthrough` and do not invoke Pillow or a
second JPEG encoder. Raw input retains the compatible Pillow/libjpeg snapshot
path only when that source mode is explicitly selected.
Dimensions and raw encoding are explicit, fixed Session configuration; a
mismatched message is rejected instead of silently changing the stream
contract. Each received frame is encoded at most once, so a stopped ROS source
also stops RTP rather than replaying the final frame forever.

The adapter starts inactive. Backend capabilities are preflighted during
readiness, but the encoder process and any hardware encoder session are not
allocated until Edge sends `set-active=true`. When the last viewer/recording
releases the source, `set-active=false` clears pending frames and terminates the
encoder. The ROS publisher may remain shared with other consumers without
holding an idle NVENC session for this video path.

## Encoder backends

| Backend | Default | Intended use |
| --- | --- | --- |
| `ffmpeg` | yes (`libx264`) | Portable CPU baseline |
| `gstreamer` | opt-in | Configurable software or hardware pipeline |

There is deliberately no `auto` backend. A deployment selects a reviewed
capability profile. FFmpeg validates the configured encoder; GStreamer checks
every selected element factory and every configured property with
`gst-inspect-1.0` before the control socket becomes ready.

`config/jetson_nvmm_gstreamer.yaml` is the single NVIDIA capability profile for
Jetson AGX Orin and Jetson Thor. It requests
`nvjpegdec -> nvvidconv -> nvv4l2h264enc` for JPEG input, or
`rawvideoparse -> nvvidconv -> nvv4l2h264enc` for raw input. The runtime does
not inspect a board model; deployments select this capability explicitly.

NVIDIA documents these accelerated GStreamer elements for
[Jetson Linux r36.4.4 (Orin)](https://docs.nvidia.com/jetson/archives/r36.4.4/DeveloperGuide/SD/Multimedia/AcceleratedGstreamer.html)
and
[Jetson Linux r38.4 (Thor)](https://docs.nvidia.com/jetson/archives/r38.4/DeveloperGuide/SD/Multimedia/AcceleratedGstreamer.html).
Thor deployments should use GStreamer for NVIDIA hardware encode; the
[r38.4 release notes](https://docs.nvidia.com/jetson/archives/r38.4/ReleaseNotes/Jetson_Linux_Release_Notes_r38.4.pdf)
state that NVIDIA FFmpeg hardware acceleration is not supported there.

## Install

```bash
# ROS 1 Noetic / Ubuntu 20.04
sudo apt install ros-noetic-xgc2-ros-image-rtp-adapter

# ROS 2 Humble / Ubuntu 22.04
sudo apt install ros-humble-xgc2-ros-image-rtp-adapter

# ROS 2 Jazzy / Ubuntu 24.04 (Jetson Thor baseline)
sudo apt install ros-jazzy-xgc2-ros-image-rtp-adapter
```

Packages include the portable FFmpeg/GStreamer dependencies. NVIDIA plugins
and GPU device access belong to the Agent container/runtime image.

## Run

ROS 2 JPEG topic:

```bash
source /opt/ros/jazzy/setup.bash
ros2 launch ros_image_rtp_adapter image_rtp_adapter.launch.py \
  image_topic:=/camera/front/image/compressed \
  input_message_type:=compressed \
  source_id:=front \
  rtp_port:=5004 \
  control_socket:=/tmp/xgc2/media/front.sock \
  width:=1280 height:=720 fps:=15.0
```

ROS 2 raw topic with the Jetson NVMM profile:

```bash
profile="$(ros2 pkg prefix ros_image_rtp_adapter)/share/ros_image_rtp_adapter/config/jetson_nvmm_gstreamer.yaml"
ros2 run ros_image_rtp_adapter image_rtp_adapter --ros-args \
  --params-file "${profile}" \
  -p image_topic:=/camera/front/image_raw \
  -p input_message_type:=raw \
  -p raw_encoding:=bgr8 \
  -p source_id:=front \
  -p rtp_port:=5004 \
  -p control_socket:=/tmp/xgc2/media/front.sock \
  -p width:=1280 -p height:=720 -p fps:=30.0
```

ROS 1 Noetic:

```bash
source /opt/ros/noetic/setup.bash
roslaunch ros_image_rtp_adapter image_rtp_adapter.launch \
  image_topic:=/camera/front/image/compressed \
  input_message_type:=compressed \
  source_id:=front \
  rtp_port:=5004 \
  control_socket:=/tmp/xgc2/media/front.sock
```

For ROS 1 on a validated Jetson container, select the ROS 1 form of the same
capability profile and keep the source parameters independent:

```bash
profile="$(rospack find ros_image_rtp_adapter)/config/jetson_nvmm_gstreamer.yaml"
roslaunch ros_image_rtp_adapter image_rtp_adapter.launch \
  config:="${profile}" \
  image_topic:=/camera/front/image_raw \
  input_message_type:=raw raw_encoding:=bgr8 \
  source_id:=front rtp_port:=5004 \
  control_socket:=/tmp/xgc2/media/front.sock \
  width:=1280 height:=720 fps:=30.0
```

Pair one or more adapters/native sources with one Edge process:

```json
{
  "sources": [
    {
      "id": "front",
      "rtpListenAddress": "127.0.0.1:5004",
      "controlSocket": "/tmp/xgc2/media/front.sock"
    },
    {
      "id": "world",
      "rtpListenAddress": "127.0.0.1:5006",
      "controlSocket": "/tmp/xgc2/media/world.sock"
    }
  ]
}
```

```bash
xgc-media-edge \
  --control-address 0.0.0.0:18090 \
  --sources-config /run/xgc2/media/sources.json
```

Production ownership is:

```text
Experiment Run freezes source roster
  -> Session starts/supervises ingress sources and Media Edge
  -> browser panel creates/deletes source-scoped viewers
  -> Session stop tears the products down
```

Opening a browser must never create a source process, and video never enters
AgentLink, Core, SSE, or the robot telemetry plane.

## Parameters

| Parameter | Default | Description |
| --- | --- | --- |
| `image_topic` | `/camera/image_raw/compressed` | Fully parameterized input topic |
| `input_message_type` | `compressed` | Explicit `compressed` or `raw` subscription |
| `raw_encoding` | `bgr8` | Required packed encoding for raw input |
| `source_id` | `camera` | Stable Edge source ID |
| `frame_id` | `camera_optical` | Optical frame in describe/snapshot |
| `rtp_host` / `rtp_port` | `127.0.0.1` / `5004` | Fixed loopback RTP destination |
| `control_socket` | `/tmp/xgc2-image-rtp-adapter.sock` | Absolute Unix control socket |
| `width` / `height` / `fps` | 1280 / 720 / 15 | Fixed output and raw-input contract |
| `bitrate` | 2500000 | Target H264 bitrate |
| `encoder_backend` | `ffmpeg` | Explicit `ffmpeg` or `gstreamer` |
| `encoder` | `libx264` | FFmpeg encoder factory |
| `ffmpeg_encoder_args_json` | `[]` | Structured FFmpeg argument array |
| `ffmpeg_video_filter` | generated scale/pad | Optional FFmpeg filter expression |
| `gstreamer_*` | portable software pipeline | Element, caps, and property profile |
| `drop_to_latest` | `true` | One-frame queue; `false` selects bounded depth 32 |
| `require_jpeg` | `true` | Validate complete JPEG compressed frames |

Runtime markers are `@bitrate`, `@bitrate_kbps`, `@fps`, `@gop`, `@width`,
`@height`, and, for caps, `@fps_fraction`.

The built-in `h264_nvenc` path is not the unconstrained FFmpeg default. It uses
4:2:0 High profile, low-latency CBR, `maxrate == bitrate`, a one-second VBV
(`bufsize == bitrate`), fixed one-second GOP, no B-frames, no lookahead, no
scene-cut I-frames, and strict GOP rate control. This makes motion-induced RTP
bursts bounded enough for the paired Media Edge receive queue to be calculated.
Do not validate this path with a static camera alone: acceptance must include
high-entropy motion and prove source FPS, adapter output/drop counters, kernel
socket drops, Edge `framesInError`, and browser decoded/presented FPS separately.

## Gates

The CI and release matrices cover:

| ROS | Ubuntu | Architectures |
| --- | --- | --- |
| Noetic | Focal | amd64, arm64 |
| Humble | Jammy | amd64, arm64 |
| Jazzy | Noble | amd64, arm64 |

Every cell builds a real DEB, installs it in its matching ROS/Ubuntu image,
runs the shared unit suite, proves JPEG and raw GStreamer RTP emission, and
runs publisher -> adapter -> Media Edge contract integration from the installed
`/opt/ros/<distro>` package in a workspace-free environment. Push CI uses the
hash-pinned Media Edge source; release-train compatibility jobs install the
signed staging APT candidate and never fall back to that source lock. Builds
use the versioned XGC2 ROS build images and their preinstalled Go toolchain;
product CI does not install distro packages or download a toolchain. Package
staging uses an exact file manifest, rejects caches or foreign files, and builds
the DEB twice under one `SOURCE_DATE_EPOCH` to prove deterministic output.

Local focused gates:

```bash
PYTHONPATH=. python3 -m pytest \
  test/test_artifact_manifest.py \
  test/test_control_socket.py test/test_encoder.py \
  test/test_frames.py test/test_media_edge_source_roster.py \
  test/test_runtime.py -q
./.xgc2/scripts/check_package_compliance.sh

# Full container package/integration gate
dependency_set_digest="$(python3 .xgc2/scripts/read_integration_lock.py \
  --lock .xgc2/integration-lock.json --field dependencySetDigest)"
./.xgc2/scripts/build_debs_in_docker.sh \
  --ros-distro jazzy --ubuntu noble \
  --prepare-action ci \
  --dependency-mode locked-source \
  --dependency-set-digest "${dependency_set_digest}"
```

## License

Apache-2.0


### ROS 1 H264 preview shared with RTP

An explicit `~video_topic` enables `foxglove_msgs/CompressedVideo` output from
the same FFmpeg encoding operation used for RTP. The original ROS Image or
CompressedImage timestamp follows the retained frame through the bounded input
queue. No second JPEG decode/encode pass is introduced for the viewer. ROS
subscribers and Media Edge independently hold the encoder active; closing one
consumer does not stop the other. This output currently requires FFmpeg and
zero B-frames. JPEG snapshots remain source-resolution passthrough.

The Annex-B stream contains AUD boundaries and repeated parameter sets for
late joins at the next IDR. Its bounded framing reader emits a frame when the
next AUD arrives (one frame of framing delay); it does not label decode time as
capture time. Missing source timestamps are rejected. GStreamer keeps its
existing RTP-only path until an equivalent encoded-packet tap is implemented.

For JPEG input with NVENC, two bounded encoder output-delay frames allow CPU JPEG
decode to overlap GPU encoding. Raw input keeps zero output delay. This buffering
is separate from the drop-to-latest source queue; it does not enable B-frames or
an unbounded display backlog. Hardware frame-identity and late-join tests can be
run with `XGC2_TEST_NVENC=1 python3 -m pytest -q test/test_h264_integration.py`.
