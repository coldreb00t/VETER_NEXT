# Camera Streaming Latency Analysis

## Current Status: ✅ Working with Known Trade-offs

**Achievement:** VETER robot now has **global vision** - camera streaming works from anywhere in the world!

**Current performance:**
- ✅ Initial latency: ~100-200ms (acceptable)
- ⚠️ Latency growth: Increases over time (2-5 minutes)
- ✅ Video quality: Good, no artifacts
- ✅ Reliability: Stable connection
- ✅ Global access: Works through any NAT/firewall

## Architecture

```
┌────────┐  UDP/RTP   ┌─────────┐  RTSP/TCP  ┌────────┐
│ Jetson │ ────────> │   VPS   │ ─────────> │ Client │
│ Russia │  H.264    │ Germany │ interleaved│ Russia │
└────────┘  2 Mbps   └─────────┘            └────────┘
              │            │                     │
           ~20-30ms    ~50-80ms             ~50-80ms
```

**Total base latency:** ~100-200ms
**Latency after 5 min:** ~300-500ms (grows)

## Known Issues

### Issue 1: Growing Latency (Buffer Bloat)

**Problem:** Latency increases over time (not constant)

**Root cause:**
1. **Network jitter** - Variable delays in internet routing
2. **Buffer accumulation** - Receivers buffer packets to smooth jitter
3. **No feedback loop** - Sender doesn't know about receiver buffer state
4. **RTSP over TCP** - TCP retransmits lost packets, adding delay

**Example:**
```
Time 0s:   Buffer = 0ms    → Latency 150ms ✅
Time 60s:  Buffer = 100ms  → Latency 250ms ⚠️
Time 300s: Buffer = 300ms  → Latency 450ms ❌
```

**Why it happens:**
- Packet arrives late → Receiver buffers it
- Next packet arrives on time → Buffer grows
- TCP waits for lost packets → More buffering
- No mechanism to flush old data

**Impact on robot control:**
- First minute: Acceptable for driving
- After 5 minutes: Too slow to react to obstacles
- Critical for autonomous navigation

### Issue 2: UDP Through Internet

**Challenge:** Pure UDP doesn't work client → server through NAT

**Why:**
- Client behind NAT (router blocks incoming UDP)
- VPS can't send UDP packets to client
- Firewall drops unsolicited UDP

**Current solution:** RTSP TCP interleaved
- RTP data embedded in TCP connection
- Works through any NAT/firewall
- But: TCP adds latency and buffer bloat

### Issue 3: Geographic Distance

**Current path:**
```
Robot (Russia) → VPS (Germany) → Client (Russia)
    100ms ping           100ms ping
```

**Problem:** Double the latency
- Data goes to Germany and back
- Each hop adds 50-100ms
- Unavoidable with current VPS location

## Solutions Comparison

### Quick Fixes (Implement Now: 1-5 minutes)

#### Option 1: Aggressive Client Buffering
```bash
# GStreamer with minimal buffering
gst-launch-1.0 -v \
  rtspsrc location=rtsp://81.200.157.230:8555/camera \
    latency=50 drop-on-latency=true ! \
  rtph264depay ! h264parse ! \
  avdec_h264 max-threads=1 ! \
  videoconvert ! \
  autovideosink sync=false
```

**Parameters:**
- `latency=50` - Only 50ms buffer (vs default 2000ms)
- `drop-on-latency=true` - Drop frames if buffer exceeds limit
- `sync=false` - Don't sync to clock, play ASAP
- `max-threads=1` - Minimal decode latency

**Pros:**
- ✅ Constant 50-100ms latency
- ✅ Immediate (no setup needed)
- ✅ Good for manual control

**Cons:**
- ❌ Frame drops on packet loss
- ❌ Possible stuttering
- ❌ Artifacts on bad network

**Use case:** Manual driving in good conditions

---

#### Option 2: FFplay Low-Latency Mode
```bash
ffplay -probesize 32 -analyzeduration 0 \
  -sync ext -fflags nobuffer -flags low_delay \
  -framedrop rtsp://81.200.157.230:8555/camera
```

**Parameters:**
- `-probesize 32` - Minimal stream analysis
- `-analyzeduration 0` - Start playing immediately
- `-sync ext` - External clock sync
- `-fflags nobuffer` - No buffering
- `-flags low_delay` - Low-latency decoder
- `-framedrop` - Drop frames to maintain latency

**Pros:**
- ✅ Lowest latency possible (~80-150ms)
- ✅ Simple command
- ✅ Aggressive frame dropping

**Cons:**
- ❌ Choppy playback
- ❌ Not suitable for recording
- ❌ May skip important frames

**Use case:** Quick reaction driving (teleoperation)

---

#### Option 3: VLC Minimal Caching
```bash
vlc --network-caching=50 --live-caching=50 \
  rtsp://81.200.157.230:8555/camera
```

**Parameters:**
- `--network-caching=50` - 50ms network buffer
- `--live-caching=50` - 50ms playback buffer

**Pros:**
- ✅ Easy to use
- ✅ ~100-150ms constant latency
- ✅ Better than default (300ms+)

**Cons:**
- ❌ Still has some buffering
- ❌ Not as aggressive as FFplay

**Use case:** Balanced approach for most scenarios

---

### Medium Solutions (Implement: 1-2 hours)

#### Option 4: WebRTC Streaming

**Architecture:**
```
Jetson → WebRTC → Janus (VPS) → WebRTC → Browser/GStreamer
```

**Why WebRTC:**
- **Adaptive bitrate** - Adjusts to network conditions
- **FEC (Forward Error Correction)** - Recovers from packet loss without retransmit
- **Congestion control** - Built-in feedback loop
- **NAT traversal** - Works through any firewall
- **Constant latency** - Designed for real-time (100-200ms stable)

**Implementation:**

1. **Install Janus on VPS:**
```bash
# On VPS
apt-get install janus
systemctl start janus
```

2. **Configure Janus streaming plugin:**
```yaml
# /etc/janus/janus.plugin.streaming.jcfg
camera: {
    type = "rtp"
    description = "VETER Robot Camera"
    audio = false
    video = true
    videoport = 9000
    videopt = 96
    videortpmap = "H264/90000"
}
```

3. **Jetson publisher (unchanged):**
```bash
# Same UDP/RTP as now
./scripts/start_camera_mediamtx.sh
```

4. **View in browser:**
```html
<script src="https://81.200.157.230:8088/janus.js"></script>
<video id="camera" autoplay playsinline></video>
```

**Or GStreamer client:**
```bash
gst-launch-1.0 webrtcsrc signaller::uri="ws://..." ! \
  rtph264depay ! h264parse ! avdec_h264 ! autovideosink
```

**Pros:**
- ✅✅✅ **Constant latency** (100-200ms, doesn't grow!)
- ✅ Adaptive bitrate (handles bad network)
- ✅ FEC recovers lost packets
- ✅ Industry standard for telepresence
- ✅ Can view in browser (no app needed)

**Cons:**
- ⚠️ Setup complexity (need Janus server)
- ⚠️ 2 hours to implement
- ⚠️ More CPU usage on VPS

**Performance:**
- Latency: 100-200ms **constant**
- Packet loss tolerance: Up to 10-15%
- Bitrate: Adaptive 500-3000 kbps

**Use case:** **RECOMMENDED for robot control** - Production-grade solution

**Time to implement:** 2 hours
**Status:** Best long-term solution

---

#### Option 5: VPS in Russia

**Problem:** Current path doubles latency
```
Russia → Germany → Russia = 200ms
```

**Solution:** Russian VPS (Moscow/SPb)
```
Russia → Moscow → Russia = 50-100ms
```

**Providers:**
- Selectel (Moscow) - ~20ms ping
- Yandex Cloud (Moscow) - ~30ms ping
- VK Cloud (SPb) - ~40ms ping

**Migration steps:**

1. **Rent Russian VPS** (2 vCPU, 2GB RAM, ~500 RUB/month)

2. **Copy configuration:**
```bash
# Backup current VPS config
ssh root@81.200.157.230 "tar czf /tmp/mediamtx-config.tar.gz /etc/systemd/system/*mediamtx* /etc/mediamtx.yml /root/camera.sdp"
scp root@81.200.157.230:/tmp/mediamtx-config.tar.gz ./

# Deploy to new VPS
scp mediamtx-config.tar.gz root@<NEW_IP>:/tmp/
ssh root@<NEW_IP> "cd / && tar xzf /tmp/mediamtx-config.tar.gz"
```

3. **Install MediaMTX + FFmpeg** (same as current)

4. **Update Jetson script:**
```bash
# Change VPS_HOST in start_camera_mediamtx.sh
VPS_HOST="<NEW_RUSSIAN_IP>"
```

5. **Test and switch DNS**

**Pros:**
- ✅ **50% latency reduction** (100ms → 50ms base)
- ✅ Lower ping to both Jetson and client
- ✅ Same setup, just different location
- ✅ Russian jurisdiction (if matters)

**Cons:**
- ⚠️ Need to rent new VPS (~500 RUB/month)
- ⚠️ 1 hour migration time
- ⚠️ Two servers during migration

**Performance:**
- Base latency: 50-100ms (vs 100-200ms now)
- Still has buffer bloat issue
- But starts from lower baseline

**Use case:** Easy improvement, keeps current architecture

**Time to implement:** 1 hour
**Cost:** ~500 RUB/month

---

#### Option 6: Direct UDP (If Starlink/4G with White IP)

**Architecture:**
```
Jetson (White IP) ←─UDP──→ Client
         No VPS needed!
```

**Requirements:**
- ✅ Starlink OR 4G with white IP
- ✅ Port forwarding on Jetson's router
- ❌ Won't work with CGNAT (most 4G providers)

**Check if you have white IP:**
```bash
# On Jetson
curl ifconfig.me
# Compare with your router's WAN IP
# If same → White IP ✅
# If different → CGNAT ❌
```

**If you have white IP:**

1. **Configure router port forwarding:**
```
External port 9000 → Jetson IP:9000 (UDP)
```

2. **Start direct UDP stream:**
```bash
# New script: start_camera_direct.sh
gst-launch-1.0 \
  nvarguscamerasrc ! ... ! x264enc ! rtph264pay ! \
  udpsink host=<CLIENT_IP> port=9000
```

3. **Client receives directly:**
```bash
gst-launch-1.0 \
  udpsrc port=9000 caps="application/x-rtp..." ! \
  rtph264depay ! avdec_h264 ! autovideosink
```

**Pros:**
- ✅✅✅ **Lowest possible latency** (50-80ms)
- ✅ No VPS cost
- ✅ Direct connection (no hops)
- ✅ Same as local network

**Cons:**
- ❌ Requires white IP (rare)
- ❌ Port forwarding needed
- ❌ Client IP must be known
- ❌ Doesn't work behind CGNAT
- ❌ Single client only

**Use case:** If you have Starlink with white IP - best solution

**Time to implement:** 30 minutes
**Cost:** Free

---

### Advanced Solutions (Future)

#### Option 7: QUIC Streaming

**What is QUIC:**
- New protocol by Google (used in HTTP/3)
- UDP-based but with reliability
- Multiplexing without head-of-line blocking
- 0-RTT connection establishment

**Why better:**
- Faster than TCP (no handshake delay)
- Reliable like TCP (retransmits)
- No head-of-line blocking (lost packet doesn't block stream)

**Status:**
- ⏳ Experimental in GStreamer
- ⏳ Not production-ready yet
- ⏳ Future consideration

---

#### Option 8: SRT with Aggressive Settings

**Current SRT problem:** Previous attempts had relay issues

**Better approach:** Direct SRT (no relay)
```bash
# Jetson
gst-launch-1.0 ... ! srtsink uri="srt://<CLIENT_IP>:9000" \
  latency=50 latency-max=100

# Client
gst-launch-1.0 srtsrc uri="srt://:9000?mode=listener" \
  latency=50 ! ...
```

**Why better than RTSP:**
- Packet recovery without full retransmit
- Better for lossy networks
- Lower latency than TCP

**Challenges:**
- Requires client white IP OR reverse tunnel
- More complex setup
- Still accumulates some latency

**Status:** Possible but not priority

---

#### Option 9: NDI (Network Device Interface)

**What is NDI:**
- Professional video-over-IP standard
- Used in broadcast industry
- Very low latency (~50-100ms)

**Pros:**
- Industry-proven
- Very low latency
- High quality

**Cons:**
- License required for encoding
- High bandwidth (10+ Mbps)
- Not open source

**Status:** Not recommended (licensing)

---

## Latency Budget Breakdown

### Current Setup (MediaMTX/RTSP)

| Component | Latency | Notes |
|-----------|---------|-------|
| Camera capture | 33ms | @30fps (1 frame) |
| H.264 encoding | 20-30ms | x264enc ultrafast |
| Jetson → VPS | 50-100ms | Network (Russia → Germany) |
| VPS processing | 10-20ms | FFmpeg + MediaMTX |
| VPS → Client | 50-100ms | Network (Germany → Russia) |
| Decode | 20-30ms | avdec_h264 |
| Display | 16ms | @60Hz (1 frame) |
| **Total initial** | **199-329ms** | ✅ Acceptable |
| **+ Buffer bloat** | **+100-300ms** | ❌ After 5 minutes |
| **Total after 5min** | **299-629ms** | ⚠️ Too high |

### With WebRTC (Proposed)

| Component | Latency | Notes |
|-----------|---------|-------|
| Camera capture | 33ms | @30fps |
| H.264 encoding | 20-30ms | Same |
| Jetson → VPS | 50-100ms | Same network |
| Janus processing | 5-10ms | WebRTC server |
| VPS → Client | 50-100ms | WebRTC transport |
| Decode | 20-30ms | Same |
| Display | 16ms | Same |
| **Total** | **194-319ms** | **✅ CONSTANT!** |
| **+ Buffer bloat** | **+0ms** | ✅ Feedback prevents |
| **Total after 5min** | **194-319ms** | ✅ Stays constant |

**Improvement:** Latency stays constant (no growth)

### With Russian VPS

| Component | Latency | Notes |
|-----------|---------|-------|
| Camera capture | 33ms | @30fps |
| H.264 encoding | 20-30ms | Same |
| Jetson → VPS | 20-40ms | ✅ Much faster! |
| VPS processing | 10-20ms | Same |
| VPS → Client | 20-40ms | ✅ Much faster! |
| Decode | 20-30ms | Same |
| Display | 16ms | Same |
| **Total initial** | **139-229ms** | ✅ 60ms better |
| **+ Buffer bloat** | **+100-300ms** | Still grows |
| **Total after 5min** | **239-529ms** | Better than now |

**Improvement:** Lower baseline, but still grows

### With Direct UDP (White IP)

| Component | Latency | Notes |
|-----------|---------|-------|
| Camera capture | 33ms | @30fps |
| H.264 encoding | 20-30ms | Same |
| Jetson → Client | 20-50ms | ✅ Direct! |
| Decode | 20-30ms | Same |
| Display | 16ms | Same |
| **Total** | **109-179ms** | ✅✅✅ Best! |
| **+ Buffer bloat** | **+0-50ms** | Minimal (UDP) |
| **Total after 5min** | **109-229ms** | ✅ Very stable |

**Improvement:** Best possible, like local network

## Recommendations by Use Case

### Use Case 1: Manual Driving (Current Need)

**Current solution is OK with tweaks:**

1. **Use FFplay for minimum latency:**
```bash
ffplay -probesize 32 -analyzeduration 0 -sync ext \
  -fflags nobuffer -flags low_delay -framedrop \
  rtsp://81.200.157.230:8555/camera
```
**Expected:** 80-150ms stable

2. **Or VLC with minimal cache:**
```bash
vlc --network-caching=50 --live-caching=50 \
  rtsp://81.200.157.230:8555/camera
```
**Expected:** 100-200ms

3. **Restart stream every 3-5 minutes** to reset buffers

**Acceptable for:** Manual control in open areas

---

### Use Case 2: Autonomous Navigation (Future)

**WebRTC is strongly recommended:**
- Constant latency (critical for path planning)
- FEC handles packet loss
- Production-tested

**Implementation:** 2 hours

---

### Use Case 3: Security Patrol (Recording)

**Current MediaMTX + recording:**
```bash
# On VPS
ffmpeg -i rtsp://localhost:8555/camera \
  -c copy /recordings/$(date +%Y%m%d_%H%M%S).mp4
```

**Quality requirements:**
- Latency less critical (not real-time control)
- Quality more important
- Current setup is good

---

### Use Case 4: Teleoperation (High-Speed)

**Best: Direct UDP (if possible):**
- Check for white IP on Starlink
- Setup takes 30 minutes
- 50-80ms latency

**Alternative: WebRTC**
- 100-200ms constant
- Reliable fallback

---

## Mitigation Strategies (No Changes Needed)

### Strategy 1: Restart Stream Periodically

**Automatic restart script:**
```bash
#!/bin/bash
# auto_restart_stream.sh
while true; do
    ./scripts/start_camera_mediamtx.sh
    sleep 300  # 5 minutes
    pkill -9 gst-launch
    sleep 5
done
```

**Effect:** Reset buffers every 5 minutes
**Downside:** 5 second interruption

### Strategy 2: Monitor Latency

**Measure current latency:**
```bash
# On Jetson, send timestamp in video
gst-launch-1.0 ... ! timeoverlay ! ...

# Or via RTP extensions (more accurate)
```

**Auto-restart when latency > 300ms**

### Strategy 3: Reduce Bitrate in Motion

```bash
# When robot moving: 1000 kbps (lower quality, lower latency)
./scripts/start_camera_mediamtx.sh 1280x720 30 1000

# When robot stopped: 2000 kbps (higher quality)
./scripts/start_camera_mediamtx.sh 1280x720 30 2000
```

**Effect:** Less buffering when latency is critical

---

## Implementation Roadmap

### Phase 1: Current (Complete ✅)
- ✅ Global camera streaming via MediaMTX
- ✅ Works from anywhere
- ✅ Good video quality
- ⚠️ Growing latency issue known

### Phase 2: Quick Fixes (Optional, 5 minutes)
- [ ] Document FFplay command for users
- [ ] Create wrapper scripts with optimal settings
- [ ] Add latency monitoring

### Phase 3: WebRTC (Recommended, 2 hours)
- [ ] Install Janus on VPS
- [ ] Configure streaming plugin
- [ ] Test latency (should be constant)
- [ ] Create viewer web page
- [ ] Document for team

### Phase 4: Optimization (Optional)
- [ ] Consider Russian VPS if latency critical
- [ ] Test direct UDP if Starlink has white IP
- [ ] Implement automatic stream restart
- [ ] Add telemetry (latency graphs)

---

## Technical Deep Dive

### Why Buffer Bloat Happens

**Root cause: No congestion control**

1. **Sender perspective:**
```
Jetson: "I send 30 fps constantly"
        "Network can handle it? Good!"
        "Network can't? Not my problem"
```

2. **Network perspective:**
```
Router: "Packet arrived but line busy"
        "Store in buffer (100ms capacity)"
        "Another packet? Store too (now 200ms)"
```

3. **Receiver perspective:**
```
VPS: "Got packet from 300ms ago"
     "Must play in order, buffer it"
     "Buffer growing to 500ms..."
```

**TCP makes it worse:**
- Lost packet → TCP retransmits
- New packets arrive while waiting
- Buffer grows even more

**Solution: WebRTC**
```
WebRTC Sender: "How's the buffer?"
WebRTC Receiver: "Getting full (200ms)"
WebRTC Sender: "OK, dropping frame, sending key frame"
WebRTC Receiver: "Buffer cleared, back to 100ms"
```

This is why WebRTC has constant latency!

---

### UDP vs TCP for Video

**UDP (What we use Jetson → VPS):**
```
Send packet → If lost, skip it → No waiting
✅ Low latency
❌ Packet loss = artifacts
```

**TCP (What happens VPS → Client):**
```
Send packet → If lost, wait & retransmit → Buffer grows
✅ No packet loss
❌ Growing latency
```

**WebRTC (Best of both):**
```
Send packet → If lost, decide:
  - Important? Retransmit
  - Old? Skip
  - Tell sender to adjust
✅ Low latency + reliability
```

---

### MTU and Fragmentation

**Current:** 1500 byte MTU (standard internet)

**H.264 NAL units:** Can be 10KB-100KB
**RTP payload:** 1200 bytes max

**What happens:**
```
10KB keyframe → Split into 8 RTP packets
Lose 1 packet → Entire frame corrupted → Artifacts until next keyframe
```

**More key frames = fewer artifacts but higher bitrate**

Current setting: Key frame every 30 frames (1 second)
- Good for bandwidth
- Artifacts last up to 1 second on packet loss

Could increase: Key frame every 15 frames (0.5 second)
- Higher bandwidth (+20%)
- Artifacts last only 0.5 second

---

## Conclusion

**Current status:** ✅ **Working and usable**

**Known limitation:** Latency grows over time (5-10 minutes)

**Best for:**
- ✅ Manual driving (restart stream as needed)
- ✅ Monitoring/security
- ✅ Testing autonomous features

**Not ideal for:**
- ⚠️ High-speed teleoperation
- ⚠️ Precision tasks
- ⚠️ Long sessions without restart

**Recommended next step:**
🚀 **Implement WebRTC** (2 hours) for constant latency

**Alternative:**
🌍 **Move to Russian VPS** (1 hour) for 50% lower baseline

**Quick fix:**
⚡ **Use FFplay** (1 minute) for minimal latency right now

---

**Document created:** November 11, 2025
**Status:** Comprehensive analysis complete
**Next review:** After WebRTC implementation
