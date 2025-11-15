# Tailscale VPN Setup for VETER_NEXT

**Setup Date:** November 15, 2025
**Tailscale Version:** 1.90.6
**Status:** ✅ Configured and Working

---

## Overview

Tailscale обеспечивает **резервный канал связи** с роботом в дополнение к SSH reverse tunnel. Это zero-config VPN с автоматическим NAT traversal и P2P подключением когда возможно.

### Преимущества

1. ✅ **Надежность** - автоматическое переподключение
2. ✅ **Простота** - zero-config, работает из коробки
3. ✅ **Безопасность** - WireGuard encryption, no open ports
4. ✅ **P2P** - прямое подключение когда возможно (низкая латентность)
5. ✅ **Mesh Network** - все устройства видят друг друга
6. ✅ **Работает везде** - через любое интернет-подключение (4G, 5G, WiFi, Starlink)

---

## Installation

**Installed:** November 15, 2025

```bash
# Download and run official install script
curl -fsSL https://tailscale.com/install.sh -o /tmp/tailscale-install.sh
chmod +x /tmp/tailscale-install.sh
sudo sh /tmp/tailscale-install.sh
```

**Output:**
```
Installation complete! Log in to start using Tailscale by running:
tailscale up
```

---

## Configuration

### 1. Initial Setup

```bash
# Start Tailscale with custom hostname
sudo tailscale up --hostname=veter-jetson-robot
```

**Authentication:**
- Visit URL: https://login.tailscale.com/a/XXXXXXXX
- Login with Google/GitHub/Microsoft
- Authorize device

### 2. Auto-Start Configuration

**Service:** `tailscaled.service` (automatically enabled during installation)

```bash
# Check service status
systemctl is-enabled tailscaled.service  # Output: enabled
systemctl is-active tailscaled.service   # Output: active

# Service starts automatically on boot
```

### 3. Network Configuration

**Jetson Tailscale IP:** `100.112.41.76`
**Hostname:** `veter-jetson-robot`
**DNS Name:** `veter-jetson-robot.tailffe24b.ts.net`

**Network Details:**
- UDP: ✅ Working
- Public IP: 46.246.9.213
- Nearest DERP relay: Helsinki (98.7ms)
- NAT Traversal: ✅ Successful

---

## Usage

### Access from Other Devices

**From MacBook/PC/Phone (in Tailscale network):**

```bash
# SSH via Tailscale IP
ssh jetson@100.112.41.76

# SSH via hostname
ssh jetson@veter-jetson-robot

# SSH via DNS name
ssh jetson@veter-jetson-robot.tailffe24b.ts.net
```

**ROS2 Commands:**
```bash
# Set ROS_DOMAIN_ID and connect
export ROS_DOMAIN_ID=0
export ROS_MASTER_URI=http://100.112.41.76:11311

# View ROS2 topics
ros2 topic list

# Control robot
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}}"
```

**Camera Streaming:**
```bash
# RTSP stream via Tailscale
vlc rtsp://100.112.41.76:8554/camera

# GStreamer
gst-launch-1.0 rtspsrc location=rtsp://100.112.41.76:8554/camera ! decodebin ! autovideosink
```

---

## Network Status

### Current Devices in Tailscale Network

```bash
sudo tailscale status
```

**Output:**
```
100.112.41.76  veter-jetson-robot   coldreb00t@  linux  online
100.74.130.99  macbook-air-eugene   coldreb00t@  macOS  online
100.118.28.72  ubuntu               coldreb00t@  linux  offline
```

### Network Quality Check

```bash
sudo tailscale netcheck
```

**DERP Latencies:**
- Helsinki: 98.7ms ← **Nearest**
- Frankfurt: 100.1ms
- Amsterdam: 100.2ms
- Nuremberg: 104.1ms
- London: 107.1ms

---

## Comparison: Tailscale vs SSH Tunnel

| Feature | SSH Reverse Tunnel | Tailscale VPN |
|---------|-------------------|---------------|
| **Setup** | Manual (systemd service) | Zero-config |
| **Reliability** | Manual restart needed | Auto-reconnect |
| **NAT Traversal** | Requires VPS relay | Built-in (STUN/DERP) |
| **Latency** | Via VPS (variable) | P2P when possible (~10-50ms) |
| **Security** | SSH encryption | WireGuard encryption |
| **Mesh Network** | No (point-to-point) | Yes (all devices) |
| **Mobile Support** | Limited | Excellent (iOS/Android apps) |
| **Cost** | VPS required ($5/mo) | Free for personal use |

**Verdict:** Use **both** for redundancy!

---

## Integration with VETER_NEXT

### Current Setup (2 Channels)

1. **SSH Reverse Tunnel** (Primary)
   - Via VPS: `ssh -p 2223 jetson@81.200.157.230`
   - Ports: 2223 (SSH), 8554 (RTSP camera)

2. **Tailscale VPN** (Backup)
   - Direct: `ssh jetson@100.112.41.76`
   - All services accessible via Tailscale IP

### Future: Add to Channel Manager

**Potential integration** as 7th communication channel:

```yaml
# config/channels_tailscale.yaml
channel_manager:
  enabled_channels:
    - tailscale       # NEW!
    - fiber
    - starlink
    - 4g
    - wifi
    - expresslrs

  priority_chain:
    1: fiber
    2: tailscale      # High priority (reliable + low latency)
    3: starlink
    4: 4g
    5: wifi
    6: expresslrs
    7: safe_stop

  timeouts:
    tailscale: 2.0    # Fast timeout (WireGuard keep-alive)
```

**Implementation Notes:**
- Tailscale provides stable IPs (100.x.x.x range)
- Can use Tailscale IP as ROS2 communication channel
- Ideal for autonomous mode when no operator present

---

## Troubleshooting

### Check Connection Status

```bash
# Overall status
sudo tailscale status

# Detailed network info
sudo tailscale netcheck

# Show current IP
sudo tailscale ip -4

# Ping another device
sudo tailscale ping macbook-air-eugene
```

### Service Issues

```bash
# Restart service
sudo systemctl restart tailscaled

# View logs
sudo journalctl -u tailscaled -f

# Check firewall (Jetson uses ufw)
sudo ufw status
```

### DNS Issues

**Warning:** "Tailscale can't reach the configured DNS servers"

**Fix (if needed):**
```bash
# Use Tailscale's MagicDNS
sudo tailscale up --accept-dns=true

# Or disable DNS warnings
sudo tailscale up --accept-dns=false
```

**Current:** DNS warning present but not affecting connectivity

---

## Security Considerations

### Authentication

- ✅ OAuth-based (Google/GitHub/Microsoft)
- ✅ Device authorization required
- ✅ Multi-factor authentication supported
- ✅ Can revoke devices remotely

### Encryption

- ✅ WireGuard protocol (state-of-the-art)
- ✅ End-to-end encryption
- ✅ Perfect forward secrecy
- ✅ Regular key rotation

### Network Isolation

```bash
# Disable key expiry (for robot that may be offline long periods)
sudo tailscale up --timeout=0

# Enable MagicDNS (easier hostname resolution)
sudo tailscale up --accept-dns=true
```

---

## Maintenance

### Regular Checks

```bash
# Weekly: Check connection status
sudo tailscale status

# Monthly: Update Tailscale
sudo apt update && sudo apt upgrade tailscale

# Quarterly: Review connected devices
# Visit https://login.tailscale.com/admin/machines
```

### Updates

```bash
# Check current version
tailscale version

# Update to latest
sudo apt update
sudo apt upgrade tailscale

# Restart after update
sudo systemctl restart tailscaled
```

---

## References

- **Tailscale Docs:** https://tailscale.com/kb/
- **WireGuard Protocol:** https://www.wireguard.com/
- **Admin Console:** https://login.tailscale.com/admin/
- **VETER_NEXT Docs:** `/home/jetson/jetson-robot-project/docs/`

---

## Configuration Summary

| Parameter | Value |
|-----------|-------|
| **Service** | tailscaled.service |
| **Status** | enabled, active |
| **Hostname** | veter-jetson-robot |
| **Tailscale IP** | 100.112.41.76 |
| **DNS Name** | veter-jetson-robot.tailffe24b.ts.net |
| **Public IP** | 46.246.9.213 |
| **Nearest Relay** | Helsinki (98.7ms) |
| **UDP** | Working ✅ |
| **P2P** | Available ✅ |

---

**Setup completed:** November 15, 2025
**Tested:** ✅ Working
**Status:** ✅ Production Ready
