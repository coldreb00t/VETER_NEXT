# VPS SSH Tunnel Setup

This document describes the SSH tunnel setup for remote access to the Jetson robot.

## Overview

An SSH reverse tunnel is established from the Jetson to the VPS server at `81.200.157.230`, allowing remote access to the Jetson from anywhere in the world.

## Architecture

```
Jetson (behind NAT) --SSH Tunnel--> VPS (81.200.157.230:2223)
                                      ^
                                      |
                               You can connect here
```

## Ports

- **VPS Port 2223**: Reverse tunnel endpoint for this Jetson robot project
  - Connects to Jetson's local SSH port 22
  - Used to avoid conflicts with other existing tunnels on the VPS

## Initial Setup

1. **Add SSH key to VPS** (one-time):
   ```bash
   cd /home/jetson/jetson-robot-project
   chmod +x scripts/setup-vps-tunnel.sh
   ./scripts/setup-vps-tunnel.sh
   ```

   You will be prompted for the VPS password during setup.

2. **Install and enable the tunnel service**:
   ```bash
   sudo cp scripts/ssh-tunnel-jetson-robot.service /etc/systemd/system/
   sudo systemctl daemon-reload
   sudo systemctl enable ssh-tunnel-jetson-robot
   sudo systemctl start ssh-tunnel-jetson-robot
   ```

3. **Check tunnel status**:
   ```bash
   sudo systemctl status ssh-tunnel-jetson-robot
   ```

## Remote Access

Once the tunnel is established, you can access the Jetson from anywhere:

```bash
ssh -p 2223 jetson@81.200.157.230
```

## Troubleshooting

### Check if tunnel is running
```bash
sudo systemctl status ssh-tunnel-jetson-robot
```

### View tunnel logs
```bash
sudo journalctl -u ssh-tunnel-jetson-robot -f
```

### Restart tunnel
```bash
sudo systemctl restart ssh-tunnel-jetson-robot
```

### Test connection from Jetson to VPS
```bash
ssh -i ~/.ssh/id_ed25519_vps root@81.200.157.230
```

### Check if port 2223 is listening on VPS
From the VPS:
```bash
ss -tulpn | grep 2223
```

## Security Notes

- The tunnel uses SSH key authentication (ed25519)
- Private key location: `/home/jetson/.ssh/id_ed25519_vps`
- The tunnel automatically reconnects if the connection is lost
- ServerAliveInterval is set to 30 seconds to keep the connection alive

## Port Conflicts

**Important**: This setup uses port 2223 on the VPS to avoid conflicts with other existing tunnels. If you need to change the port, edit:
1. `scripts/setup-vps-tunnel.sh` - update TUNNEL_PORT variable
2. `scripts/ssh-tunnel-jetson-robot.service` - update the -R parameter
