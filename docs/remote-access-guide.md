# Remote Access Guide

## Tunnel Status: ✓ Active

The SSH reverse tunnel is successfully configured and running!

## Accessing Jetson Remotely

From any computer with internet access:

```bash
ssh -p 2223 jetson@81.200.157.230
```

Password: `cvbp`

## Verification

The tunnel is active and listening on VPS port 2223:
- Jetson hostname: `ubuntu`
- VPS hostname: `5073427-ap85932`
- Tunnel port: `2223`

## Service Management

### Check tunnel status
```bash
systemctl status ssh-tunnel-jetson-robot
```

### Restart tunnel
```bash
sudo systemctl restart ssh-tunnel-jetson-robot
```

### View tunnel logs
```bash
sudo journalctl -u ssh-tunnel-jetson-robot -f
```

### Stop tunnel
```bash
sudo systemctl stop ssh-tunnel-jetson-robot
```

### Start tunnel
```bash
sudo systemctl start ssh-tunnel-jetson-robot
```

## Automatic Startup

The tunnel service is configured to:
- Start automatically on boot
- Restart automatically if connection is lost
- Keep the connection alive with heartbeat packets every 30 seconds

## Security

- The tunnel uses SSH key authentication (ed25519)
- Private key: `/home/jetson/.ssh/id_ed25519_vps`
- Connection to VPS is encrypted
- Access to Jetson through the tunnel still requires jetson user credentials

## Troubleshooting

### Check if tunnel process is running
```bash
ps aux | grep "ssh.*2223"
```

### Test connection from Jetson to VPS
```bash
ssh -i ~/.ssh/id_ed25519_vps root@81.200.157.230
```

### Verify port is open on VPS
From Jetson:
```bash
ssh -i ~/.ssh/id_ed25519_vps root@81.200.157.230 "ss -tlnp | grep 2223"
```

### Check for firewall issues
If you cannot connect from outside, check:
- VPS firewall allows incoming connections on port 2223
- Your local firewall allows outgoing SSH connections
- Network allows SSH traffic

## Advanced: SSH Config

You can simplify access by adding this to your `~/.ssh/config` on your local machine:

```
Host jetson-robot
    HostName 81.200.157.230
    Port 2223
    User jetson
```

Then connect with:
```bash
ssh jetson-robot
```
