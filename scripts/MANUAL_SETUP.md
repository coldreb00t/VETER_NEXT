# Manual SSH Tunnel Setup Instructions

Since automated key copying requires certain packages or VPS configuration, here are manual steps:

## Option 1: Copy Key from Terminal (Recommended)

Open a terminal on the Jetson and run:

```bash
ssh-copy-id -i ~/.ssh/id_ed25519_vps.pub root@81.200.157.230
```

When prompted, enter password: `zXJEV.QC3mtp^A`

## Option 2: Add Key Manually on VPS

If you have access to the VPS directly:

1. Get the public key content:
   ```bash
   cat ~/.ssh/id_ed25519_vps.pub
   ```

2. SSH to VPS:
   ```bash
   ssh root@81.200.157.230
   ```

3. On the VPS, add the key:
   ```bash
   mkdir -p ~/.ssh
   chmod 700 ~/.ssh
   echo "PASTE_PUBLIC_KEY_HERE" >> ~/.ssh/authorized_keys
   chmod 600 ~/.ssh/authorized_keys
   ```

## After Key is Added

1. Test the connection from Jetson:
   ```bash
   ssh -i ~/.ssh/id_ed25519_vps root@81.200.157.230
   ```

2. Install the tunnel service:
   ```bash
   cd /home/jetson/jetson-robot-project
   sudo cp scripts/ssh-tunnel-jetson-robot.service /etc/systemd/system/
   sudo systemctl daemon-reload
   sudo systemctl enable ssh-tunnel-jetson-robot
   sudo systemctl start ssh-tunnel-jetson-robot
   ```

3. Check the status:
   ```bash
   sudo systemctl status ssh-tunnel-jetson-robot
   ```

4. Test remote access:
   From any computer with internet access:
   ```bash
   ssh -p 2223 jetson@81.200.157.230
   ```

## Troubleshooting

If ssh-copy-id asks for a password but doesn't accept it, the VPS may have password authentication disabled in `/etc/ssh/sshd_config`. In this case, use Option 2.
