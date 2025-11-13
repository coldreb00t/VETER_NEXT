#!/bin/bash
# Setup reverse SSH tunnel for web interface
# Пробрасывает порты 8000 (HTTP) и 9090 (WebSocket) через VPS

VPS_HOST="81.200.157.230"
VPS_USER="root"
SSH_KEY="/home/jetson/.ssh/id_ed25519_vps"
LOCAL_HTTP_PORT="8000"
LOCAL_WS_PORT="9090"
REMOTE_HTTP_PORT="8000"
REMOTE_WS_PORT="9090"

echo "🌐 Setting up reverse SSH tunnel for web interface..."
echo "   HTTP: localhost:$LOCAL_HTTP_PORT → $VPS_HOST:$REMOTE_HTTP_PORT"
echo "   WebSocket: localhost:$LOCAL_WS_PORT → $VPS_HOST:$REMOTE_WS_PORT"
echo "   Using SSH key: $SSH_KEY"

# Create SSH tunnel with autossh for reliability
# -M 0: disable monitoring port (use ServerAliveInterval instead)
# -N: no remote commands
# -T: no pseudo-tty
# -R: reverse tunnel
# -i: identity file (SSH key)
autossh -M 0 \
  -o "ServerAliveInterval=30" \
  -o "ServerAliveCountMax=3" \
  -o "StrictHostKeyChecking=no" \
  -o "ExitOnForwardFailure=yes" \
  -i "$SSH_KEY" \
  -N -T \
  -R $REMOTE_HTTP_PORT:localhost:$LOCAL_HTTP_PORT \
  -R $REMOTE_WS_PORT:localhost:$LOCAL_WS_PORT \
  $VPS_USER@$VPS_HOST
