#!/bin/bash
# Script to set up SSH tunnel to VPS for remote Jetson access

VPS_HOST="81.200.157.230"
VPS_USER="root"
TUNNEL_PORT=2223  # Port on VPS to access this Jetson (avoiding conflicts with other tunnels)
LOCAL_SSH_PORT=22

echo "Setting up SSH tunnel to VPS..."
echo "This Jetson will be accessible via: ssh -p $TUNNEL_PORT jetson@$VPS_HOST"
echo ""

# Copy public key to VPS
echo "Step 1: Adding SSH public key to VPS..."
echo "You will be prompted for the VPS password."
ssh-copy-id -i ~/.ssh/id_ed25519_vps.pub $VPS_USER@$VPS_HOST

if [ $? -eq 0 ]; then
    echo "✓ SSH key successfully copied to VPS"
else
    echo "✗ Failed to copy SSH key. Please check your VPS password and connection."
    exit 1
fi

# Test connection
echo ""
echo "Step 2: Testing SSH connection..."
ssh -i ~/.ssh/id_ed25519_vps -o BatchMode=yes -o ConnectTimeout=5 $VPS_USER@$VPS_HOST "echo 'Connection successful'"

if [ $? -eq 0 ]; then
    echo "✓ SSH connection test passed"
else
    echo "✗ SSH connection test failed"
    exit 1
fi

echo ""
echo "✓ Setup complete!"
echo ""
echo "Next steps:"
echo "1. The tunnel service will be created and started"
echo "2. You'll be able to access this Jetson from anywhere via:"
echo "   ssh -p $TUNNEL_PORT jetson@$VPS_HOST"
