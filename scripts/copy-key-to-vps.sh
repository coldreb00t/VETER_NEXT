#!/bin/bash
# Script to copy SSH key to VPS using password
# Usage: ./copy-key-to-vps.sh

VPS_HOST="81.200.157.230"
VPS_USER="root"
VPS_PASSWORD="zXJEV.QC3mtp^A"

echo "Installing sshpass if needed..."
if ! command -v sshpass &> /dev/null; then
    echo "sshpass not found. Installing..."
    sudo apt-get update && sudo apt-get install -y sshpass
fi

echo "Copying SSH public key to VPS..."
sshpass -p "$VPS_PASSWORD" ssh-copy-id -i ~/.ssh/id_ed25519_vps.pub -o StrictHostKeyChecking=no $VPS_USER@$VPS_HOST

if [ $? -eq 0 ]; then
    echo "✓ SSH key successfully copied to VPS"

    # Test connection
    echo "Testing connection..."
    ssh -i ~/.ssh/id_ed25519_vps -o BatchMode=yes $VPS_USER@$VPS_HOST "echo 'Connection successful'"

    if [ $? -eq 0 ]; then
        echo "✓ Connection test passed!"
        echo ""
        echo "You can now set up the tunnel service."
    fi
else
    echo "✗ Failed to copy key"
    exit 1
fi
