#!/usr/bin/env python3
"""
Script to add SSH public key to VPS server
"""

import subprocess
import sys
import os

VPS_HOST = "81.200.157.230"
VPS_USER = "root"
VPS_PASSWORD = "zXJEV.QC3mtp^A"
KEY_FILE = os.path.expanduser("~/.ssh/id_ed25519_vps.pub")

def run_command(cmd, input_text=None):
    """Run a shell command"""
    try:
        result = subprocess.run(
            cmd,
            shell=True,
            capture_output=True,
            text=True,
            input=input_text,
            timeout=30
        )
        return result.returncode, result.stdout, result.stderr
    except subprocess.TimeoutExpired:
        return 1, "", "Timeout"

def main():
    print("Setting up SSH key authentication to VPS...")
    print()

    # Read public key
    try:
        with open(KEY_FILE, 'r') as f:
            pub_key = f.read().strip()
        print(f"✓ Read public key from {KEY_FILE}")
    except FileNotFoundError:
        print(f"✗ Key file not found: {KEY_FILE}")
        return 1

    # Method 1: Try using sshpass if available
    code, _, _ = run_command("which sshpass")
    if code == 0:
        print("Using sshpass method...")
        cmd = f"sshpass -p '{VPS_PASSWORD}' ssh-copy-id -i {KEY_FILE} -o StrictHostKeyChecking=no {VPS_USER}@{VPS_HOST}"
        code, out, err = run_command(cmd)
        if code == 0:
            print("✓ Key copied using sshpass")
        else:
            print(f"✗ sshpass failed: {err}")
            return 1
    else:
        # Method 2: Manual SSH command to add key
        print("Using direct SSH method...")
        ssh_cmd = f"mkdir -p ~/.ssh && chmod 700 ~/.ssh && echo '{pub_key}' >> ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys && echo 'Key added'"

        # This will prompt for password interactively
        cmd = f"ssh -o StrictHostKeyChecking=no {VPS_USER}@{VPS_HOST} \"{ssh_cmd}\""
        print(f"\nPlease enter VPS password when prompted: {VPS_PASSWORD}\n")
        code = os.system(cmd)

        if code != 0:
            print("✗ Failed to add key")
            return 1

    # Test the connection
    print("\nTesting SSH key authentication...")
    cmd = f"ssh -i ~/.ssh/id_ed25519_vps -o BatchMode=yes -o ConnectTimeout=5 {VPS_USER}@{VPS_HOST} 'echo Connection successful'"
    code, out, err = run_command(cmd)

    if code == 0 and "Connection successful" in out:
        print("✓ SSH key authentication works!")
        print()
        print("Next step: Set up the tunnel service")
        print("  sudo cp scripts/ssh-tunnel-jetson-robot.service /etc/systemd/system/")
        print("  sudo systemctl daemon-reload")
        print("  sudo systemctl enable ssh-tunnel-jetson-robot")
        print("  sudo systemctl start ssh-tunnel-jetson-robot")
        return 0
    else:
        print(f"✗ Authentication test failed: {err}")
        return 1

if __name__ == "__main__":
    sys.exit(main())
