#!/usr/bin/expect -f
# Automated script to add SSH key to VPS
# Requires: expect package

set timeout 30
set vps_host "81.200.157.230"
set vps_user "root"
set vps_password "zXJEV.QC3mtp^A"

# Read the public key
set fp [open "/home/jetson/.ssh/id_ed25519_vps.pub" r]
set pub_key [read $fp]
close $fp

# SSH to VPS and add the key
spawn ssh -o StrictHostKeyChecking=no $vps_user@$vps_host "mkdir -p ~/.ssh && chmod 700 ~/.ssh && echo '$pub_key' >> ~/.ssh/authorized_keys && chmod 600 ~/.ssh/authorized_keys && echo 'Key added successfully'"

expect {
    "password:" {
        send "$vps_password\r"
        exp_continue
    }
    "Key added successfully" {
        puts "\n✓ SSH key successfully added to VPS"
    }
    timeout {
        puts "\n✗ Connection timeout"
        exit 1
    }
    eof {
        puts "\n✓ Done"
    }
}

# Test the connection
puts "\nTesting SSH key authentication..."
spawn ssh -i /home/jetson/.ssh/id_ed25519_vps -o BatchMode=yes $vps_user@$vps_host "echo 'Connection successful'"

expect {
    "Connection successful" {
        puts "\n✓ SSH key authentication works!"
        exit 0
    }
    timeout {
        puts "\n✗ Authentication failed"
        exit 1
    }
}
