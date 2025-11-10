# VETER Bringup Package

Launch files and configurations for VETER_NEXT robot system.

## Launch Files

### Minimal System (RC Control Only)
```bash
ros2 launch veter_bringup veter_minimal.launch.py
```
- DroneCAN Bridge (ESP32 communication)
- Channel Manager (RC-only mode)

### Full System (All Components)
```bash
ros2 launch veter_bringup veter_full.launch.py
```
- All components + MAVROS + Navigation

### Teleop Testing
```bash
ros2 launch veter_bringup veter_teleop.launch.py
```
- Keyboard control for testing

### MAVROS Only
```bash
ros2 launch veter_bringup mavros.launch.py
```
- Mini Pixhawk GPS/IMU interface

## Auto-Start Setup

```bash
# Install service
sudo cp scripts/veter.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable veter
sudo systemctl start veter

# Check status
sudo systemctl status veter

# View logs
sudo journalctl -u veter -f
```

## Configuration

Edit config files in `config/` directory:
- `robot_params.yaml` - Robot specifications
- `mavros_config.yaml` - MAVROS settings

## Topics

See individual package READMEs:
- veter_dronecan_bridge
- veter_channel_manager

## Author

Eugene Melnik (eugene.a.melnik@gmail.com)
