"""
Channel Health Monitoring

Monitors health of all communication channels and determines availability.
"""

import time
from enum import Enum
from typing import Dict, Optional


class ChannelState(Enum):
    """Channel health states"""
    UNKNOWN = 0
    HEALTHY = 1
    DEGRADED = 2
    FAILED = 3


class ChannelHealth:
    """Monitor health of a single communication channel"""

    def __init__(self, name: str, timeout: float = 2.0):
        """
        Initialize channel health monitor

        Args:
            name: Channel name (fiber, starlink, 4g, wifi, dmr, expresslrs)
            timeout: Timeout in seconds before channel considered failed
        """
        self.name = name
        self.timeout = timeout
        self.state = ChannelState.UNKNOWN
        self.last_heartbeat = 0.0
        self.last_data = 0.0
        self.message_count = 0
        self.error_count = 0

    def update_heartbeat(self) -> None:
        """Update heartbeat timestamp"""
        self.last_heartbeat = time.time()

    def update_data(self) -> None:
        """Update data received timestamp"""
        self.last_data = time.time()
        self.message_count += 1

    def report_error(self) -> None:
        """Report communication error"""
        self.error_count += 1

    def check_health(self) -> ChannelState:
        """
        Check channel health based on timeouts

        Returns:
            Current channel state
        """
        current_time = time.time()

        # Use most recent timestamp (heartbeat or data)
        last_contact = max(self.last_heartbeat, self.last_data)

        if last_contact == 0.0:
            # Never received any data
            self.state = ChannelState.UNKNOWN
        elif current_time - last_contact > self.timeout:
            # Timeout exceeded
            self.state = ChannelState.FAILED
        elif self.error_count > 0 and self.message_count > 0:
            # Has errors but still receiving data
            error_rate = self.error_count / self.message_count
            if error_rate > 0.3:
                self.state = ChannelState.DEGRADED
            else:
                self.state = ChannelState.HEALTHY
        else:
            # Receiving data without errors
            self.state = ChannelState.HEALTHY

        return self.state

    def reset_stats(self) -> None:
        """Reset error and message counters"""
        self.message_count = 0
        self.error_count = 0

    def get_info(self) -> Dict:
        """
        Get channel information

        Returns:
            Dictionary with channel stats
        """
        current_time = time.time()
        last_contact = max(self.last_heartbeat, self.last_data)

        return {
            'name': self.name,
            'state': self.state.name,
            'timeout': self.timeout,
            'last_contact': last_contact,
            'time_since_contact': current_time - last_contact if last_contact > 0 else -1,
            'message_count': self.message_count,
            'error_count': self.error_count,
            'error_rate': self.error_count / self.message_count if self.message_count > 0 else 0.0
        }


class ChannelHealthManager:
    """Manage health monitoring for all channels"""

    def __init__(self):
        """Initialize channel health manager"""
        self.channels: Dict[str, ChannelHealth] = {}

    def add_channel(self, name: str, timeout: float = 2.0) -> None:
        """
        Add channel to monitor

        Args:
            name: Channel name
            timeout: Timeout in seconds
        """
        self.channels[name] = ChannelHealth(name, timeout)

    def remove_channel(self, name: str) -> None:
        """
        Remove channel from monitoring

        Args:
            name: Channel name
        """
        if name in self.channels:
            del self.channels[name]

    def update_heartbeat(self, name: str) -> None:
        """
        Update heartbeat for channel

        Args:
            name: Channel name
        """
        if name in self.channels:
            self.channels[name].update_heartbeat()

    def update_data(self, name: str) -> None:
        """
        Update data received for channel

        Args:
            name: Channel name
        """
        if name in self.channels:
            self.channels[name].update_data()

    def report_error(self, name: str) -> None:
        """
        Report error for channel

        Args:
            name: Channel name
        """
        if name in self.channels:
            self.channels[name].report_error()

    def check_all(self) -> Dict[str, ChannelState]:
        """
        Check health of all channels

        Returns:
            Dictionary mapping channel names to states
        """
        states = {}
        for name, channel in self.channels.items():
            states[name] = channel.check_health()
        return states

    def get_healthy_channels(self) -> list:
        """
        Get list of healthy channels

        Returns:
            List of channel names that are healthy
        """
        self.check_all()
        return [name for name, channel in self.channels.items()
                if channel.state == ChannelState.HEALTHY]

    def get_channel_info(self, name: str) -> Optional[Dict]:
        """
        Get information for specific channel

        Args:
            name: Channel name

        Returns:
            Channel info dictionary or None if not found
        """
        if name in self.channels:
            return self.channels[name].get_info()
        return None

    def get_all_info(self) -> Dict[str, Dict]:
        """
        Get information for all channels

        Returns:
            Dictionary mapping channel names to info dictionaries
        """
        return {name: channel.get_info()
                for name, channel in self.channels.items()}
