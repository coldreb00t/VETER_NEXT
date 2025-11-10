"""
Failover Logic

Implements priority-based failover chain with hysteresis.
"""

import time
from typing import List, Optional, Dict
from .channel_health import ChannelHealthManager, ChannelState


class FailoverManager:
    """Manage failover between communication channels"""

    def __init__(self, health_manager: ChannelHealthManager):
        """
        Initialize failover manager

        Args:
            health_manager: Channel health manager instance
        """
        self.health_manager = health_manager
        self.priority_chain: List[str] = []
        self.current_channel: Optional[str] = None
        self.last_switch_time = 0.0
        self.hysteresis_time = 5.0  # Seconds before allowing channel switch back
        self.switch_history: List[Dict] = []

    def set_priority_chain(self, channels: List[str]) -> None:
        """
        Set priority chain for failover

        Args:
            channels: Ordered list of channel names (highest to lowest priority)
        """
        self.priority_chain = channels.copy()

        # If no current channel, select first available
        if self.current_channel is None:
            self.select_best_channel()

    def get_priority_chain(self) -> List[str]:
        """
        Get current priority chain

        Returns:
            Ordered list of channel names
        """
        return self.priority_chain.copy()

    def select_best_channel(self, force: bool = False) -> Optional[str]:
        """
        Select best available channel based on priority chain

        Args:
            force: Force channel switch ignoring hysteresis

        Returns:
            Selected channel name or None if no healthy channels
        """
        # Get current channel states
        channel_states = self.health_manager.check_all()

        # Find first healthy channel in priority chain
        for channel in self.priority_chain:
            if channel in channel_states and channel_states[channel] == ChannelState.HEALTHY:
                # Check if we should switch to this channel
                if self._should_switch_to(channel, force):
                    self._switch_to_channel(channel)
                    return channel

        # No healthy channel found - check for degraded channels
        for channel in self.priority_chain:
            if channel in channel_states and channel_states[channel] == ChannelState.DEGRADED:
                if self._should_switch_to(channel, force):
                    self._switch_to_channel(channel)
                    return channel

        # No usable channel found - go to safe stop
        if self.current_channel != 'safe_stop':
            self._switch_to_channel('safe_stop')

        return None

    def _should_switch_to(self, new_channel: str, force: bool = False) -> bool:
        """
        Check if we should switch to new channel

        Args:
            new_channel: Target channel name
            force: Force switch ignoring hysteresis

        Returns:
            True if should switch, False otherwise
        """
        # No current channel - always switch
        if self.current_channel is None:
            return True

        # Already on this channel - no switch needed
        if self.current_channel == new_channel:
            return False

        # Force switch
        if force:
            return True

        # Get priorities
        try:
            current_priority = self.priority_chain.index(self.current_channel)
        except ValueError:
            # Current channel not in priority chain - switch immediately
            return True

        try:
            new_priority = self.priority_chain.index(new_channel)
        except ValueError:
            # New channel not in priority chain - don't switch
            return False

        # Switch to higher priority channel (lower index)
        if new_priority < current_priority:
            # Check hysteresis time
            time_since_switch = time.time() - self.last_switch_time
            if time_since_switch >= self.hysteresis_time:
                return True
            else:
                return False

        # Switch to lower priority only if current channel failed
        if new_priority > current_priority:
            current_state = self.health_manager.channels.get(self.current_channel)
            if current_state and current_state.state in [ChannelState.FAILED, ChannelState.UNKNOWN]:
                return True
            else:
                return False

        return False

    def _switch_to_channel(self, channel: str) -> None:
        """
        Switch to new channel

        Args:
            channel: Target channel name
        """
        old_channel = self.current_channel
        self.current_channel = channel
        self.last_switch_time = time.time()

        # Record switch in history
        self.switch_history.append({
            'timestamp': self.last_switch_time,
            'from': old_channel,
            'to': channel,
            'reason': self._get_switch_reason(old_channel, channel)
        })

        # Keep only last 100 switches
        if len(self.switch_history) > 100:
            self.switch_history = self.switch_history[-100:]

    def _get_switch_reason(self, old_channel: Optional[str], new_channel: str) -> str:
        """
        Determine reason for channel switch

        Args:
            old_channel: Previous channel name
            new_channel: New channel name

        Returns:
            Reason string
        """
        if old_channel is None:
            return 'initial_selection'

        if new_channel == 'safe_stop':
            return 'no_healthy_channels'

        try:
            old_priority = self.priority_chain.index(old_channel)
            new_priority = self.priority_chain.index(new_channel)

            if new_priority < old_priority:
                return 'higher_priority_available'
            elif new_priority > old_priority:
                return 'current_channel_failed'
            else:
                return 'manual_switch'
        except ValueError:
            return 'priority_chain_updated'

    def get_current_channel(self) -> Optional[str]:
        """
        Get current active channel

        Returns:
            Current channel name or None
        """
        return self.current_channel

    def get_switch_history(self, count: int = 10) -> List[Dict]:
        """
        Get recent channel switch history

        Args:
            count: Number of recent switches to return

        Returns:
            List of switch records
        """
        return self.switch_history[-count:]

    def get_status(self) -> Dict:
        """
        Get failover manager status

        Returns:
            Status dictionary
        """
        return {
            'current_channel': self.current_channel,
            'priority_chain': self.priority_chain,
            'last_switch_time': self.last_switch_time,
            'time_since_switch': time.time() - self.last_switch_time,
            'hysteresis_time': self.hysteresis_time,
            'total_switches': len(self.switch_history)
        }

    def force_channel(self, channel: str) -> bool:
        """
        Force switch to specific channel

        Args:
            channel: Target channel name

        Returns:
            True if successful, False if channel not available
        """
        if channel not in self.priority_chain:
            return False

        channel_state = self.health_manager.channels.get(channel)
        if channel_state is None:
            return False

        # Force switch regardless of health
        self._switch_to_channel(channel)
        return True

    def set_hysteresis_time(self, seconds: float) -> None:
        """
        Set hysteresis time for channel switching

        Args:
            seconds: Hysteresis time in seconds
        """
        self.hysteresis_time = max(0.0, seconds)
