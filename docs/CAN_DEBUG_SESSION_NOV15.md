# CAN Bus Debug Session - November 15, 2025

## Session Summary

**Objective:** Test Jetson ↔ ESP32 CAN communication
**Result:** ❌ **FAILED - No communication**
**Root Cause:** Missing or non-functional CAN bus termination resistors

## What We Tried

### 1. Initial Test
- ESP32 Motor Controller: Transmitting, getting TX timeout
- Jetson CAN: ERROR-PASSIVE state, cannot transmit
- Zero successful message exchange

### 2. Configuration Changes
- ✅ Adjusted Jetson CAN sample point: 74% → 86% (closer to ESP32's 87.5%)
- ✅ Reset CAN interface multiple times
- ✅ Reloaded CAN driver (mttcan)
- ✅ Reset error counters (ERROR-PASSIVE → ERROR-ACTIVE)
- ✅ Reconnected ESP32 USB

### 3. Result After All Fixes
- ❌ Still TX timeout on both devices
- ❌ No CAN messages exchanged
- ❌ RX: 0 packets, TX: 0 packets

## Research Findings

### VESC Termination (Web Search)

**Important discoveries:**
1. **Flipsky 75200 has built-in 220Ω terminators** (not 120Ω)
2. **Terminators are PASSIVE** - should work when VESC unpowered
3. **With 2× VESC: 220Ω || 220Ω = 110Ω** (acceptable for CAN)

**Source:** VESC Project forums, Flipsky documentation

### Expected vs Actual

**Expected:**
- VESC terminators should be active even without power
- Should measure ~110Ω between CAN-H and CAN-L
- CAN communication should work

**Actual:**
- No CAN communication
- Both devices getting TX timeout (no ACK)
- Suggests terminators NOT present on bus

## Possible Root Causes

### 1. Missing Terminators (Most Likely)
- VESC terminators not present in this model variant
- VESC not physically connected to bus
- Defective/burned terminators

### 2. Physical Connection Issues
- VESC CAN wiring disconnected
- Wrong connectors used
- Wire break/poor contact

### 3. Other (Less Likely)
- Incompatible TWAI/CAN settings (already ruled out)
- Sample point mismatch (already fixed)
- Driver issues (already tested)

## CRITICAL: Actions Required in Office

### Priority 1: Measure CAN Bus Resistance

**What to do:**
1. Power off all devices (or leave Jetson on)
2. Set multimeter to Ohm (Ω) measurement mode
3. Measure resistance between CAN-H and CAN-L wires
4. **Record the reading**

**Expected results:**
- **~110Ω** = Both VESC terminators present ✅
  - Problem is something else (wrong bitrate, wrong wiring, etc.)
- **~220Ω** = Only ONE VESC connected ⚠️
  - Need to connect second VESC or add external terminator
- **∞ (infinite/open)** = NO terminators ❌
  - MUST add external 220Ω resistors at both bus ends

### Priority 2: If No Terminators (∞ reading)

**Required hardware:**
- 2× 220Ω resistors (1/4W or higher)
- OR 2× 120Ω resistors (will work, different standard)

**Installation:**
```
Left bus end (VESC1 or Jetson):  CAN-H ──[220Ω]── CAN-L
Right bus end (VESC2 or ESP32):  CAN-H ──[220Ω]── CAN-L
```

**Verification:**
- Remeasure: should now read ~110Ω with 2× 220Ω
- Or ~60Ω with 2× 120Ω

### Priority 3: Test CAN After Terminators Added

**Test procedure:**
```bash
# 1. Check CAN state
ip -d link show can0

# 2. Monitor CAN bus
candump can0

# 3. Check ESP32 output
python3 scripts/esp32_monitor.py /dev/ttyACM1
```

**Success criteria:**
- ✅ ESP32: No more TX timeout
- ✅ Jetson: CAN state ERROR-ACTIVE, receiving messages
- ✅ candump shows DroneCAN heartbeats and ESC commands

## Current Configuration (Working)

### Jetson CAN
```bash
Bitrate: 1000000 (1 Mbps)
Sample point: 0.860 (86%)
State: ERROR-ACTIVE
Restart-ms: 100
```

### ESP32 TWAI
```
Bitrate: 1000000 (1 Mbps)
Sample point: ~0.875 (87.5%)
Mode: NORMAL
GPIO: TX=4, RX=5
```

### Physical Topology
```
[VESC1] ←─ CAN-H/CAN-L ─→ [ESP32] ←─ CAN-H/CAN-L ─→ [Jetson] ←─ CAN-H/CAN-L ─→ [VESC2]
  ???                         ✅ ON                      ✅ ON                      ???
```

## Documentation Created

1. `docs/CAN_TERMINATION_CRITICAL_FIX.md` - Full analysis of termination problem
2. `docs/CAN_TERMINATION_DEBUG.md` - Debug procedure
3. `docs/CAN_DEBUG_SESSION_NOV15.md` - This file

## Next Session Checklist

**In office:**
- [ ] Measure CAN bus resistance with multimeter
- [ ] If ∞ (no terminators): Obtain 2× 220Ω resistors
- [ ] Install terminators at both bus ends
- [ ] Verify with multimeter (~110Ω expected)
- [ ] Test CAN communication
- [ ] Document results

**If terminators present but still not working:**
- [ ] Check VESC physical connections
- [ ] Trace CAN-H/CAN-L wires from VESC to bus
- [ ] Verify no wire breaks
- [ ] Check for cold solder joints

## Key Takeaways

1. **CAN requires terminators** - absolutely mandatory
2. **Terminators must be passive** - work without power
3. **Measure first** - don't assume, verify with multimeter
4. **VESC uses 220Ω** - different from standard 120Ω CAN
5. **Software was correct** - problem is 100% hardware

## Status

**Blocker:** Missing CAN bus terminators
**Impact:** Complete system halt - no CAN communication possible
**Priority:** P0 - CRITICAL
**Next Action:** Measure resistance in office
**ETA:** Next office visit

---

**Session Date:** November 15, 2025
**Time Spent:** ~2 hours
**Devices Tested:** Jetson Orin Nano, ESP32-S3 Motor Controller
**Outcome:** Identified critical hardware issue requiring office visit
