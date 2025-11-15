# CAN Bus Termination - Critical Architecture Fix

**Date:** November 15, 2025
**Severity:** 🔴 CRITICAL
**Status:** ⚠️ BLOCKS SYSTEM OPERATION

## Problem Discovery

During initial testing of Jetson ↔ ESP32 CAN communication (November 15, 2025), discovered **complete CAN bus failure** preventing any communication between devices.

### Symptoms
- ESP32 Motor Controller: `[TWAI] TX timeout` - cannot transmit
- Jetson CAN: `ERROR-PASSIVE (berr-counter tx 128)` - cannot transmit
- Zero successful message exchange
- Both devices functioning correctly but unable to communicate

### Root Cause

**MISSING CAN BUS TERMINATION RESISTORS**

Original architecture assumed VESC 75200 has built-in 120Ω terminators that work when unpowered. This assumption was **INCORRECT**.

#### Why This is Critical

Without proper termination:
- Signal reflections on CAN bus
- Bit errors and transmission failures
- **COMPLETE SYSTEM FAILURE** - no communication possible
- Cannot send emergency stop commands
- Cannot control robot

## Incorrect Original Architecture

```
[VESC1] ←→ [ESP32] ←→ [Jetson] ←→ [VESC2]
(assumed    (no term)  (no term)   (assumed
 120Ω)                              120Ω)

❌ VESC terminators either:
   - Don't exist
   - Only active when VESC powered
   - Require configuration in VESC Tool
```

**Result:** With VESC unpowered → NO terminators → CAN DEAD ❌

## Correct Architecture (REQUIRED)

```
[VESC1]──┬─[CAN-H]─────[ESP32]─────[Jetson]─────[CAN-H]─┬──[VESC2]
         │                                                │
       [120Ω]                                          [120Ω]
         │                                                │
         └─[CAN-L]───────────────────────────────[CAN-L]─┘

Physical     (middle         (middle        (middle     Physical
end 1        of bus)         of bus)        of bus)     end 2
```

**Key Requirements:**
- **120Ω resistor between CAN-H and CAN-L at BOTH physical ends**
- **Passive resistors** (no power needed, always present)
- **Independent of any device state** (works even if VESC/Jetson/ESP32 fail)

## Technical Specifications

### CAN Bus Configuration (Verified Working)
- **Bitrate:** 1 Mbps
- **Sample point:** ~86% (Jetson) / ~87.5% (ESP32) ✅ Compatible
- **Termination:** 120Ω ± 10% at each end (MISSING - TO BE ADDED)

### Why 120Ω?

CAN bus impedance = 120Ω
Two 120Ω resistors in parallel at ends = 60Ω total ✅ Correct

### Current State (November 15, 2025)

**Physical Topology:**
```
VESC1 LEFT ←─ CAN-H/CAN-L ─→ ESP32 Motor Controller
                               ↓
                           CAN-H/CAN-L
                               ↓
                         Jetson Orin Nano
                               ↓
                           CAN-H/CAN-L
                               ↓
VESC2 RIGHT ←─ CAN-H/CAN-L ─→ (connects here)
```

**Current Status:**
- ❌ No terminator on VESC1 end
- ❌ No terminator on VESC2 end
- ✅ Wiring correct (CAN-H, CAN-L, GND)
- ✅ Sample points compatible
- ✅ Both devices transmitting (but failing due to no ACK)

## Required Fix

### IMPORTANT UPDATE (November 15, 2025)

**Research findings:** Flipsky 75200 VESC has **built-in 220Ω terminators** (not 120Ω)!

- VESC uses 220Ω for low-speed CAN (intentional design)
- Terminators are **passive** - should work even when VESC unpowered
- With 2× VESC: 220Ω || 220Ω = **110Ω total** (acceptable for CAN)

**Problem:** Despite built-in terminators, CAN not working!

**Possible causes:**
1. VESC not physically connected to bus
2. Defective terminators in specific units
3. Terminators not present in this model variant

### Debug Required

**FIRST:** Measure resistance between CAN-H and CAN-L with multimeter:
- ~110Ω = Both VESC terminators present ✅
- ~220Ω = Only one terminator ⚠️
- ∞ = No terminators ❌

### Hardware Required (if terminators missing)

- **2× 220Ω resistors** (to match VESC standard, 1/4W or higher)
- Alternative: **2× 120Ω resistors** (standard CAN, will work but different)

### Installation Locations

**Option 1 (Recommended):** On VESC connectors
```
VESC1 CAN connector:  CAN-H ──[120Ω]── CAN-L
VESC2 CAN connector:  CAN-H ──[120Ω]── CAN-L
```

**Option 2:** On device transceivers (if VESC inaccessible)
```
Jetson transceiver:   CAN-H ──[120Ω]── CAN-L  (left end)
ESP32 transceiver:    CAN-H ──[120Ω]── CAN-L  (right end)
```

### Installation Procedure

1. **Obtain resistors:** 2× 120Ω ±5% resistors
2. **Power off all devices**
3. **Install terminator 1:**
   - Solder/connect 120Ω between CAN-H and CAN-L at left bus end
4. **Install terminator 2:**
   - Solder/connect 120Ω between CAN-H and CAN-L at right bus end
5. **Verify with multimeter:**
   - Measure resistance between CAN-H and CAN-L
   - Should read ~60Ω (two 120Ω in parallel)
6. **Power on and test**

## Verification Test

After installing terminators:

```bash
# 1. Check CAN interface
ip -d link show can0
# Should show: state ERROR-ACTIVE (not ERROR-PASSIVE)

# 2. Monitor for ESP32 messages
candump can0
# Should see: DroneCAN heartbeats, ESC commands

# 3. Check ESP32 serial
python3 scripts/esp32_monitor.py /dev/ttyACM1
# Should see: Successful DroneCAN sends (no TX timeout)
```

**Expected Result:**
- ✅ ESP32: No more TX timeouts
- ✅ Jetson: CAN state ERROR-ACTIVE
- ✅ Messages exchanged successfully
- ✅ Both devices see each other on bus

## Impact on System Safety

### Before Fix (CRITICAL FAILURE)
- ❌ No CAN communication possible
- ❌ Cannot send emergency stop
- ❌ Cannot control motors
- ❌ Complete system paralysis
- ❌ **SAFETY CRITICAL FAILURE**

### After Fix
- ✅ CAN communication reliable
- ✅ Emergency stop functional
- ✅ Motor control operational
- ✅ Fail-safe architecture
- ✅ Works even if VESC unpowered

## Lessons Learned

1. **Never assume built-in terminators** - always verify
2. **CAN terminators must be passive** - independent of device power
3. **Test with unpowered devices** - verify fail-safe operation
4. **Physical layer first** - before debugging protocol/software
5. **Measure impedance** - verify 60Ω with multimeter

## Documentation Updates Required

- [x] Create this critical fix document
- [ ] Update `docs/CAN_SETUP.md` with termination requirement
- [ ] Update `CLAUDE.md` with termination warning
- [ ] Update hardware BOM with 120Ω resistors
- [ ] Add to `docs/DEVELOPMENT_STATUS.md` as blocker

## Action Items

**Priority: IMMEDIATE (P0)**

1. ⏳ **Order/obtain 120Ω resistors** (2 pieces minimum, 4 recommended for spares)
2. ⏳ **Install terminator on left bus end** (VESC1 or Jetson)
3. ⏳ **Install terminator on right bus end** (VESC2 or ESP32)
4. ⏳ **Verify with multimeter** (60Ω between CAN-H and CAN-L)
5. ⏳ **Test Jetson ↔ ESP32 communication**
6. ⏳ **Test Jetson ↔ ESP32 ↔ VESC (powered) communication**
7. ⏳ **Update all documentation**

**ETA:** Until resistors installed - **CAN bus NON-FUNCTIONAL**

## Related Documents

- `docs/CAN_SETUP.md` - CAN configuration guide
- `docs/JETSON_CAN_INTEGRATION.md` - Jetson CAN integration (Nov 14)
- `firmware/esp32_motor_controller/TWAI_SUCCESS.md` - ESP32 CAN success (Nov 10)
- `docs/DEVELOPMENT_STATUS.md` - Current project status

## Conclusion

This is a **critical architecture error** discovered during integration testing. The system **cannot operate** without proper CAN bus termination.

**Next session priority:** Install 120Ω terminators before any further testing.

---

**Discovery Date:** November 15, 2025
**Discovered By:** Integration testing (Jetson ↔ ESP32 communication test)
**Severity:** 🔴 P0 - BLOCKS ALL CAN OPERATIONS
**Status:** ⏳ AWAITING HARDWARE (120Ω resistors)
