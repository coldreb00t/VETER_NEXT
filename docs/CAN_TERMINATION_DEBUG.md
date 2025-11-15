# CAN Termination Debug - November 15, 2025

## Research Findings

### VESC 220Ω Termination (from web search)

**Confirmed:**
- Flipsky 75200 has **built-in 220Ω terminators**
- Terminators are **PASSIVE** components (always active)
- Work **even when VESC unpowered**
- Internal to VESC (not removable without disassembly)

### Expected Resistance

With 2× VESC (each 220Ω in parallel):
```
R_total = (220Ω × 220Ω) / (220Ω + 220Ω) = 110Ω
```

## Current Problem

Despite VESC having built-in terminators that should work when unpowered:
- ❌ Jetson: ERROR-PASSIVE, TX timeout
- ❌ ESP32: TX timeout
- ❌ No CAN communication

## Possible Causes

1. **VESC not physically connected to bus**
   - Wires disconnected?
   - Wrong connectors?

2. **Defective terminators in VESC**
   - Burned out?
   - Never installed in this model variant?

3. **Measurement needed**
   - Measure actual resistance between CAN-H and CAN-L
   - Should read ~110Ω if both VESC terminators present

## Debug Steps

### Step 1: Measure CAN bus resistance

```bash
# Power off ALL devices
# Using multimeter:
# Measure between CAN-H and CAN-L at ANY point on bus
```

**Expected values:**
- ~110Ω = Both VESC terminators working ✅
- ~220Ω = Only one VESC terminator ⚠️
- ∞ (infinite) = No terminators ❌

### Step 2: Check VESC connections

Verify VESC are actually connected to the bus:
- VESC1 CAN-H/CAN-L → to bus
- VESC2 CAN-H/CAN-L → to bus

### Step 3: If no terminators detected

Add external 220Ω resistors:
```
Left end:  CAN-H ──[220Ω]── CAN-L
Right end: CAN-H ──[220Ω]── CAN-L
```

## Next Actions

**IMMEDIATE:**
1. Measure resistance between CAN-H and CAN-L with multimeter
2. Report result

This will definitively show if VESC terminators are present.
