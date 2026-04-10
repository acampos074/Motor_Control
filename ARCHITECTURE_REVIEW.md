# Architecture Review & Improvement Recommendations

> STM32F446RE FOC Motor Controller — reviewed 2026-04-09

Organized by priority: critical real-time issues first, then structural/design, then hygiene.

---

## Critical Issues

### 1. `printf` Inside the 40 kHz ISR

**File:** `Core/Src/stm32f4xx_it.c` — line 407

During `systemID_flag`, `printf` is called on every ISR tick at 40 kHz. A UART `printf` at typical baud rates takes far longer than the 25 µs ISR budget, causing overruns and missed commutation cycles.

**Recommendation:** Write data to a ring buffer inside the ISR and drain it from the main loop. The ISR should never block on I/O.

---

### 2. `double` Arithmetic on a Cortex-M4

**Files:** `Core/Src/FOC.c` line 308 (`static double time`), `Core/Inc/FOC.h` line 196 (`hes_t.counter`)

The M4 FPU only accelerates single-precision float. Every `double` operation falls back to software floating-point, roughly 10× slower than `float`. At 40 kHz this is measurable cycle budget waste.

**Recommendation:** Replace all `double` with `float`. There is no precision benefit here that justifies the cost.

---

### 3. PI Integrators Wind Up During Calibration / Open-Loop Modes

**File:** `Core/Src/FOC.c` — lines 215–219

`Sum_id_error` and `Sum_iq_error` are updated unconditionally on every call to `commutate()`, even when `cal_flag == 1` or `open_loop_test_flag == 1` (where the voltage output is forced to a fixed value). When the system transitions back to closed-loop, the integrators are pre-wound with stale error accumulated during open-loop operation, causing a current spike at mode entry.

**Recommendation:** Gate integrator updates behind the same condition that selects PI output. Integrators should only accumulate when the PI output is actually driving the plant.

---

## Structural / Architectural Issues

### 4. Eight Boolean Flags Instead of One Mode Enum

**Files:** `Core/Inc/FOC.h` lines 157–165, `Core/Src/FSM.c`, `Core/Src/stm32f4xx_it.c`

Control mode is encoded as 8 separate `uint8_t` fields (`controller_flag`, `voltage_FOC_flag`, `cal_flag`, `open_loop_test_flag`, `systemID_flag`, `pos_control_flag`, `speed_control_flag`, `current_control_flag`, `torque_control_flag`). These are mutually exclusive but nothing enforces it — multiple flags can be set simultaneously. Adding a new mode requires changes in the FSM, the ISR dispatch, `torque_control()`, and `reset_variables()`.

**Recommendation:** Replace with a single `control_mode_t` enum:
```c
typedef enum {
    MODE_IDLE,
    MODE_CALIBRATION,
    MODE_OPEN_LOOP_TEST,
    MODE_SYSTEM_ID,
    MODE_VOLTAGE_FOC,
    MODE_CURRENT,
    MODE_SPEED,
    MODE_POSITION,
    MODE_TORQUE,
} control_mode_t;
```
The ISR dispatch becomes a clean `switch (foc.mode)` with no ambiguity.

---

### 5. `foc_t` Is a God Struct

**File:** `Core/Inc/FOC.h` — lines 129–169

The struct packs motor state (currents, voltages, angles), sensor offsets, calibration offsets, CAN receive buffers, debug fields, PI integrator state, and all mode flags into one 200+ byte object. Every module touches it freely, creating implicit coupling between otherwise independent subsystems.

**Recommendation:** Split by responsibility:
- `motor_state_t` — currents, voltages, angles, velocity
- `pi_state_t` — integrators, error terms, gains
- Keep CAN receive data in `can.c` / a dedicated `can_rx_t`
- Move `hes_t` out of `FOC.h` into `HES.h` where it belongs

---

### 6. `DT` and `Fs` Are Independent Defines That Must Stay in Sync Manually

**File:** `Core/Inc/FOC.h` — lines 49, 103

`Fs = 0x08C9` sets the TIM1 ARR (hardware PWM period) while `DT = 0.000025f` must be manually set to match. The multiple commented-out gain sets for 25 kHz, 29 kHz, 33 kHz, 40 kHz confirm this is a recurring source of error — changing the frequency requires touching `Fs`, `DT`, and all four PI gains by hand.

**Recommendation:** Define a single `F_PWM` in Hz and derive everything from it:
```c
#define F_PWM       40000UL
#define Fs          (180000000UL / (2 * F_PWM))   // TIM1 ARR
#define DT          (1.0f / F_PWM)
```
PI gains depend on plant inductance and desired bandwidth, so they still need to be re-tuned per frequency, but at least `DT` and `Fs` can never diverge.

---

### 7. Blocking ADC Reads in the ISR

**File:** `Core/Src/FOC.c` — lines 130–142

`sample_ADC()` calls `HAL_ADC_Start()` and immediately reads the result with `HAL_ADC_GetValue()`. The ADC conversion must complete synchronously before the ISR can proceed, burning cycles in the tightest real-time path.

**Recommendation:** Trigger the ADC conversion at the end of one ISR tick and read the result at the start of the next (double-buffered), or use injected ADC triggered automatically by TIM1. This overlaps conversion time with computation and removes the blocking wait entirely.

---

## Design / Correctness Issues

### 8. Voltage Limiter Uses Hardcoded `V_BUS` Instead of Measured `foc->VM`

**File:** `Core/Src/FOC.c` — line 223

`v_max` is correctly computed from the measured bus voltage `foc->VM`, but the vector magnitude clamp on line 223 uses the constant `V_BUS = 24.0f`. At lower battery voltages the SVM will generate duty cycles the bus cannot actually deliver. The limiter and SVM should use `foc->VM` consistently throughout.

---

### 9. Encoder Linearization LUT Is Generated but Never Applied

**File:** `Core/Src/HES.c` — lines 94–99

The full LUT interpolation block is commented out. The calibration routine executes, populates `lut_arr`, and prints it over UART — but it is never applied to `theta_mech_raw` during normal operation. The calibration effort is currently wasted.

**Recommendation:** Re-enable the interpolation block. If LUT application is intentionally deferred, document why and add a `lut_enabled` flag so it can be toggled without commenting out code.

---

### 10. Two Nearly Identical Commutation Functions

**File:** `Core/Src/FOC.c` — lines 159–302

`commutate()` and `commutate_v2()` implement the same DQ0 → PI → SVM → PWM pipeline. `commutate()` adds mode-dependent branching inside (selecting between fixed-voltage open-loop, voltage FOC, system ID, and PI control). The ISR must decide which function to call, and each partially duplicates the other.

**Recommendation:** One `commutate()` that always executes the full pipeline. Mode logic belongs only in the layer that sets `iq_ref`, `id_ref`, `vd_cmd`, and `vq_cmd` before `commutate()` is called — not inside it.

---

### 11. Outer-Loop Rate Divisor Is a Magic Number Tied to Sample Rate

**File:** `Core/Src/stm32f4xx_it.c` — line 357

```c
if(foc.torque_control_counter > 40)  // use 40 if using 40kHz
```
Adjacent comments show values 25, 29, 33, 40 for different sample rates — set manually each time the frequency changes. This divisor should be:
```c
#define TORQUE_LOOP_HZ   1000
#define TORQUE_DIVIDER   (F_PWM / TORQUE_LOOP_HZ)
```
so it updates automatically when `F_PWM` changes.

---

## Code Hygiene

### 12. Large Volume of Commented-Out Code

Nearly every function carries 30–50% commented-out lines: old polling SPI reads, alternative DAC debug calls, dead experiment code, and obsolete implementations. This makes it difficult to trace the active execution path and understand intent.

**Recommendation:**
- Delete code that is not coming back
- Use `#if DEBUG_SCOPE` guards for DAC/oscilloscope debug lines you want to keep available
- Move notable experiments or prior implementations to git history via a well-described commit, then delete from source

---

### 13. CAN Telemetry TX Is Fully Disabled

**File:** `Core/Src/FSM.c` — lines 415–430

The `ES_CAN_RX` handler only clears `CANFlag` and prints. All CAN TX (position, velocity feedback to the host) is commented out. If closed-loop control from a host controller is the intended use case, the telemetry path needs to be completed.

---

## Summary

| Priority | Item | Impact |
|---|---|---|
| Critical | `printf` in 40 kHz ISR | ISR overrun, missed commutation |
| Critical | `double` arithmetic on M4 | ~10× slower than `float`, burns cycle budget |
| Critical | Integrator windup in open-loop modes | Current spike on mode transition |
| High | Flag soup → mode enum | Ambiguous state, fragile mode transitions |
| High | God struct `foc_t` | Tight coupling, hard to test subsystems |
| High | `DT`/`Fs` manual sync | Silent mismatch when changing frequency |
| High | Blocking ADC in ISR | Wastes cycles in tightest real-time path |
| Medium | Hardcoded `V_BUS` in limiter | Over-drives at low battery |
| Medium | LUT generated but not applied | Calibration has no effect on tracking |
| Medium | Two commutate functions | Duplicate logic, confusing dispatch |
| Medium | Magic outer-loop divisor | Breaks silently when changing `F_PWM` |
| Low | Commented-out code | Readability / maintainability |
| Low | CAN TX disabled | Missing telemetry path |
