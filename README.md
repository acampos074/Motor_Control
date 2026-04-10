# FOC Motor Controller — STM32F446RE

Field-Oriented Control (FOC) firmware for a 3-phase BLDC motor, running on an STM32F446RE at 180 MHz. Designed for precision servo applications with real-time current control, encoder feedback, and CAN communication.

---

## Hardware

| Peripheral | Function |
|---|---|
| STM32F446RE (180 MHz) | Main MCU |
| TIM1 — Center-aligned PWM @ 40 kHz | 3-phase motor commutation (PA8/PA9/PA10) |
| ADC1/2/3 — Triple mode, 12-bit | Phase currents ia, ib + bus voltage |
| SPI3 + DMA (PC10–12, PA15) | 14-bit absolute encoder (HES) |
| SPI2 (PB12–15) | DRV8323 gate driver |
| CAN1 @ 1 Mbps (PB8/PB9) | External motor commands |
| USART2 (PA2/PA3) | Debug serial menu |
| DAC1/2 | Oscilloscope debug outputs |

**Motor parameters**: 7 pole pairs · 24 V bus · 40 A max · KT = 0.0217 Nm/A · 12:1 gear ratio · Ld = Lq = 567 µH

---

## Control Loop (40 kHz ISR)

Runs every 25 µs in the `TIM1_UP_TIM10_IRQHandler`:

1. **ADC sample** → phase currents ia, ib; ic = −(ia + ib)
2. **HES read (SPI DMA)** → 14-bit raw position → mechanical angle → electrical angle (× 7 pole pairs)
3. **Clarke + Park transform** → id, iq in rotating DQ frame
4. **PI current controllers** → vd_cmd, vq_cmd (with anti-windup + vector magnitude limiting)
5. **Inverse Park transform** → phase voltages u, v, w
6. **Space Vector Modulation (SVM)** → duty cycles with DC offset injection
7. **Write TIM1 CCR1/2/3** → PWM output to gate driver

**Current control bandwidth**: ~125 Hz at 40 kHz sampling

---

## Control Modes

Selectable via serial terminal or CAN command:

| Key | Mode | Description |
|---|---|---|
| `k` | Calibration | Characterizes encoder nonlinearity; generates 128-entry correction LUT |
| `p` | Open-loop test | Diagnostic rotation without closed-loop feedback |
| `f` | Voltage FOC | External vq_cmd via CAN; used for system identification |
| `i` | System ID | Constant 0.2 V injection for 10 ms; logs current step response |
| `m` | Position control | PD outer loop → iq_ref → current loop |
| `w` | Speed control | P outer loop → iq_ref → current loop |
| `x` | Current control | Direct iq_ref command (torque control at full bandwidth) |
| `z` | Torque control | Torque command updated at 1 kHz instead of 40 kHz |

---

## File Structure

```
FOC_/
├── Core/
│   ├── Inc/
│   │   ├── FOC.h              # foc_t struct, motor parameters, FOC declarations
│   │   ├── FSM.h              # Finite State Machine declarations
│   │   ├── HES.h              # Hall Effect Sensor interface
│   │   ├── DRV.h              # DRV8323 gate driver declarations
│   │   ├── Calibration.h      # Encoder calibration declarations
│   │   ├── FOC_Math.h         # Fast sine/cosine lookup table declarations
│   │   └── ES_*.h             # Event-Services framework headers
│   └── Src/
│       ├── FOC.c              # Core FOC: dq0, uvw, svm, PI controllers
│       ├── FSM.c              # State machine, mode dispatch
│       ├── HES.c              # SPI DMA encoder reads, angle & velocity computation
│       ├── Calibration.c      # 896-point encoder nonlinearity mapping
│       ├── FOC_Math.c         # 512-point sine lookup table
│       ├── main.c             # Peripheral init and main loop
│       ├── stm32f4xx_it.c     # Interrupt handlers (TIM1 40 kHz entry point)
│       ├── tim.c              # TIM1 configuration
│       ├── adc.c              # ADC1/2/3 configuration
│       ├── can.c              # CAN1 configuration
│       ├── spi.c              # SPI2/3 configuration
│       ├── usart.c            # USART2 configuration
│       ├── EventCheckers.c    # FSM event detection
│       └── ES_*.c             # Event-Services framework implementation
├── Drivers/                   # STM32 HAL + CMSIS drivers
├── FOC_.ioc                   # STM32CubeMX project file
├── STM32F446RETX_FLASH.ld     # Linker script (Flash)
├── STM32F446RETX_RAM.ld       # Linker script (RAM)
└── README.md
```

---

## Key Data Structures

```c
// Main motor state (FOC.h)
typedef struct {
    float i_a, i_b, i_c;              // Phase currents [A]
    float i_d, i_q;                    // DQ frame currents [A]
    float iq_ref, id_ref;              // Current setpoints [A]
    float Sum_iq_error, Sum_id_error;  // PI integrators
    float vq_cmd, vd_cmd;             // Voltage commands [V]
    float v_u, v_v, v_w;             // Phase voltages [V]
    float dc_u, dc_v, dc_w;          // Duty cycles [0–1]
    float theta_elec, theta_mech;     // Angles [rad]
    float theta_dot_mech;             // Angular velocity [rad/s]
    uint16_t adc1_offset, adc2_offset;// Current sensor zero offsets
    uint8_t controller_flag;          // Enable/disable FOC loop
    float kp, kd;                     // PD gains (CAN-tunable)
} foc_t;
```

---

## Building

Developed in **STM32CubeIDE 1.10.1**. Open `FOC_.ioc` to regenerate HAL code via STM32CubeMX, or import the project directory directly into STM32CubeIDE.

**Toolchain**: GCC ARM Embedded  
**Optimization**: -O2 recommended for timing-critical 40 kHz loop

---

## Scaling / Calibration Constants

| Constant | Value | Description |
|---|---|---|
| `AMPS_PER_COUNTS` | 0.020142 A/count | ADC → current (40 V/V gain, 12-bit) |
| `VOLTS_PER_COUNTS` | 0.01289 V/count | ADC → bus voltage |
| `RADS_PER_COUNTS` | 3.835×10⁻⁴ rad/count | Encoder count → mechanical angle |
| `NPP` | 7 | Pole pairs (electrical = 7× mechanical) |
| `Fs` | 2249 (0x8C9) | TIM1 ARR — sets 40 kHz PWM frequency |
