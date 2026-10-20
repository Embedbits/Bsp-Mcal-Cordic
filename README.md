# CORDIC Peripheral Driver

This module provides an abstraction layer for the **CORDIC coprocessor** (`CORDIC`) of STM32 MCUs: the hardware evaluation of the
trigonometric and hyperbolic functions - sine, cosine, arctangent, the modulus of a vector, the hyperbolic functions, the natural
logarithm and the square root - in the fixed-point format, in a few tens of clock cycles instead of the thousands of cycles of the
software library. It offers a single calculation, a calculation of an array by the DMA and the conversion helpers between the floating
point and the fixed-point values. It is a low-level MCAL module: it owns the CORDIC hardware, the application and the other modules never touch the registers.

The public interface (`Cordic_Port.h`, `Cordic_Types.h`) is **identical for all supported families**. The implementation
for a family lives in its own branch (`Dev/<Family>` for development, `Releases/<Family>` for the released versions), so
a module or an application written against the interface does not change when the MCU family changes.

This README is the landing page and the **specification of the interface** (master). The family branches contain their
own README with everything that is specific to the family: the instances of the device, their settings and the status of
the implementation.

> **Status: design.** No implementation exists yet, the interface below is a proposal. Stories and branches follow.

---

## Intent and purpose

- The typical users are the **motor control** (the Park and the inverse Park transform need the sine and the cosine of the rotor angle at every
  PWM period, the angle of a back-EMF vector needs the arctangent), the **signal processing** (the magnitude and the phase of a complex
  sample, the generation of a sine), the **audio** and the **measurement** (the logarithm of a ratio). On a Cortex-M the software `sinf` / `atan2f` costs
  hundreds to thousands of cycles; the CORDIC does the same in about 30 cycles and in parallel with the CPU.
- The hardware works in **fixed point** (`q1.31` or `q1.15`, the angle normalized to `[-1, 1)` as a fraction of pi), with the programmable number of
  the iterations (the precision) and a scaling factor for the functions whose argument or result is outside of the range. The module makes this
  usable: it converts the application values to the format, selects the function, sets the precision and the scale and returns the result in the
  requested form. For the performance critical code the raw `q` interface without the conversions is available.
- The array calculation (`Cordic_Set_CalculateArray`) streams the inputs from a buffer and writes the outputs to another one **by the DMA**, with no CPU
  load: a table of the sines, the magnitude of a block of the complex samples.
- The module hides the differences between the families: the data width, the precision options and the DMA structure differ, the
  interface stays the same.

What the module is **not**: it does not do the floating-point math (the FPU and the `libm` do), it does not offer the functions beyond the ones the hardware
has (`tan`, `exp`, `pow` are composed by the application) and it does not manage the fixed-point scaling of an algorithm (the module only documents the ranges).

---

## Concept

| Item | Meaning |
|---|---|
| **Function** (`cordic_Function_t`) | `CORDIC_FUNC_COSINE`, `_SINE`, `_PHASE` (atan2), `_MODULUS`, `_ARCTANGENT`, `_HYPERBOLIC_COSINE`, `_HYPERBOLIC_SINE`, `_ARCTANGENT_HYPERBOLIC`, `_NATURAL_LOGARITHM`, `_SQUARE_ROOT`. |
| **Format** (`cordic_Format_t`) | `CORDIC_FORMAT_Q31` or `CORDIC_FORMAT_Q15` for the arguments and the results (packed two per word in the 16-bit format). |
| **Precision** | The number of the iterations (the unit of 4 cycles): more iterations - more accuracy and time (`CORDIC_PRECISION_1` ... `_15`). |
| **Scale** | The scaling exponent (0 ... 7) that extends the valid range of the argument of `atanh`, `ln`, `sqrt`, the hyperbolic functions and the modulus. |
| **Arguments** | One or two input values (the second one is the modulus for the sine / cosine, the y coordinate for the phase) and one or two results. |

Rules:

1. `Cordic_Set_Calculate( function, arguments, results )` starts the calculation, waits for the result (the hardware needs tens of cycles; the read of the
   result register stalls the bus until ready) and returns the results; it is the **zero overhead** blocking mode.
2. `Cordic_Set_CalculateArray( request )` is a request of the array of the inputs, the output array and the count; the DMA reads the inputs and writes the
   outputs; the complete callback reports the end. The same request and the same callbacks are used in the DMA and the interrupt mode.
3. The configuration (the function, the precision, the scale, the format) is changed only while no calculation is running; a change returns an error otherwise.
4. The helper functions `Cordic_Get_FromFloat`, `Cordic_Get_ToFloat` convert between `float` and the `q` format with the **range check** (a value outside
   of `[-1, 1)` after the scale is reported as an error instead of being wrapped).
5. The valid ranges per function are documented in the interface (for example the arguments of `ln` and `sqrt` are limited to a part of the range that depends on the scale,
   as the reference manual tabulates); an out-of-range argument returns an error where the module can detect it.
6. All parameters are checked before a register is written.

---

## Supported families

Every family gets its own story, its own tasks and its own branches `Dev/<Family>` and `Releases/<Family>` cut from this
master.

| Family | Branch | Coprocessor | Status |
|---|---|---|---|
| STM32H5 | `Dev/STM32H5` | `CORDIC` on STM32H543, H553, H562, H563, H573 and the H5E / H5F lines: 24-bit rotation engine, circular and hyperbolic modes, rotation and vectoring, the ten functions, programmable precision, DMA read and write channels | planned (first) |
| STM32G4 | `Dev/STM32G4` | `CORDIC` | planned |
| STM32H7 | `Dev/STM32H7` | `CORDIC` on the lines that have it | planned |
| STM32U5, STM32U3 | `Dev/STM32U5`, `Dev/STM32U3` | device dependent | planned |
| STM32F4, STM32F7, STM32L4 | `Dev/STM32F4` ... | no CORDIC on the device - the module is a stub that returns `CORDIC_REQUEST_ERROR` | planned |

A function without a meaning in a family stays in the interface, returns `CORDIC_REQUEST_ERROR` and suppresses the unused
parameters (for example the 16-bit format on a hardware with the 32-bit only, a precision beyond the maximum of the device).

---

## Public interface in short (proposal)

| Group | Functions |
|---|---|
| Module management | `Cordic_Get_ModuleVersion`, `Cordic_Init`, `Cordic_Deinit`, `Cordic_Task`, `Cordic_Get_DefaultConfig` |
| Calculation | `Cordic_Set_Calculate`, `Cordic_Set_CalculateArray`, `Cordic_Get_State` |
| Settings | `Cordic_Set_Function` / `Get`, `Cordic_Set_Precision` / `Get`, `Cordic_Set_Scale` / `Get`, `Cordic_Set_Format` / `Get` |
| Conversion | `Cordic_Get_FromFloat`, `Cordic_Get_ToFloat` |
| Interrupts | `Cordic_Set_IrqPriority` / `Get`, the callbacks are a part of the configuration |

```c
cordic_ModuleVersion_t Cordic_Get_ModuleVersion  ( void );

cordic_RequestState_t  Cordic_Init               ( const cordic_Config_t * const cordicConfig );
cordic_RequestState_t  Cordic_Deinit             ( void );
void                   Cordic_Task               ( void );
cordic_RequestState_t  Cordic_Get_DefaultConfig  ( cordic_Config_t * const cordicConfig );

cordic_RequestState_t  Cordic_Set_Calculate      ( const cordic_Arguments_t * const arguments, cordic_Results_t * const results );
cordic_RequestState_t  Cordic_Set_CalculateArray ( const cordic_ArrayRequest_t * const request );

cordic_RequestState_t  Cordic_Set_Function       ( cordic_Function_t function );
cordic_RequestState_t  Cordic_Set_Precision      ( cordic_Precision_t precision );
cordic_RequestState_t  Cordic_Set_Scale          ( cordic_Scale_t scale );

cordic_RequestState_t  Cordic_Get_FromFloat      ( float value, cordic_Q31_t * const qValue );
cordic_RequestState_t  Cordic_Get_ToFloat        ( cordic_Q31_t qValue, float * const value );
```

Every function returns `CORDIC_REQUEST_OK` / `CORDIC_REQUEST_ERROR` and checks its parameters and the state of the module
before it touches a register.

| Type | Meaning |
|---|---|
| `cordic_Config_t` | Function, format, precision, scale, transfer mode (`CORDIC_XFER_MODE_POLL` / `ISR` / `DMA`), DMA channels, callbacks, interrupt priority |
| `cordic_Arguments_t`, `cordic_Results_t` | One or two values in the `q` format |
| `cordic_ArrayRequest_t` | The input array, the output array, the number of the calculations |
| `cordic_Q31_t`, `cordic_Q15_t` | The fixed-point types of the interface |

---

## Usage

### Sine and cosine of the rotor angle in the motor control loop

```c
#include "Cordic_Port.h"

cordic_Config_t    cordicConfig;
cordic_Arguments_t arguments;
cordic_Results_t   results;

(void)Cordic_Get_DefaultConfig( &cordicConfig );
cordicConfig.Function  = CORDIC_FUNC_COSINE;                  /* the cosine and the sine are both produced */
cordicConfig.Format    = CORDIC_FORMAT_Q31;
cordicConfig.Precision = CORDIC_PRECISION_6;
(void)Cordic_Init( &cordicConfig );

arguments.Value1 = rotorAngleQ31;                              /* angle / pi in [-1, 1) */
arguments.Value2 = CORDIC_Q31_ONE;                             /* the modulus: 1.0 */
(void)Cordic_Set_Calculate( &arguments, &results );            /* results.Value1 = cos, results.Value2 = sin */
```

### A table of the sines by the DMA

```c
const cordic_ArrayRequest_t table =
{
    .Input = angleTable, .Output = sineTable, .Count = 256u,
};

(void)Cordic_Set_Function( CORDIC_FUNC_SINE );
(void)Cordic_Set_CalculateArray( &table );                     /* the callback reports the end, no CPU load */
```

---

## Behaviour and limits

- **Fixed-point ranges.** The argument and the result are in `[-1, 1)` (the angles as a fraction of pi); the functions with the larger range need the
  scale; a value out of range is wrapped by the hardware - use the range-checked helpers.
- **Precision and the time.** Each precision step adds iterations (4 cycles each); the reference manual tabulates the accuracy against the precision. The module does not
  choose the precision for the application.
- **Bus stall.** Reading the result immediately after the start stalls the bus for the duration of the calculation; at the very high
  rates the application uses the interrupt or the DMA.
- **DMA data.** The array mode needs the arguments and the results in the format that the configuration says (two values per calculation for the
  functions with two arguments); the module checks the sizes.
- **Not part of the first version:** the chaining of the several functions in one request, the floating-point array interface (the conversion is done
  by the caller with the helpers), the automatic scale selection.

---

## Dependencies

- **Rcc** - peripheral clock and reset
- **Nvic** - interrupt
- **Gpdma** - DMA transfers of the arrays
- **RAL (Register Abstraction Layer)** - used internally to access the registers

All mandatory RCC, NVIC and DMA configurations are handled internally.

---

## Testing

- **Unit tests** (host, Unity + CMock + RegMem): the function, precision, scale and format register mapping, the conversion helpers (the ends of the range, rounding, the out-of-range
  rejection), the rejected changes during a calculation, the array request flow, the callbacks.
- **Integration tests** (target, probe-rs): the results of every function compared with the double-precision `libm` over the valid ranges (the error
  against the precision setting), the cycle count against the software library with the DWT, the DMA table generation.

---

## Interface conventions

The module follows the common rules of the MCAL layer, so that it behaves like the other modules:

- **Lists instead of free members.** Everything that the hardware restricts (the pins, the kernel clock sources, the DMA
  channels, the triggers and the other selectable sources) is a `typedef enum` list with exactly the items that are valid
  for the device line; a pin is an item `CORDIC_<SIGNAL>_PIN_<INSTANCE>_P<port><pin>` and there is an `*_UNUSED` item. A choice
  that is not in the list cannot be written.
- **Own peripheral only.** The module calls only the registers / LL functions of its own peripheral. The clocks, the
  interrupts, the pins, the DMA and the EXTI lines go through the API of the Rcc, Nvic, Gpio, Gpdma and Exti modules; a
  missing function is added to that module, it is not bypassed.
- **Init verifies itself.** `Cordic_Init` checks the result at the end; when it is not as required, the module is
  deinitialized and an error is returned. The waiting for the hardware is a loop on the register state with a timeout,
  never a blind delay.
- **Transfer modes.** Where the module moves data, the POLL (served by `Cordic_Task`), ISR and DMA modes have the same
  configuration structure, the same request and the same callbacks (error, half transfer, transfer complete).
- **Flags and states are enums of the module** (`cordic_FunctionState_t`, `cordic_FlagState_t`), the limits and the constants are
  named, there are no magic numbers in the interface.
- **Unsupported functions stay.** A function without a meaning in a family keeps its signature, suppresses the unused
  parameters and returns `CORDIC_REQUEST_ERROR`.
- **Documentation.** Every function and type has a Doxygen description, the main header has the `\defgroup`.

---

## Repository layout and branches

The master holds this README and the license. The family branches (`Dev/<Family>`, `Releases/<Family>`) hold:

| Path | Content |
|---|---|
| `README.md` | README of the family: the instances of the device, specifics, errata, status |
| `Cordic_Port.h`, `Cordic_Types.h` | Public interface (identical in all families) |
| `Cordic*.c`, `Cordic.h` | Implementation of the family |
| `CMakeLists.txt` | Library `Cordic_Lib` of the EmBi platform |
| `Tests/UnitTests` | Unit tests (Unity + CMock + RegMem) |
| `Tests/IntegrationTests` | Integration tests on the target (probe-rs) |

A change of the public interface is made in all families.

---

## CMake integration

```cmake
target_link_libraries( User_Lib PRIVATE Cordic_Lib )
```

1. Link `Cordic_Lib` to your CMake library.
2. Include `Cordic_Port.h` in your project.
3. Configure the module as needed for your hardware.

---

## License

This project is licensed under the **Creative Commons Attribution-NonCommercial 4.0 International (CC BY-NC 4.0)**.

You are free to use, modify, and share this work for **non-commercial purposes**, provided appropriate credit is given.

See [LICENSE.md](LICENSE.md) for full terms or visit [creativecommons.org/licenses/by-nc/4.0](https://creativecommons.org/licenses/by-nc/4.0/).

---

## Authors

- **Mr.Nobody** - [embedbits.com](https://embedbits.com)

Contributions are welcome! Please open a pull request.
