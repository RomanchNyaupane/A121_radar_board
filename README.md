# STM32WB55 Multi-Sensor Firmware

Embedded firmware for STM32WB55 microcontroller interfacing with multiple sensors over I2C, SPI, and UART. Runs FreeRTOS with separate tasks for each peripheral.

## Hardware

- **MCU**: STM32WB55
- **IMU**: LSM6DSV16X (I2C) - orientation via sensor fusion
- **Radar**: Acconeer A121 (SPI1) - distance detection
- **Flash**: SPI flash memory (SPI2)
- **Charger**: BQ25180 (I2C) - battery management with ADC monitoring
- **RTC**: Internal RTC for timekeeping
- **Communication**: LPUART1 with RS-485/SDI-12 compatible protocol

## Project Structure

```
app/
├── main.c              - initialization, FreeRTOS tasks
└── stm32wbxx_hal_msp.c - peripheral MSP configuration

drivers/
├── inc/                - driver headers
└── src/
    ├── imu_driver.c    - LSM6DSV16X basic interface
    ├── charger_driver.c - BQ25180 control and ADC
    ├── spi_flash.c     - flash read/write/erase
    └── sdi_cli.c       - SDI-12-like command protocol

vendor/acconer/
├── acc_hal_integration_stm32cube_xe121_single_sensor.c
└── example_detector_distance.c - Acconeer distance detector
```

## Architecture

### Task Design

Each sensor runs in its own FreeRTOS task with fixed priorities. Tasks are largely independent but share data through global arrays that the SDI machine task reads.

- **imu_task**: Configures LSM6DSV16X for 30Hz SFLP (sensor fusion low power) mode using game rotation vectors. Reads quaternion data from FIFO when interrupt fires, converts to Euler angles. Stores roll/pitch/yaw in `imu_data[]` array.

- **radar_task** (A121_Task): Runs Acconeer's distance detector algorithm. Handles sensor calibration, measurement cycles, and processes results through a moving average filter. Updates `radar_data[]` with up to 3 distance/strength pairs.

- **charger_task**: Polls BQ25180 battery charger IC. Configures and triggers ADC conversions for VBAT, VSYS, VBUS, IBAT, IBUS, and battery temperature. Writes results to `charger_data[]`. Also handles charge interrupt events.

- **flash_task**: Initializes SPI flash but currently just delays. Originally intended for periodic data logging.

- **rtc_task**: Reads hardware RTC registers in BCD format every 100ms, stores in `rtc_data[]`.

- **sdi_task**: Handles incoming UART commands. Receives bytes via interrupt callback, accumulates them until '!' terminator, then calls the command parser.

- **sdi_machine**: Main protocol state machine. Checks API flags set by SDI commands, formats sensor data as ASCII strings with proper field width (`%+08.3f`), and calls the response transmitter.

- **led_task**: Toggles GPIO pins for visual feedback. Simple heartbeat.

### Data Flow

Sensor tasks → Global float arrays → SDI machine → ASCII formatting → UART transmission

The `sdi_cli.c` module bridges command reception to sensor control through API flags like `sdi_api_imu_start`. These flags act as mailboxes between the SDI state machine and sensor tasks. Not elegant but it avoids queues and mutexes.

### IMU Sensor Fusion

Uses LSM6DSV16X's built-in SFLP to generate game rotation vectors (quaternions without magnetometer). Configuration:
- 30Hz ODR for accel, gyro, and SFLP
- Quaternion stored as half-precision floats in FIFO
- FIFO watermark interrupt at 32 samples
- Gyro bias compensation enabled

The quaternion (qi, qj, qk) is read from FIFO and converted to single-precision. The scalar part (qw) is computed from the unit quaternion constraint: `qw = sqrt(1 - qi² - qj² - qk²)`.

Euler angles extracted using standard formulas with singularity handling:
```c
roll = atan2(2*(qw*qi + qj*qk), 1 - 2*(qi² + qj²))
pitch = asin(2*(qw*qj - qk*qi))  // clamped to ±π/2
yaw = atan2(2*(qw*qk + qi*qj), 1 - 2*(qj² + qk²))
```

### Flash Memory Management

Designed for sequential writes with automatic address wrapping. Maintains internal counters for active block/sector/page position.

**Write strategy**: 
- Single byte writes check if address matches expected position
- Batch writes handle page boundaries automatically, breaking transfers at 256-byte boundaries
- Address manager increments position after each successful write

**Erase logic**:
- Only allows erasing the current active sector or one sector behind
- `back_sector_erase_available` flag prevents accidental data loss
- Sector erase aligns address to 4KB boundaries (sector size)
- Sets page counter to 0 after erase

This prevents writing to random addresses (which would require explicit erase) while allowing circular buffer patterns. Not flexible but safer for the intended logging use case.

### SDI-12 Protocol Implementation

Not full SDI-12 spec, but close enough for sensor interrogation patterns.

**Command structure**: `[address][command][params]!`

Commands are parsed into a `full_command` struct then converted to a 32-bit hex identifier for switch-case execution. Length-based decoding separates command types:

- Length 1: Address acknowledge `a!` or query `?!`
- Length 2: Action commands `aM!`, `aV!`, `aC!`, `aI!`
- Length 3: Parameterized commands like `aM1!` (partial read), `aD0!` (send data)
- Length 4: Commands with CRC like `aMC1!`

The hex conversion trick avoids string comparisons. For example, `aMC!` becomes `0x004D4300` by packing ASCII bytes.

**State tracking**: Each sensor has a `sensor_status` struct tracking:
- Address (can be changed with `aAb!` command)
- Busy/idle state
- Data ready flag
- Error conditions
- Measured data buffer
- CRC value

The `config` sensor struct holds the current command state (blocking mode, CRC enabled, sub-reading index) which gets copied to the target sensor on `_sensor_set()`.

**CRC**: Uses CRC-16 with polynomial 0xA001 (reflected form of CRC-16-MODBUS). Calculated over the ASCII data payload, then encoded as 3 printable ASCII characters in the range 0x40-0x7F by packing 6-bit chunks.

**Response format**: `[address][time:3digits][count:1digit][data][CRC:3chars]\r\n`

Time field is hardcoded to 002 (not real timing). Count indicates number of values. Data uses fixed-width signed float format.

### Charger ADC Monitoring

BQ25180 has integrated ADC for monitoring battery parameters. Driver configures ADC control registers then triggers conversions:

```c
charge_adc_ctrl = 0xC0;       // Enable ADC
charge_adc_func_dis_0 = 0x02; // Disable unused channels  
charge_adc_func_dis_1 = 0xF0;
```

Reads 16-bit values from ADC result registers, converts to floats with appropriate scaling (mV/mA units). The `get_adc()` function updates all 6 charger parameters in one call.

Interrupt on PB1 signals charge events (connected/disconnected, fault conditions). Handler sets `charge_exti_flag` which charger task checks to read flag registers.

### Radar Distance Detection

Integrates Acconeer's RSS library which handles the heavy lifting. Our code provides HAL implementation for SPI transfers and GPIO control.

**Calibration sequence**:
1. Sensor calibration - characterizes the sensor/antenna system
2. Detector calibration - builds background model for CFAR threshold
3. Measurement loop with periodic recalibration when drift detected

Configuration uses balanced preset: 0.1-4m range, profile 5, CFAR thresholding with 0.5 sensitivity, 15dB signal quality requirement.

The `print_distance_result()` function applies a 10-sample moving average filter to smooth distance and strength readings before storing in `radar_data[]`. Helps reduce jitter from single-frame outliers.

## Current State

### What Works
- All sensors initialize and read data
- FreeRTOS tasks run independently 
- IMU quaternion to Euler angle conversion with singularity handling
- Radar distance detection with moving average smoothing
- Flash consecutive writes with automatic address management
- SDI-12 inspired command protocol over RS-485
- Battery ADC monitoring with interrupt support
- RTC time/date tracking
- CRC calculation and ASCII encoding
- Dynamic address assignment

### Known Issues
- IMU task has hardcoded delays, not using data-ready properly even though FIFO interrupt is configured
- SDI protocol implementation works but the nested switch cases and hex encoding make it hard to follow
- Flash driver address logic prevents legitimate random access, too restrictive
- No proper error handling in most places - functions return status but callers don't check
- Memory allocation checks exist but recovery just calls Error_Handler() and hangs
- CRC calculation works but ASCII conversion might not match SDI-12 spec exactly
- Task stack sizes are guesswork based on "it didn't crash"
- Interrupt priorities aren't carefully thought out, everything at priority 0
- Global data arrays have no protection mechanism, potential race conditions
- UART transmit uses blocking HAL calls with HAL_MAX_DELAY, can hang system
- Radar task never yields because Acconeer example has infinite loop
- No watchdog configured
- Flash task does nothing useful yet

## Communication Protocol

Basic SDI-12 style commands over LPUART1 (9600 baud):

```
?!              - query all sensor addresses
a!              - acknowledge address 'a'
aM!             - start measurement on sensor 'a'
aD0!            - send data from sensor 'a'
aMC!            - start measurement with CRC
aM1!..aM9!      - read specific value (e.g., only roll from IMU)
aMC1!..aMC9!    - read specific value with CRC
aAb!            - change address from 'a' to 'b'
```

Sensors:
- `a` - IMU (roll, pitch, yaw in degrees)
- `b` - Charger (vbat, vsys, vbus, ibat, ibus, temp)
- `c` - Radar (3x distance/strength pairs)
- `d` - Flash (placeholder)
- `e` - RTC (seconds, minutes, hours, date, month, year)

Response format: `a0021+010.023-045.670+123.450\r\n` (address + time + count + fixed-width data)

With CRC: `a0021+010.023-045.670+123.450@AB\r\n` (3-char CRC appended)

## Building

Uses STM32CubeIDE or standard ARM GCC toolchain. HAL and FreeRTOS libraries required. Acconeer RSS SDK included in vendor directory.

**Clock configuration**:
- 64MHz system clock from HSI-PLL (16MHz * 8 / 2)
- APB1 at 4MHz (affects I2C and LPUART timing)
- APB2 at 64MHz (SPI and USART1)
- LSI for RTC (32kHz internal oscillator)

**Memory**:
- Flash: 1MB (using ~50KB with current code)
- RAM: 256KB (FreeRTOS heap sized in FreeRTOSConfig.h)
- Tasks consume: ~8KB total stack space

## GPIO Mapping

| Pin | Function | Notes |
|-----|----------|-------|
| PA2 | A121 Enable | Active high |
| PA3 | A121 Interrupt | Rising edge |
| PA4 | A121 SPI CS | Software controlled |
| PA5-7 | SPI1 (Radar) | SCK, MISO, MOSI |
| PA9-10 | USART1 | TX, RX (debug, unused) |
| PB1 | Charger Interrupt | Falling edge |
| PB4-5 | IMU Interrupts | INT1 (accel), INT2 (gyro) |
| PB6-7 | I2C1 | SCL, SDA (IMU, charger) |
| PB13-15 | SPI2 (Flash) | SCK, MISO, MOSI |
| PC0-1 | LPUART1 | RX, TX (SDI protocol) |
| PC2 | RS-485 DE | TX enable |
| PC3 | Line Select | RS-485 vs SDI mode |
| PC4-6 | LED | Status indicators |

## Things That Need Work

- Proper RTOS synchronization between SDI task and sensor tasks (semaphores or queues instead of flags)
- Flash driver needs rewrite, the address logic is too convoluted and restrictive
- IMU should use FIFO interrupts correctly instead of polling with delays
- SDI command parser works but the switch-case forest is hard to extend
- Error recovery - don't just hang on allocation failure
- UART should use DMA or non-blocking transmit
- Battery charger interrupt handling needs actual fault response logic
- No power management or sleep modes configured
- Radar task should periodically yield to scheduler
- Test coverage is "it compiled and sort of works on my desk"
- Documentation for the SDI protocol deviations from actual SDI-12

## Design Decisions

**Why global arrays instead of queues?** Simpler and lower overhead for what's essentially a data publication pattern. Sensors write, SDI machine reads. No back-pressure needed.

**Why the hex encoding trick in SDI parser?** Avoids strcmp chains and makes the command dispatch O(1). Trades readability for efficiency.

**Why sequential-only flash writes?** Intended use case is circular logging. Preventing random writes simplifies erase management and reduces wear leveling complexity we don't need.

**Why FreeRTOS tasks per sensor?** Each peripheral has different timing requirements. Tasks keep them independent rather than polling everything in one loop.

**Why moving average on radar?** Single-frame distance measurements are noisy. 10-sample MA smooths without significant latency (333ms at 30Hz).

## Notes

The SDI protocol implementation diverges from actual SDI-12 spec in timing, response format, and CRC encoding. It follows the command structure but doesn't implement concurrent measurements, verification commands, or continuous measurement modes properly.

Flash writes are sequential only by design but the error reporting doesn't make that clear to callers. You'll get a status bit set but no indication of what address would have been valid.

Radar task never yields because the Acconeer example code has its own while loop. Works fine in practice since other tasks are lower priority, but not great RTOS citizenship.

GPIO assignments and peripheral configurations are split between the MSP file and main.c initialization. Yes, this should be consolidated.

The CRC16 implementation uses 0xA001 polynomial (MODBUS) but the ASCII encoding packs 6-bit chunks into printable range. Whether this matches SDI-12 spec exactly is unclear - it calculates *something* consistently.

LPUART baud rate is 9600 because that's what SDI-12 uses. Could go faster but no need for the data rates here.
