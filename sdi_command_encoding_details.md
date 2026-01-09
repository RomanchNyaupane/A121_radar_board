# SDI Command Protocol - Encoding Reference

## Overview

This document describes the SDI (Sensor Data Interface) command encoding scheme used to efficiently decode and execute variable-length ASCII commands on embedded systems. The system converts human-readable commands into compact 32-bit instruction codes. The hex based approach is for removing the text processing overhead. The hex based approach trades code readability with less amount of processing.

## Command Structure

All SDI commands follow the pattern:

```
[address][command][modifier][number]
```

Where:
- **address**: Single character sensor address ('a', 'b', etc.) or '?' for query
- **command**: Single character operation code
- **modifier**: Optional single character modifier
- **number**: Optional single digit (0-9)

## 32-bit Encoding Format

```
0xAABBCCDD
│   │  │  └── DD: Flags and numeric parameters
│   │  └───── CC: Sub-command/modifier (ASCII << 8)
│   └──────── BB: Main command (ASCII << 16)
└──────────── AA: Address/query prefix (ASCII << 24)
```

### Bit Layout Details

```
31          24 23          16 15           8 7            0
┌──────────────┬──────────────┬──────────────┬──────────────┐
│    Address   │  Main Cmd    │  Sub-Cmd     │  Flags/Num   │
│  (0x61/0x3F) │   (ASCII)    │   (ASCII)    │              │
└──────────────┴──────────────┴──────────────┴──────────────┘
```

**Flag/Number Byte (DD):**
```
Bit 7   6   5   4   3   2   1   0
┌───┬───┬───┬───┬───┬───┬───┬───┐
│   │   │   │L4 │   │   │   │Num│  For Length 4 commands
└───┴───┴───┴───┴───┴───┴───┴───┘
    └───────┬───────┘   └───┬───┘
        Length 4    Numeric value
           flag        (0-9)
```

## Command Reference Table

### Length 1 Commands

| ASCII | Hex Encoding | Description |
|-------|--------------|-------------|
| `a!` | `0x61000000` | Acknowledge sensor address |
| `?!` | `0x3F000000` | Query all sensor addresses |

### Length 2 Commands

| ASCII | Hex Encoding | Description |
|-------|--------------|-------------|
| `aA` | `0x00410000` | Change sensor address |
| `aM` | `0x004D0000` | Start measurement |
| `aV` | `0x00560000` | Additional verification |
| `aC` | `0x00430000` | Additional concurrent measurement |
| `aR` | `0x00520000` | Continuous measurement |

### Length 3 Commands

#### Non-Numeric (Pure Letters)

| ASCII | Hex Encoding | Description |
|-------|--------------|-------------|
| `aMC` | `0x004D4300` | Start measurement with CRC request |
| `aCC` | `0x00434300` | Concurrent measurement with CRC request |

#### Numeric (Ends with 0-9)

| Pattern | Base Encoding | Final Encoding | Description |
|---------|---------------|----------------|-------------|
| `aDx` | `0x00441000` | `0x00441x00` | Send data (x = 0-9) |
| `aMx` | `0x004D1000` | `0x004D1x00` | Additional measurement (x = 0-9) |

**Note:** Numeric length-3 commands set bit 12 (`0x00001000`) as a flag.

### Length 4 Commands

All length-4 commands set bit 4 (`0x00000010`) as a length flag.

| Pattern | Base Encoding | Final Encoding | Description |
|---------|---------------|----------------|-------------|
| `aMCx` | `0x004D4310` | `0x004D431x` | Additional measurement with CRC (x = 1-9) |
| `aCx` | `0x00431000` | `0x0043100x` | Additional concurrent measurement (x = 0-9) |
| `aCCx` | `0x00434310` | `0x0043431x` | Additional concurrent with CRC (x = 1-9) |
| `aRx` | `0x00521000` | `0x0052100x` | Continuous measurement (x = 0-9) |
| `aRCx` | `0x00524310` | `0x0052431x` | Continuous measurement with CRC (x = 0-9) |

## Decoding Algorithm

### Step-by-step Process

```c
1. Parse ASCII command into full_command structure
   - c1: First char (query '?' or address)
   - c2: Second char (main command)
   - c3: Third char (modifier or number)
   - c4: Fourth char (number for length-4)

2. Determine command length (1-4)

3. Apply encoding rules:
   - Length 1: Use 0x61000000 or 0x3F000000
   - Length 2: Keep only bytes 1-2 (mask 0x00FF0000)
   - Length 3:
     * If c3 == 'C': Keep bytes 1-3 (mask 0x00FFFF00)
     * Otherwise: Keep bytes 1-2 + set bit 12 (0x00001000)
   - Length 4: Keep bytes 1-3 + set bit 4 (0x00000010)
```

### Code Example

```c
// Encoding examples for different command types:
"a!"    → 0x61000000  // Length 1
"aM"    → 0x004D0000  // Length 2
"aMC"   → 0x004D4300  // Length 3 non-numeric
"aD5"   → 0x00445000  // Length 3 numeric (aD5)
"aMC3"  → 0x004D4313  // Length 4 (aMC3)
```

## Execution Flow

### Switch-case Structure

The encoded instruction is used in a switch statement:

```c
switch (encoded_instr) {
    case CMD_ACKNOWLEDGE:         // 0x61000000
        // Handle a!
        break;
    case CMD_START_MEASURE:       // 0x004D0000
        // Handle aM
        break;
    case CMD_SEND_DATA_BASE:      // 0x00441000
        // Handle aD0-aD9 (check c3 for number)
        break;
    // ... etc
}
```

### Parameter Extraction

For commands with numeric parameters:

```c
if (instr & FLAG_LENGTH_3_NUM) {
    // Length 3 numeric: aDx, aMx
    param = command->c3 - '0';
} else if (instr & FLAG_LENGTH_4) {
    // Length 4: aMCx, aCCx, etc.
    param = command->c4 - '0';
}
```

## Special Cases and Edge Conditions

### Address Validation
- Invalid addresses are stored as '0' in parsing
- Execution checks `__address_mapper()` and cancels if unknown

### Measurement Blocking
- System prevents new sensor requests if previous not completed
- Same sensor can repeat measurement command (aM! after aM!)

### CRC Flag Handling
- Commands ending with 'C' enable CRC calculation
- `aMC`: Base measurement with CRC
- `aMCx`: Additional measurement x with CRC

## Benefits of This Encoding Scheme

### 1. Efficiency
- Single 32-bit comparison replaces string parsing
- Minimal memory usage (no string storage)
- Fast execution with direct switch statement

### 2. Extensibility
- Clear pattern for adding new commands
- 8-bit space for numeric parameters (0-255 possible)
- Room for additional flags in unused bits

### 3. Maintainability
- Self-documenting hex values
- Clear mapping between ASCII and encoding
- Easy to add debug logging

### 4. Type Safety
- Compiler can check enum values
- Clear separation of command types

## Future Expansion

### Available Bit Space

```
Unused bits available for expansion:
- Bits 24-31: Only 0x61 ('a') and 0x3F ('?') used
- Bits 13-15: Currently unused
- Bits 5-7: Available in flag byte
```

### Possible Extensions
- **Extended addressing**: Use bits 24-31 for >16 sensors
- **Extended parameters**: Use multiple bytes for larger numbers
- **Additional flags**: Error reporting, priority levels, timing controls

## Debugging Tips

### Common Issues
- **Bit masking errors**: Ensure correct masks for each length
- **Numeric conversion**: ASCII '0' = 0x30, subtract '0' for value
- **Length confusion**: Verify command length before decoding

### Debug Output

```c
printf("CMD: %c%c%c%c → 0x%08X\n", 
       cmd->c1, cmd->c2, cmd->c3, cmd->c4,
       encoded_instr);
```

## Appendix: ASCII to Hex Reference

```
'!' = 0x21    '0'-'9' = 0x30-0x39
'?' = 0x3F    'A'-'Z' = 0x41-0x5A
'a' = 0x61    'C' = 0x43
'D' = 0x44    'M' = 0x4D
'R' = 0x52    'V' = 0x56
```

---

This encoding scheme provides an optimal balance between human-readable commands and machine-efficient execution for embedded SDI protocol handling.
