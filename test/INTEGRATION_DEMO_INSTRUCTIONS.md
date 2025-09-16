# BuzzCar Simple Hardware Demo - Instructions

## Simple Hardware Test

Created a **simple hardware demo** that focuses on the motor and sensors for integration testing.

## What the Demo Tests

**Real-time sensor readings** from all 3 photosensors  
**Basic motor control** - OFF and 50% speed alternating every 3 seconds  
**160MHz CPU frequency** setting  
**9600 baud serial communication**  

## SETUP
# GPIO Mapping
Pin mappings can be found in GPIOManager.hpp, updates can be done here if needed
# Layout of Pins
A layout of pins can be found under test folder


## How to Run the Demo

### Step 1: Upload the Demo Program
# In the project folder, run:
platformio run -e hardware_demo --target upload


### Step 2: Open Serial Monitor  

# Using PlatformIO
platformio device monitor -e hardware_demo

**Settings:** 9600 baud rate

### Step 3: Watch the Output
Sample output showing:
```
BuzzCar Hardware Demo
====================
CPU Frequency: 160 MHz
Initializing hardware...
Motors initialized
Sensors initialized
Starting demo...

Motors: ON (50% speed)

Sensors: A=2341  B=892  C=1654  |  Motors: 50%
Sensors: A=2350  B=885  C=1649  |  Motors: 50%
Sensors: A=2345  B=890  C=1652  |  Motors: 50%

Motors: OFF

Sensors: A=2341  B=892  C=1654  |  Motors: OFF
Sensors: A=2350  B=885  C=1649  |  Motors: OFF
```


## Expected Output

The demo alternates every 3 seconds between:
- **Motors OFF**: No motor sound, status shows "OFF"  
- **Motors 50%**: Motor sounds at half speed, status shows "50%"

