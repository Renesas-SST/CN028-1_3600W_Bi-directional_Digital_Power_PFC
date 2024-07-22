# CN028-1 3600W Bi-Directional Digital Power PFC

# Description
3600W bi-directional AC/DC power supply, based on digital control, using a totem pole bridgeless PFC topology and a combination of SiC and Si MOSFETs to operate in continuous conduction mode (CCM) at high switching frequency.

# Installation
## Operation Conditions
- Microcontroller: RA6T2 (R7FA6T2BD3CFP)
- IDE: e2-studio v2023-01<br>
https://www.renesas.com/us/en/document/uid/e-studio-2023-01-installer-windows
- FSP version: 4.0.0<br>
https://github.com/renesas/fsp/releases/tag/v4.0.0
- Toolchain: GCC 10.3.1.20210824<br>
https://developer.arm.com/downloads/-/gnu-rm
- Emulator: J-Link / E2 / E2-Lite

## GUI
- CN028-1 Digital Power (PFC) Solution v1.2 (UART).exe
- CN028-1 Digital Power (PFC) Solution v1.2 (CAN).exe

### USB-UART Driver
https://www.silabs.com/developers/usb-to-uart-bridge-vcp-drivers<br>

### USB-CAN Driver
http://www.gcan.com.cn/3d05/222b/8f07/8d5f<br>
This driver is for the CAN-USB box (GCAN USBCAN-I Pro Bus Analyzer).<br>
<img src="./docs/pics/GCAN%20USBCAN-I%20Pro%20Bus%20Analyzer.jpg" alt="img" width="200">

# Usage
## Quick Start Guide
### PFC Application
#### Hardware Connection and GUI Setting
<img src="./docs/pics/hardware_connection (pfc mode).png" alt="img" width="360">
<img src="./docs/pics/gui_setting (pfc mode).png" alt="img" width="130">

#### Run Demo and Observe Measured Data
1. Power on by AC power supply. Open "CN028-1 Digital Power (PFC) Solution vx.x (UART).exe". Select the COM port and click on “Connect” button. Choose “PFC” application and set “VDC Target” to 400V. Click “WRITE”. Then click “RUN” button.
2. Get 400V on "CON1".<br>
<img src="./docs/pics/pfc result2.png" alt="img" width="360">
<img src="./docs/pics/pfc result1.png" alt="img" width="371"><br>
(Green: VDC; Pink: IAC; Blue: VAC)

Note: It is recommended to always disconnect the load before the system enters the RUN state of PFC. Connect the load until a stable 400VDC voltage is output to prevent the board from being an uncontrolled rectificaition state for a long time, which may cause heating and burning of components.

### Grid-off Inverter Application
#### Hardware Connection and GUI Setting
<img src="./docs/pics/hardware_connection (grid-off inverter).png" alt="img" width="360">
<img src="./docs/pics/gui_setting (grid-off inverter).png" alt="img" width="130">

#### Run Demo and Observe Measured Data
1. Power on by 400V DC power supply. Open “CN028-1 Digital Power (PFC) Solution vx.x (UART).exe”. Select the COM port and click on “Connect” button. Choose “Off-grid Inverter” application and set “FAC Target” to 50Hz, set “VAC Target” to 220V. Click “WRITE”. Then click “RUN” button.
2. Get 220V/50Hz on "CON2".<br>
<img src="./docs/pics/grid-off inverter result2.png" alt="img" width="360">
<img src="./docs/pics/grid-off inverter result1.png" alt="img" width="371"><br>
(Green: VDC; Pink: IAC; Blue: VAC)