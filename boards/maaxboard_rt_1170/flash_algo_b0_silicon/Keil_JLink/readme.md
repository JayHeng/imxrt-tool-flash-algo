# MaaxBoard RT
MaaxBoard is AVNET's 1GHz SBC, based on NXP imxrt1176.

## Features
- 1GHz SBC with advanced real-time capabilities
    - Use as a processing sub-assembly in OEM products or as a versatile development board 
    - Raspberry Pi-4B form-factor facilitates easier enclosure and cabling integration 
    - 40-pin HAT compatible expansion header (ie. access Pi HAT ecosystem add-on boards)
    - 18-pin custom expansion connector
-  Based on NXP i.MX RT1176 Crossover MCU
    - Cortex-M7 @1GHz and Cortex-M4 @400 MHz
- Board is well resourced with high-speed memories
    - On-chip: 2 MB RAM (TCM and OCRAM)
    - Onboard: 32 MB SDRAM 
    - Onboard: 32 MB HyperFlash
- Comprehensive set of board interfaces and peripheral devices
    - 40-pin Pi-HAT compatible expansion connector 
    - 18-pin custom expansion connector
    - 1x USB Host interface (type-A connector)
    - 1x USB Device interface (type-C connector)
    - 1x GbE Ethernet (with TSN)
    - 1x 10/100 Ethernet (with IEEE1588)
    - SWD/JTAG debugger 10-pin mini header (use NXP MCU-Link, or 3rd party debug probe)
    - MIPI-DSI touch display interface (2-lane, supports up to 1280x800 display) 
    - MIPI-CSI camera interface (2-lane, pinout is same as on Raspberry-Pi)
    - Audio subsystem with 4x onboard digital microphones plus stereo audio output jack
    - 802.11ac Wi-Fi and Bluetooth 5 
    - U.FL connected external antenna
    - High efficiency 5V to 3.3V dc/dc buck convertor, plus low-current low-voltage LDOs
    - Operating Temperature: -30~85°C
    - Dimensions: 85mm x 56mm (same as Raspberry-Pi 4)

## Flash Algo Use age
- A. Build This project using MDK.
- B. Find your J-Link driver installation directory, normally in `X:Program Files\SEGGER\JLinkxxx`.
- C. Copy the algorithm `MAAXBOARD_RT1176_HyperFlash_256KB_SEC.FLM` to  `\SEGGER\JLinkxxx\Devices\NXP\iMXRT_1176_MaaxBoard` directory.
- D. Modify `\SEGGER\JLinkxxx\JLinkDevices.xml` , add in contents downblew before `</DataBase>` tag:

    ```xml
    <Device>
        <ChipInfo      Vendor="NXP"
                    Name="MAAXBOARD"
                    WorkRAMAddr="0x20000000"
                    WorkRAMSize="0x00080000"
                    Core="JLINK_CORE_CORTEX_M7"
                    Aliases="MIMXRT1176xxx8_M7; MIMXRT1176xxxA_M7" />
        <FlashBankInfo Name="Hyper Flash"
                    BaseAddr="0x30000000"
                    MaxSize="0x02000000"
                    Loader="Devices/NXP/iMXRT_1176_MaaxBoard/MAAXBOARD_RT1176_HyperFlash_256KB_SEC.FLM"
                    LoaderType="FLASH_ALGO_TYPE_OPEN" />
    </Device>
    ```
- E. Now you can find a device which named "MAAXBOARD" in Jlink device list.

## Flash Algo Speedtest
> Full chip(32MB) R/W/Verify, tested on SWD 50Mhz.<br>
> Benchmark SW: JFlash.exe(v7.56), HW: J-Link V11<br>
> Payload: 0x00 - 0xff loop around, size = 32MBytes.

| project | benchmark(s)| speed(kB/s)|
|---------|-------------|------------|
| sector erase|142.765s|229.52kB/s|
| chip erase|118.135s|277.378kB/s|
| write|229.791s|142.60kB/s|
| read |102.456s|319.825kB/s|
| verify(CRC)|51.718s|633.59kB/s|


