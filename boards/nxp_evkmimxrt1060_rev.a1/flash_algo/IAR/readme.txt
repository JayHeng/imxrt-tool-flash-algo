
DESCRIPTION
===========

Flash loader iMXRT595 FlexSPI
Uses iMXRT595 ROM APIs functions
These APIs only support 1 single FLASH device connected to PORTA on FlexSPI

CONFIGURATION
=============

The project contains configuration:

- FlashIMXRT500_MX25U
Builds a flash loader for iMXRT595
Uses 256KB RAM.
Output file:FlashIMXRT500_MX25U.out
Macro file: FlashIMXRT500_MX25U.mac
USAGE
=====
To specify the connected memory use the
flash loader with one of the following args:
"--Qspi"      - FlexSPI NOR - Quad SDR Read (same if no arg is used)
"--QspiDDR"   - FlexSPI NOR - Quad DDR Read
"--Hyper1V8"  - HyperFLASH 1V8
"--Hyper3V0"  - HyperFLASH 3V0
"--MxicOpiDDR"-  MXIC OPI DDR (OPI DDR enabled by default)
"--MxicOct"-  MXIC Octal DDR
"--McrnOct"   -  Micron Octal DDR
"--McrnOpi"   -  Micron OPI DDR
"--McrnOpiDDR"- Micron OPI DDR (DDR read enabled by default):
"--AdstOpi"   - Adesto OPI DDR
"--Opt0 0x..."- set Option0 hex value
"--Opt1 0x..."- set Option1 hex value
For more info refer to IMXRT600 ROM API specification