
DESCRIPTION
===========

Flash loader iMXRT685 FLEXSPI
Uses iMXRT685 ROM APIs functions
These APIs only support 1 single FLASH device connected to PORTA on FLEXSPI

CONFIGURATION
=============

The project contains configuration:

- FlashIMXRT600_EVK_FLEXSPI
Builds a flash loader for iMXRT685
Uses 256KB RAM.
Output file:FlashIMXRT600_EVK_FLEXSPI.out

USAGE
=====
To specify the connected memory use the
flash loader with one of the following args:
"--Qspi"      - QuadSPI NOR - Quad SDR Read (same if no arg is used)
"--QspiDDR"   - QuadSPI NOR - Quad DDR Read
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