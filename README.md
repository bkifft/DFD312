# DFD312
Firmware for the eStim MK312BT hardware
The MK312 project has delivered a nicely documented and complete hardware desctiption for building a eStim device inspired by the famous ET312. Firmware in binary format exists and works. This allows the community to build such devices. Great work. I did miss however possibility to change the firmware, extend and modify it. I could not find anything on the Internet about this so I dicided to write it on my own. This repository contains all files needed to compile the firmware and load it to a MK312-BT hardware. It works with the Arduino IDE and is simply structured. Functionality is beeing increased over time. At the time of the first release the basic functionality of the MK312 firmware is implemented. 
- Complete menu user interface identical to MK312
- Basic stimmulation pattern (wave, stroke ..)

Missing so far is:
- Audio patterns
- remote link / blue tooth
- Phase patterns
- Self test during startup

To generate the patterns, hardware -and software timers are used. The basic pulse that is generated by switinch on -and off the gates of the respective MOS FETs is quite short. Measuring the puls of the original MK312 shows me durations in the range of 5us - 130us. To achieve this is the 8MHz ATMega16, I use the hardware timers T2 and T3 for generating the pulses of both channels. Other variations of the output signal are slower so that a software timer works. Besides, the number of HW timers is limited to two (T1 is generally used by the Arduino SW).  The setup with these timers works. However, some optimization might be needed. The original MK312 generates pulses as fast as 3us. In my current setup I only reach 7us. Faster is possible but it seems that overall output signals get a bit of a jitter. Not sure if that is noticable when connecting the electrodes to  the body. Output signal strength is calibrated to the original device. This output level is generated via the DAC LTC 1661. Obviously, having access to the source code this can easily be changed.

## Compiling and flashing
Compiling with the Arduino IDE is straight forward. Just put the .ino file and the .h file in a subdirectory of your sketchbook folder and name it like the name of the .ino file.
To flash it make sure that the fueses are set correctly. I use:

`avrdude -C  /usr/local/arduino-1.8.13/hardware/arduino/avr/bootloaders/gemma/avrdude.conf -c usbtiny -b 115200 -p m16 -e -U hfuse:w:0xDF:m -U lfuse:w:0xFF:m -U flash:w:DFD312V094.ino.hex`

## Bluetooth HC-05 configuring
The [MK-312BT](https://github.com/CrashOverride85/mk312-bt/tree/master/bluetooth_conf) nicely describes the initial configuring of the HC-05 module. However, running the provided ATMega16 binary did bring me uncertainties. [The program](https://github.com/CrashOverride85/mk312-bt/blob/master/bluetooth_conf/MK-312BT%20V1.2%20HC-05%20Initialization%20ATMEGA16.bin) (I would be interested in the source of this, if available) did run till the password config step and stopped there. As I was not sure about the state I configured my HC-05 manually with a USB/serial adaptor. See [this picture](https://github.com/MauiKano/DFD312/blob/main/Documentation/hc-05-config.jpg) for the wiring. Using a standard terminal adaptor the HC-05 can be put into command mode (press the button during power up) and the known AT commands can be executed:

```
picocom /dev/ttyUSB0 --baud 38400 --omap crcrlf --echo
AT
OK
AT+VERSION?
VERSION:3.0-20170601
OK
AT+NAME=DFD-312BT
OK
```

Regarding the polarity of pin6 the description in MK-312BT seems wrong. I use
```
AT+POLAR=0,0
OK
```

which turns the 'radio LED' on once the HC-05 module is pluged in.
Also to be noted is, that the HC-05 runs only bluethooth v2 which is not supported by iOS, e.g. bluetooth with the MK-312BT and the firmware provided in this repository will not work with an iPhone.
After configuring the HC-05 several times by hand I finally understood how to programm it and how to use the UART interface of the ATMega16. The result is a BTConfig option in the 'More Options' menu.

