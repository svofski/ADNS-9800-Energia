# ADNS-9800-Energia

This is a code to test and evaluate ADNS-9800 laser optical mouse sensor. It was created in Energia and tested on a
Frauchpad MSP430FR5739 board, but any Arduino-like board should work just fine. 

This sketch is based on the [original] (https://github.com/mrjohnk/ADNS-9800) code
by [mrjohnk] (https://github.com/mrjohnk). 

Factory settings put the sensor in a mode that's
optimal for mouse use, but not necessarily good for use in robotics. 
This sketch sets it up to maximum reasonable framerate and resolution. The parameters are documented 
and should be easy to adjust.

There's also a terminal-based mousing toy included which may be useful for evaluation of sensor's capabilities. 
To use it you need a real terminal emulator that
supports VT102/ANSI escape sequences. Large window size is preferrable.

ADNS-9800 Firmware bytecode is Copyright Avago Technologies.
