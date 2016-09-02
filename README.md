# micromill

3-Axis CNC firmware for 28BYJ steppers with ULN2003 driver running on
AtMega328 like Arduino Nano, emulating the grbl communication protocol.

See Appendix A for supported G-commands.

Supported sender is [Universal G-Code Sender from Will Winder], 
(https://github.com/winder/Universal-G-Code-Sender) Classic Version, Nightly
build 29. Aug 2016 or later.

Developed in [Arduino/Genduino](https://www.arduino.cc/en/Main/Software] 1.6.11)

This firmware uses [TimerOne legacy library](https://github.com/PaulStoffregen/TimerOne)