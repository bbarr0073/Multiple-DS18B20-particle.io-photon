# Multiple-DS18B20-particle.io-photon
A sketch to run multiple ds18b20 devices on OneWire particle.io Photon
This sketch can be setup to run (1 to n) DS18B20 devices on a particle.io device. I used five. It is flexible in number of devices, sample times (90ms-whatever, or as fast as can be done), bit resolution (9-12bits), publishing max times, publishing min times (based on a defined temperature differential).

It is fast and does not depend on any delay functions except those built into the OneWire protocol. If you have 10 devices hooked up, you could get about 80 temperature conversions/second in 9-bit mode. This program has some built-in monitoring of CRC failures, that were used to find the source of the CRC errors. But with the fixed OneWire.cpp file, there are no errors for the Photon.

Through this project we discovered a bug in the particle.io implementation of OneWire. If you include the fixed “OneWire.cpp” file as part of your compile, and you have one of the “fixable devices”, you will get ZERO CRC errors once you setup your experiment to run correctly (hookups, pullup resistor, etc). The OneWire bus is actually extremely robust when not interrupted in the middle of bit transfers. Hopefully, particle.io will have some suggestions to fix this for ALL its devices in the near future. See this thread for more information:

https://community.particle.io/t/onewire-library-bug-proposed-fix-june-2019-affects-ds18b20-and-other-onewire-devices/50589/35

If you find this useful, let me know…I am kind of proud of my first real program that interacts with the environment (temp sensors), haha.
