# RTScheduler-Discovery
A realtime preemptive scheduler for the STM Discovery boards

This is the Core base directory. It contains the OS + a small blinking application.

To use it, create a directory for your application. Copy one of the applications OBDI (chevy tpi analyzer) or cec_gw.
In the makefile have the macro KREL point to a board in a base release diretory.

f.ex. KREL=../RTScheduler-Discovery/boards/MB997C

copy the config.h from the board type here, and update it wrg. drivers, io pins ...

to build it, an gcc arm compiler is needed...
