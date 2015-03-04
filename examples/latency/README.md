Latency!
============

This is a basic test for RIOT. It test how the real time is RIOT.

Compare the latency between actual time and  when the thread wake up in especific time.
It prints out histogram from latency values. 

* First you can define Latency resolution and limit test values. 

* The variable `RIOTBASE` contains an absolute or relative path to the directory where you have checked out RIOT.
  If your code resides in a subdirectory of RIOT, then you can use `$(CURDIR)` as it's done in here.

* The variable `QUIET`, which is either `1` or `0`, defines whether to print verbose compile information, or hide them, respectively.

* The last line of your Makefile must be `include $(RIOTBASE)/Makefile.include`.

The code itself may look like your usual *C* beginners hello-world example.
