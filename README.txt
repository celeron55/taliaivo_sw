Sumobrain
=========

Performance benchmarking
------------------------
Heap profiling
- Mainly just to see that Piston is behaving. Heap allocations aren't used on
  the embedded target.
$ cd sumobrain_simulator
$ cargo build
$ valgrind --tool=massif ../target/debug/sumobrain_simulator
$ ms_print massif.out.<pid> | less

Stack profiling:
- This gives an idea about how much memory will be used on the embedded target
  also.
$ cd sumobrain_simulator
$ cargo build
$ valgrind --tool=massif --heap=no --stacks=yes ../target/debug/sumobrain_simulator
$ ms_print massif.out.<pid> | less

Debugging on physical hardware
------------------------------
$ cd sumobrain_embedded
$ ./build_release.sh
$ ./flash_release.sh
$ openocd -f interface/stlink.cfg -f target/stm32f4x.cfg

In another terminal:
$ cd sumobrain_embedded
$ arm-none-eabi-gdb ../target/thumbv7em-none-eabihf/release/sumobrain_embedded
(gdb) target extended-remote localhost:3333
(gdb) break Reset
