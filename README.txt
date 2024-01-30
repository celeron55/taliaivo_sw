Sumobrain
=========

Using Taliaivo 1.0 hardware
---------------------------
Manual fixes to be done on a fresh board:
1. Solder a 4.7kohm resistor or wire from STM32 pin 37 to a nearby ground, for
   example the near side of R10 or the far side of R20
	* This connects BOOT1 to ground
	* NOTE: There are multiple unused pins around pin 37, for example 35, 36,
	  38, 39. For this reason you can solder bridge the neighboring pins and it
	  does not hurt at all, which makes this modification very easy to make.
	* NOTE: The pin at the top line of the "R17" text is 38. 37 is the one just
	  off from the "R17" text.
	* This enables booting the bootloader by connecting BOOT0 to 3.3V even after
	  initially flashing the device
2. Solder a 470 ohm (or so) resistor from D3 positive side to U12.8 or U11.4
	* This connects VUSB to PA9=OTG_FS_VBUS
	* This enables the USB functionality of most USB libraries, including the
	  one in CircuitPython
3. Solder a jumper wire from C23 to C34 (VREF to 3.3V)
4. Solder two resistors from STM32 pin 32 and 33 (parallel them, for ease of
   soldering):
	1. 100k to the center terminal of SW1 (the big switch)
	2. 4.7k to GND (e.g. R20, R10, C17 or C20)
5. Solder U6(MPU6050).9 -> U6.8 (or to any other 3.3V pad)

Enabling the DFU:
- Connect BOOT0 to 3.3V when powering up the board via USB. This should put the
  MCU into USB DFU mode

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

Compiling for physical hardware
-------------------------------
$ mkdir lib
$ cd lib
$ git clone --depth=1 https://github.com/embassy-rs/embassy.git
$ cd embassy
$ git checkout 343be37f397ff82eb9d34161920449cc4b45c31a
$ cd sumobrain_embedded
$ ./build_release.sh

Flashing physical hardware
--------------------------
Direct USB firmware update using DFU mode:
- Hold BOOT0 connected to 3.3V
- Connect USB.
- The board should appear as "Product: STM32 BOOTLOADER"
$ ./dfu_release.sh
- OR
$ dfu-util -a 0 --dfuse-address 0x08000000  -D ../target/thumbv7em-none-eabihf/release/sumobrain_embedded.bin

Using an ST-Link v2 clone or similar:
- Connect GND, SWDIO, SWCLK and 3.3V
$ ./flash_release.sh

- This also kind of works, but RTT logging is not set up so it will complain after flashing succeeds:
$ cargo run --release

Monitoring using USB serial
---------------------------
Pressing a key after running the command starts USB logging
$ picocom --baud 115200 -r -l -c -e x /dev/ttyACM0
$ while true; do picocom --baud 115200 -r -l -c -e x /dev/ttyACM0; sleep 1; done

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

Editing
-------
- Use any editor (vim is of course recommended 8-))
- Install bacon and run it in another terminal
$ cargo install --locked bacon
$ bacon
