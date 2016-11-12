Meet je stad! Arduino board definitions
=======================================

The files in this repository allow using the Meet je stad sensor board
together with the Arduino IDE. These files tell the Arduino IDE about
the hardware, how to compile code for it, and how to upload the compiled
code to the board.

To set this up, you can use the Arduino IDE boards manager:
 - Open the Arduino IDE, and go to File -> Preferences.
 - In the "Additional board manager URLs", add the following url:
   `https://github.com/meetjestad/mjs_boards/raw/master/package_meetjestad.net_index.json`
 - Close the preferences, and open Tools -> Board -> Boards Manager...
 - Find the "Meet je stad! AVR Boards" in the list, and click Install

 Alternatively, you can manually install the boards files by heading
 over to the latest release on github, downloading the `mjs_boards.zip`
 file and unpacking that into a `hardware/meetjesad` directory inside
 your sketchbook (so you would have
 `/path/to/sketchbook/hardware/meetjestad/avr/boards.txt`)

License
=======
The files in this repository are distributed under various licenses.
Each file lists the license that applies to it, with the exception of
the bootloader binaries. These are distributed under the GPLv2, see
https://github.com/Optiboot/optiboot for the full terms and the sources
used to compile the bootloader (see the commit log of this repository to
find out the exact version used).
