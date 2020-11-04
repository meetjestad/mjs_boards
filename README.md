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

License
=======
All content in this repository is licensed under the ["Beluki"
license][1]:

Permission is hereby granted, free of charge, to anyone obtaining a copy
of this document and accompanying files, to do whatever they want with
them without any restriction, including, but not limited to, copying,
modification and redistribution.

NO WARRANTY OF ANY KIND IS PROVIDED.

[1]: https://github.com/Beluki/License/blob/master/Documentation/License
