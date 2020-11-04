# Arduino Core for STM32L0 based boards

## What is it ?

ArduinoCore-stm32l0 is targeted at ultra low power scenarios, sensor hubs, with LoRaWAN connectivity.

## Supported boards
 * MJS2020-PROTO1
 * MJS2020-PROTO2

## Installing

### Board Manager

 1. [Download and install the Arduino IDE](https://www.arduino.cc/en/Main/Software) (at least version v1.6.8)
 2. Start the Arduino IDE
 3. Go into File -> Preferences
 4. Add ```https://github.com/meetjestad/mjs_boards/raw/master/package_meetjestad.net_index.json``` as an "Additional Board Manager URL"
 5. Open the Boards Manager from the Tools -> Board menu and install "Meetjestad! STM32L0 Boards"
 6. Select your board from the Tools -> Board menu

#### OS Specific Setup

##### Linux

 1. Download [49-meetjestad.rules](drivers/linux/49-meetjestad.rules).
    This file configures udev to assign the right permissions to the
    various device files.
 2. Place it into the ```/etc/udev/rules.d``` directory.

        sudo cp 49-meetjestad.rules /etc/udev/rules.d

 3. Add your own user to the "dialout" group to make use of the
    permissions assigned by udev. If you've used Arduino before, you
    likely already did this step (you can check your current groups with the `id` command).

        sudo usermod -a -G dialout $USERNAME

 4. Log out and in again so the new group takes effect.

This assumes the "dialout" group is available and used for serial ports,
which is the default on most Linux distributions. If not, the above
commands and udev rules must be changed accordingly.

#####  Windows

###### STM32 BOOTLOADER driver setup for MJS2020 boards (for uploading sketches)

 1. Download [Zadig](http://zadig.akeo.ie)
 2. Plugin STM32L0 board and toggle the RESET button with the BOOT0 jumper present (or while manually shorting the BOOT0 pins)
 3. Let Windows finish searching for drivers
 4. Start ```Zadig```
 5. Select ```Options -> List All Devices```
 6. Select ```STM32 BOOTLOADER``` from the device dropdown
 7. Select ```WinUSB (v6.1.7600.16385)``` as new driver
 8. Click ```Replace Driver```

###### USB Serial driver setup for MJS2020 boards (for accessing USB serial port, Window XP / Windows 7 only)

 0. This is not needed on Windows 10
 1. Download [```dpinst_x86.exe```](drivers/windows/dpinst_x86.exe) (32 bit Windows) or [```dpinst_amd64.exe```](drivers/windows/dpinst_amd64.exe) (64 bit Windows)
 2. Right-click the downloaded file and select ```Run as administrator```
 3. Click on ```Install this driver software anyway``` at the ```Windows Security``` popup as the driver is unsigned

###### ST-LINK V2.1 driver setup for STMicroelectronics boards (for step-by-step debugging)

 0. This is only needed if you have a ST-LINK debugger dongle.
 1. Plugin STMicroelectronics board
 2. Download and install [ST-Link USB Drivers](http://www.st.com/en/embedded-software/stsw-link009.html)

### From git (to run the latest unreleased version)

 1. Follow steps from Board Manager section above (this installs also
    installs the core but is needed to ensure the required tools,
    compilers, etc. are installd).
 2. ```cd <SKETCHBOOK>```, where ```<SKETCHBOOK>``` is your Arduino Sketch folder:
  * OS X: ```~/Documents/Arduino```
  * Linux: ```~/Arduino```
  * Windows: ```~/Documents/Arduino```
 3. Create a folder named ```hardware```, if it does not exist, and change into it
 3. Create a subfolder named ```meetjestad-git```, if it does not exist, and change into it
 4. Clone this repo in a subfolder called ```stm32l0```
 5. If done correctly, you now have e.g.
    ```~/Arduino/Arduino/hardware/meetjestad-git/stm32l0/boards.txt```.
 6. Restart the Arduino IDE.

Note that this uses ```meetjestad-git``` rather than just
```meetjestad```, to distinguish your git-cloned version from the
board-manager-installed version. Inside the IDE, you will now have the
boards listed twice. Recent IDE versions mark the one from git as "in
sketchbook" to distinguish it from the board-manager installed version.


## Recovering from a faulty sketch

Sometimes a faulty sketch can render the normal USB Serial based
integration into the Arduindo IDE not working. In this case plugin the
board and toggle the RESET button while the BOOT0 pins are shorted (e.g.
using a jumper). This forces the board into bootloader mode and should
allow a single upload to succeed.
