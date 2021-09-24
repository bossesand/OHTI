# How to install Windows 10

Install npm and node on your win 10 computer, also python 2.7 its needed

<https://blog.teamtreehouse.com/install-node-js-npm-windows>

Tips from internet:

<https://www.espruino.com/BLE+Advertising>

Node.js

Node.js has a great module called Noble available that works on Windows, Mac OS and Linux.

Mac OS Mojave currently has broken BLE support in noble so you may want to use the noble-mac library instead.

Windows 10 users may want to use noble-uwp instead of noble to get out-of-the-box Bluetooth support.

Windows support requires a USB Bluetooth LE dongle that Windows doesn't have a Bluetooth driver installed for. This means you don't have to be using Windows 10, but if you do have Windows 10 and Bluetooth LE is working, you'll want to plug in an additional USB BLE dongle!.

Make sure Node.js is installed

Then install noble via npm

Follow all of this exept use noble-uwp instead of noble !!!
<https://github.com/noble/noble/blob/master/README.md>

install noble-uwp
<https://github.com/jasongin/noble-uwp>

`npm install --global --production windows-build-tool`

`npm install npm@latest -g`

`npm install noble-uwp --no-optional` - for Windows 10 only!

`npm install noble --no-optional` - Is this really needed for Win10?

`npm install serialport --no-optional`

`npm install bluetooth-hci-socket --no-optional`

`npm install usb --no-optional`

<https://www.npmjs.com/package/node-gyp> - complete install instructions!

`npm install -g node-pre-gyp`

`npm install -g node-gyp`

`npm install --unsafe-perm usb`

`node-gyp --python C:/Python27/`

`npm config set python C:/Python27/`

`C:\Program Files (x86)\nodejs>npm install node-gyp`

`npm config set msvs_version 2015 --global`

`npm install express`

`SET PATH=C:\Program Files\Nodejs;%PATH%`

`git clone --recursive https://github.com/nonolith/node-usb.git`

`cd node-usb`

`npm install`

On windows 10
<https://zadig.akeo.ie/> is needed which basically replace the driver loaded for a particular device so now it is not appearing as a regular COM Port, it is now detected as a "real" USB Serial Device without a COM port attached to it, and now chrome is able to connect to it just fine even without enabling the new backend flag.
