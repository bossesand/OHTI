
On a win10 PC you might need an extra usb BLE reciver if the built in chipset is not compatible with github.com/noble
I have tested using i CSR8510 A10 usb to BT4.0

install Node.js command prompt

Your environment has been set up for using Node.js 6.11.4 (x64) and npm.
To use the npm command for adding/downloading JavaScript/node modules you need to habe Python 2.7 installed also.

C:\Users\Bosse\Documents\BitBucketData\ohti-bossesand>

 // if you do not start in the correct directory change to the directory where you have the ohti-bossesand

Start the app with the command:

node main_ht_app.js

// if you get Error: cannot find module "express" or other run the command npm install express or with other missing module name.



Use chrome to connect to  http://localhost:81  to start the ambisonic headtracked player

Select omnitone or JSambisonics players

On top there is 6 buttons from left to right the functions are

1 - Toggle the text panel for function choices and hiding of panel.

2 - To handle the connection to the OHTI BLE HW, 
     Start discover/scan
     choose "OHTI undefined" from the rolldown menu
     connect to BLE Device
     Start HT-track BLE

3 - Configure for usb port when connected with cable ( have been verified to wor but not included in this OHTI firmware release)

4 - Set headtracker reference, reset head direction to forward listening at horizontal level.

5 - Reset headtracked reference to previous value