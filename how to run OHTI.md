
On a win10 PC you might need an extra usb BLE reciver if the built in chipset is not compatible with github.com/noble
I have tested using i CSR8510 A10 usb to BT4.0

install Node.js command prompt
Start the app

Your environment has been set up for using Node.js 6.11.4 (x64) and npm.

C:\Users\Bosse\Documents\BitBucketData\ohti-bossesand>

 // if you do not start in the correct directory change to the directory where you have the ohti-bossesand

C:\Users\Bosse\Documents\BitBucketData\ohti-bossesand>node main_ht_app.js port 82

// if you get Error: cannot find module "express" or other run the command npm install express or wit other module name.

node main_ht_app.js  port 82

Use chrome to connect to  http://localhost:82  to start the ambisonic headtracked player

Select omnitone or JSambisonics players

On top there is 6 buttons from left to right the functions are

1 - Toggle the text panel for function choices and hiding of panel.

2 - To handle the connection to the OHTI BLE HW, 
     Start discover/scan
     choose "OHTI undefined" from the rolldown menu
     connect to BLE Device
     Start HT-track BLE
     
     Scan for a while, OHTI needs to be found, It takes a while

3 - Configure for usb port when connected with cable ( have been verified to wor but not included in this OHTI firmware release)

4 - Set headtracker reference, reset head direction to forward listening at horizontal level.

5 - Reset headtracked reference to previous value

