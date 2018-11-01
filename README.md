
# OHTI Headtracking device HW and Firmware + Player for streams or files of Ambisonic format recordings.

Update 1 Nov 2018

Now this project is using the September 2018 version of Omnitone.
The GUI is improved to allow easier access to Local Ambisonic files,
Also handling of files from the local webserver is changed, It can now handle more than 8 channels in a single file on Chrome and Firefox.
Assitance from Omnitone team was received and this non headtracked experimental HOA player was written for trouble shooting:
https://rawgit.com/GoogleChrome/omnitone/exp-opus-multichannel/examples/hoa-player.html

Host software to use a local webserver to allow Binaural Headphone listening with headtracking to Ambisonic files up to Third order using either Omnitone or JSAmbisonics binaural decoders in JavaScript capable browser like Chrome or Firefox.

The current headtracker hardware is based on nrF53832 and a BNO055 as IMU sensor with magnetic field sensor to minimize drift. 

History and Design choices:
The first version was based on https://www.rcgroups.com/forums/showthread.php?1677559-DIY-Headtracker-(Easy-build-No-drift-OpenSource) on a Arduino nano and a imu sensor board.
Matthias Kronlashner was kind enough to create a working sw plugin for his Ambix plugins suite that could use this headtracker.

I decided a couple of years ago to use the BNO055 Sensor as I saw the possibility to build a compact low cost sensor using Arduino nano and USB serial to the host computer this solution can be made to work with a variant of the current software.

One of the design decisions I took was to use I2C to interface between the CPU and the BNO055.

Next Step in the evolution was to look at using BT 2.0 to interface between the Playing host and the Head tracker, test was done with BT 2.0 modules and serial interface on the head tracker, it did not feel like a final solution with the 3 modules needed operating on different voltages.

Next step in developement was to change over to use BLE with nrf51822 this would merge BLE and CPU in to one chip, issues with I2C clock stretching and not having so good IDE support for BLE resulted in a slowdown of developement of that version.

As availability of free software for spatial VR sound and Spatial Audio Rendering on the webb started mature and also when my understanding of the developement IDE for Arduino with Nordic chips matured one more version was started.

The current OHTI is based on nrF52832 and BNO055 with I2C connectivity.
A lower HW cost USB Serial version will also be published.

Quaternions is used as directional information carrier, the nordic emulated serial protocol in S132 is used to carry the data.
The 4 quartenion values is compressed by z85 algorithm allowing the directional data to be sent in ja single BLE packet.

We have verified that this way of communicating between OHTI Headtracker and Web player can send and update the directional information in a loop that is run with a delay of 10 milliseconds between  the action to read the 4 * 16 bit quaternion registers and encode with z85 and send one BLE packet.
I estimate the possible update rate to be verified at over 90 Hz with this solution!

----------------------------------------------------------------------------------------------------------------
Host and Player Software:
By using JavaScript and browser for the players we can run the local host software on Linux, OSX and Windows 10.

The software offers the possibility to choose between Omnitone rel 1.2.5  and JSAmbisonics as binaural headtracked players.
The player allows local files in Ambix ACN format to be selected and played.
-----------------------------------------------------------------------------------------------------------------
Licensing:

Currently this distribution has a restricting License but the License is planned to change to a less restrictive later.

This software and firmware is the result of a long and slow journey with headtrackers starting 2014,
The start was as a result of wanting to listen to my own FOA Ambisonic (tetramic) recordings over headphones.



A lot of people have been kind to answer questions and assist in other ways, for example with software modifications.
A few of those I want to mention:

 The Sursound mailing list,
 Stefan Schreiber,
 Hector Centeno,
 Matthias Kronlachner,
 Marc Lavallee,
 Politis Archontis,
 David Poirier-Quinot,
 Antti Vanne  IDA Audio - personal SOFA file and SOFA converion programs,
 Aaron Heller,
 Fons Adriaensen,
 Len Moskowitz,

 and others.

 ------------------------------------------------------------------------------------------------
 ToDo

 - Verify and fix if not correct - included sound clips to be in Ambix ACN format - This as FuMa have been used recently.

 - Upload/add programs to convert personal SOFA files to formats that can be used to replace the binaural rendering,
 The format of these files are now changed.

 - Still a current Windows 10 problem, It looses the connection to the OHTI HeadTracker after a while usually within 10 minutes, This used not to happen around new year, I did run the software for many hours with no issues, No changes was done to OHTI firmware or Host software between tests. 
 I think I have a lead on this problem but need to recode and test.

 - Make the calibration results survive a power off, and be reloaded at next power on.
 
 - Make possible to initialize calibration with push button action.
 
 - Look in to if there is any advantage to move to a own specified BLE characterstics that transfers binary variable values,
   will probably not be done.



