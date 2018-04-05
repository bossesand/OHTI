<<<<<<< HEAD
# OHTI Headtracking device HW and Firmware

Host software to create a local webserver to allow listening to Ambisonic files up to Third order using either Omnitone or JSAmbisonics library in JavaScript capable browser like Chrome or Firefox.

The current hardware is based on nrF53832 and a BNO055 as IMU sensor with magnetic field sensor to minimize drift. 

The first version was based on https://www.rcgroups.com/forums/showthread.php?1677559-DIY-Headtracker-(Easy-build-No-drift-OpenSource) on a Arduino nano and a imu sensor board.
Matthias Kronlashner was kind enough to create a working sw plugin for his Ambix plugins suit that could use this headtracker.

I decided a couple of years ago to use the BNO055 Sensor as I saw the possibility to build a compact low cost sensor using Arduino nano and USB serial to the host computer this solution can be made to work with a variant of the current software.

One of the design decisions I took was to use I2C to interface between the CPU and the BNO055.

Next Step in the evolution was to look at using BT 2.0 to interface between the Playing host and the Head tracker, test was done with BT 2.0 modules and serial interface on the head tracker, it did not feel like final solution with 3 modules needed operating on different voltages.

Next step in developement was to change over to use BLE with nrf51822 this would merge BLE and CPU in to one chip, issues with I2C clock stretching and not having so good IDE support for BLE resulting in stopping that version.

As availability of free software for spatial VR sound and Spatial Audio Rendering on the webb started mature and also my understanding of the developement IDE for Arduino with nordic chips matured one more version was started.

The current OHTI is based on nrF52832 and BNO055 with I2C connectivity.
Quaternions is used as directional information carrier, the nordic emulated serial protocol in S132 is used to carry the data.
The 4 quartenion values is compressed by z85 allowing data to be sent over the serial port in just one BLE packet.

We have verified that this way of communicating between OHTI Headtracker and Web player can send and update the directional information in a loop that is run with a delay of 10 milliseconds between  the action to read the 4 16 bit quaternion registers and encode with z85 and send one BLE packet.
I estimate the possible update rate to be verified at over 90 Hz with this solution!

----------------------------------------------------------------------------------------------------------------
Host Software:
By using JavaScript and browser for the players we can run the host software on Linux, OSX and Windows 10.

The software offers the possibility to choose between Omnitone rel 1.3  and JSAmbisonics as binaural headtracked players.
The player allows local files in Ambix ACN format to be selected and played.
-----------------------------------------------------------------------------------------------------------------
Licensing:

Currently this distribution has a restricting License but the License is planned to change to a less restrictive later.

This software and firmware is the result of a long and slow journey with headtrackers starting 2014,
The start was as a result of wanting to listen to my own FOA Ambisonic (tetramic) recordings over headphones.



A lot of people have been kind to answer questions and assist in other ways, for example with software modifications.
A few of those I want to mention:

 The Sursound mailing list
 Stefan Schreiber
 Hector Centeno
 Matthias Kronlachner
 Marc Lavallee
 Politis Archontis
 David Poirier-Quinot
 Antti Vanne  IDA Audio - personal SOFA file and SOFA converion programs
 Aaron Heller
 Fons Adriaensen
 Len Moskowitz

 and others.

 ------------------------------------------------------------------------------------------------
 ToDo

 - Verify and fix if not correct - included sound clips to be in Ambix ACN format - This as FuMa have been used recently.

 - Upload/add programs to convert personal SOFA files to formats that can be used to replace the binaural rendering.

 - Windows 10 problem, It looses the connection to the OHTI HeadTracker after a while usually within 10 minutes, This used not to happen a month ago, before this I did run the software for many hours with no issues, No changes was done to OHTI firmware or Host software between tests. 

 - Look in to if there is any adwantage to move to a own specified BLE characterstics that transfers binary variable values?








=======
**Edit a file, create a new file, and clone from Bitbucket in under 2 minutes**

When you're done, you can delete the content in this README and update the file with details for others getting started with your repository.

*We recommend that you open this README in another tab as you perform the tasks below. You can [watch our video](https://youtu.be/0ocf7u76WSo) for a full demo of all the steps in this tutorial. Open the video in a new tab to avoid leaving Bitbucket.*

---

## Edit a file

You’ll start by editing this README file to learn how to edit a file in Bitbucket.

1. Click **Source** on the left side.
2. Click the README.md link from the list of files.
3. Click the **Edit** button.
4. Delete the following text: *Deleted*
5. After making your change, click **Commit** and then **Commit** again in the dialog. The commit page will open and you’ll see the change you just made.
6. Go back to the **Source** page.

---

## Create a file

Next, you’ll add a new file to this repository.

1. Click the **New file** button at the top of the **Source** page.
2. Give the file a filename of **contributors.txt**.
3. Enter your name in the empty file space.
4. Click **Commit** and then **Commit** again in the dialog.
5. Go back to the **Source** page.

Before you move on, go ahead and explore the repository. You've already seen the **Source** page, but check out the **Commits**, **Branches**, and **Settings** pages.

---

## Clone a repository

Use these steps to clone from SourceTree, our client for using the repository command-line free. Cloning allows you to work on your files locally. If you don't yet have SourceTree, [download and install first](https://www.sourcetreeapp.com/). If you prefer to clone from the command line, see [Clone a repository](https://confluence.atlassian.com/x/4whODQ).

1. You’ll see the clone button under the **Source** heading. Click that button.
2. Now click **Check out in SourceTree**. You may need to create a SourceTree account or log in.
3. When you see the **Clone New** dialog in SourceTree, update the destination path and name if you’d like to and then click **Clone**.
4. Open the directory you just created to see your repository’s files.

Now that you're more familiar with your Bitbucket repository, go ahead and add a new file locally. You can [push your change back to Bitbucket with SourceTree](https://confluence.atlassian.com/x/iqyBMg), or you can [add, commit,](https://confluence.atlassian.com/x/8QhODQ) and [push from the command line](https://confluence.atlassian.com/x/NQ0zDQ).
>>>>>>> 73a09229e979bd33a412251ba0b64292e84899fb
