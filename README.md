# Twin-Helicopter Slung Load Transportation System
This repository contains all the hardware and software development that was done for the Twin-Helicopter Slung Load Transportaton System project.

![alt-text](https://github.com/rddyas002/Twin-Helicopter-Slung-Load-Transportation-System/blob/master/twin_lift.jpg "Twin-Helicopter Load Transportation Scheme")

## Folder Hierarchy
-> root <br/>
---> Hardware	(Contain schematics and PCB designs)<br/>
-----> Aerial Application Board 2<br/>
-----> RadioInterface-v2<br/>
--->Software	(Contain source code)<br/>
-----> Aerial Application Board	(firmware for main avionics board)<br/>
-----> Base Station<br/>
--------> Archived	(old development code)<br/>
--------> MotionCatureSlave	(Image processing code that runs on uMocap slave PCs)<br/>
--------> MotionCaptureSlaveOnServer	<br/>
--------> UAVControl	(Main base-station software, command and control, state-estimation)<br/>
-----> Radio Multiplexer	(firmware for radio multiplexer board)<br/>

## Simulations and Videos
1. Single helicopter position control test <https://youtu.be/7Kq4-oiGThU>
2. Simulation showing trajectory tracking of translation and attitude <https://youtu.be/bJs7ehfYPP0>


