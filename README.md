# DueMotorCtrl
An Arduino Due (SAM3X) based motor controller. 
Somewhat generic in terms of hardware but really meant 
primarily to replace the main board in a DMOC645

There are a few basic reasons for this to exist:

1. There are nearly 100 DMOC's (maybe more) that are air cooled and 
somewhat between the gen1 and gen2 DMOCs. There isn't much information 
about these and so these DMOCs cannot other be used.

2. The Azure DMOCs aren't really meant for driving all sorts of motors
so if you don't use the exact motor the firmware was meant for you 
run into trouble.

3. There aren't really any good open source examples of using the SAM3X
in an inverter. That's unfortunate as the SAM3X has full support for
synchronized PWM channels with dead time as well as many other useful
features for being a motor controller. But, it isn't a very common
chip to use for motor control.
