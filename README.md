PIC-Accelerometer
=================

A program for a PIC 18 series to measure the accelerometer. Intended to be on a car.

Our micro-controller has to be programmed so we can capture (we will use the Capture pins for this) the two PWM's generated by the accelerometers, and perform the following tasks:

Calculate the tilting angle in that axis: knowing the duty of the PWM to calculate the tilt angle we do as follows: 2∗π∗arcsin((X1/(X1+X2)−0.5)∗8) . So after capturing the PWM's duty we should do this operation.
Average the last three angle measurements: The accelerometers measurements are usually noisy, so we want to average the last 5 measures. To do this we simply create a five element array and a pointer to this array so we can keep track of where the last angle was added. In the following task, when I talk about angles I am referring to the average of the angles.
Print the tilt angles to an LCD: A standard two-lines LCD is connected to the uController and we have to output the tilting values previously calculated.
Generate two PMW's encoding the angle: Once we have calculated the angles we have to also encode them into PWM with a duty of 50% representing an angle of zero degrees and for each degree negative or positive we should add or subtract respectively 1% of the duty. That way an angle of 13 deg. should be encoded as a duty of 63%. This PWM should have a total period of 1 msec.
Record the angles in a EEPROM: To keep historical data, it can be useful in case of accident or other problems, sort of simple black box for cars. To communicate with the EEPROM we will use i2C .

You should be able to compile it with CCS compiler or equivalent. I am not including the lcd.h file because it has a copyright by the CCS compiler so you may want to comment out the printf lines in the main file if you don't want to compile it with this functionality.



