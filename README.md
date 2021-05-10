# Hoppy Foot - 1-DoF Leg Test

## Background

In redesigning the chip robot dog platform, we're starting by making a 1-DoF development platform leg to test communications, software stack, and controls. Much of the hardware on the Chip V2+ will remain the same but the mechanical system will be entirely re-designed as well as the control structure. The goal of the 1-DoF leg test is to test control on the actuators (which are now a 10:1 gear ratio as opposed to a 100:1). We will gain the following information from this test.

* Control of a REV Robotics SparkMAX/NEO w/ 10:1 gearbox in terms of impedance/torque control with Position/Velocity/ and Feed-Forward Torque. 
* Control required to make a 1-DoF leg jump and perform impact mitigation while landing.
* Max weight the actuation system can support (we will use variable masses on-top of the leg).
* A general control and software architechture to be implemented on the Chip V2+ robot in later stages of development. 
* Accuracy of the current sensor and if we can determine foot contact w/ the ground through the current sensor.

We will start w/ this 1-DoF system, then redesign the entire leg and make the 3-DoF system functional before finally re-designing the Chip V2 robot to become Chip V2+ and start manufacturing that robot and writing the software for that. We think this approach will allow us to develop a better robotic system by starting from the ground-up.

## This Repository

This repository contains the code as well as basic mechatronics design for our 1-DoF leg test as well as a bill-of-materials. As this system is very simple electrically, mechanically, and on the control side, most of the documentation will be contained in this README.md file. For other documentation please email chipv2@mit.edu or adim@mit.edu.

## Mechanical Design

## Hardware System

## Software Architechture 

## Bill of Materials

## Credits

MIT Biomimetic Robotics Lab; 
Professor Sangbae Kim;
Elijah B. Stranger-Jones;

## License

###### This repository is licensed under the MIT License, see LICENSE.md for more inforamtion. (MIT License Copyright (c) 2021 Chip V2)
