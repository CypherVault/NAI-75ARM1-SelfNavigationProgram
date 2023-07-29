# NAI-75ARM1-SelfNavigationProgram
Summer 2023 Intern Project Archive

# Lidar-Based Autonomous Driving Project

## Summary

In this portfolio project, I focused on exploring the practical applications of Lidar technology in autonomous driving systems. With the assistance of my mentors and colleagues, the project aimed to explore the potential of Lidar technology in autonomous driving systems, a domain currently gaining considerable attention. The primary objective was to achieve feasible and autonomous operations by leveraging available modules specialized in data handling and power generation.

Module selection was critical for successful implementation. With guidance from mentors and input from colleagues, I chose the MOD-SX2-RS-8CH and MOD-SX2-DA5-4CH modules, integrated into the 75ARM1 single-board arm computer. The combination of these modules provided the required computing power for autonomous functionality.

Further details on the 75ARM1 can be found [here](https://www.naii.com/Model/75ARM1), and details for the MOD-SX2-RS-8CH module can be found [here](https://www.naii.com/Model/SC3), while information about the MOD-SX2-DA5-4CH module is available [here](https://www.naii.com/Model/DA5).

## Configuration Process
The system needed alot of hardware and software config to be able to even operate. Internal documentation tools and flashing procedures were used on both modules and the motherboard. Firmware, power delivery, NVRAM conifg, ethernet access, and Petalinux installation all needed to occur before the board was even able to be tested on.

## Functionality Testing
Using internal testing suites, and panels, as well as documentation on the modules,my mentor and i isolated evrything required for our goals to be acheived in regards to the DA module operation as well as SC3 module operation. Countless hours were spent pinging out connections to the board, documenting parts and outputs/inputs. 

## XILINX SDK Development
The SDK was a challenge as i needed a lot of internal libraries and fucntions to enable the board to properly communicate with the modules as well as build a compilable petalinux arm executable. 

## Core Program Design

The core program was designed to operate autonomously by continuously reading data from the front left and right side Lidars within a while loop. Decisions were made based on user-defined thresholds, determining whether to proceed forward or make directional adjustments to avoid potential collisions. Feedback and suggestions from my colleagues were invaluable in refining the core program.

## Mentors and Colleague Support
Mike Mason ( Mentor ) 
Other Collegues Support:
Caitlyn S, Hayen D, Mike P, Andrew L, Brendan T, Terence L. Kurt C, Kaitlin F, Paul K, Thomas C, Salvator B, Salvator D.
Assistance given: Board / Module FIrmware prep, Inspiration, Hardware Reccomendation, Solder / construction help, SDK knowldege, SDK setup, SC3 research, IDE installation, IT support, Internal library access, IO config for modules, DA5 Research

## NAI's Configurable Hardware Architecture (COSA)

The significance of NAI's configurable hardware architecture, COSA, was underscored throughout the project. It offered numerous options for tailoring solutions to specific project requirements, showcasing the flexibility and versatility of NAI's product line. The compact form factor of the project was also very impressive and only possible thank to our design philosophy.

## Conclusion

The autonomous driving implementation with Lidar technology, with the assistance of my mentors and colleagues, proved to be a robust and promising venture. With continuous advancements in Lidar technology, the potential for autonomous driving and various other applications appears promising. The success of this project owes credit to the invaluable support and dedication of my mentors and colleagues, as well as NAI's configurable hardware options. The project underscores the significance of NAI's COSA architecture, offering tailored solutions for specific project requirements, and highlights the potential for autonomous driving with Lidar technology in the future.



# Project Write Up 

**Portfolio Project Report: Implementation of Autonomous Driving with Lidar Technology**

In this portfolio project, with the assistance of my mentors and colleagues from time to time, the focus was on practical and widely-used technology, Lidar, which holds significant applications in both military and civilian contexts. The primary objective was to explore its potential in autonomous driving systems, a domain currently gaining considerable attention. Leveraging available modules specialized in data handling and power generation, the project aimed to achieve feasible and autonomous operations.

Shown above: SC3(left), DA5(right)
Module selection was critical for successful implementation. With guidance from my mentors and input from my colleagues, I chose the MOD-SX2-RS-8CH and the MOD-SX2-DA5-4CH modules, integrated into the 75ARM1[link: https://www.naii.com/Model/75ARM1] single-board arm computer. The combination of these modules, with the support of my colleagues, provided the required computing power for autonomous functionality. Further details on the MOD-SX2-RS-8CH module can be found at [link: https://www.naii.com/Model/SC3], while information about the MOD-SX2-DA5-4CH module is available at [link: https://www.naii.com/Model/DA5].

Shown above: 75ARM1 with SC3 and DA5 installed
The configuration process, with guidance from my mentors, was an essential phase, although it presented several challenges. Flashing the board with firmware, programming the Infineon chips through I2C, configuring the main ZYNQ arm chip via SERIAL, and setting up the general-purpose FPGAs demanded meticulous attention. Additionally, formatting the onboard SATA chip and ensuring proper communication among the modules, with the assistance of my colleagues, were prerequisites for a successful implementation.

With the support and advice of my mentors, functionality testing was conducted to verify the project's viability. The Lidar integration with the SC3 module, as guided by my mentors, demonstrated successful communication, and the MOD-SX2-DA5-4CH, with the assistance of my colleagues, effectively operated the motor within the predefined parameters, validating the proof of concept.

Developing the XILINX SDK for the ZYNQ 7015 chip posed a challenge, necessitating logins, licenses, and access to the NAI software library and dependencies. Nevertheless a functional workspace was established, allowing access to the required source files and enabling effective coding for the modules and ZYNQ chip.

The core program, with feedback and suggestions from my colleagues, was designed to operate autonomously by continuously reading data from the front left and right side Lidars within a while loop. Decisions were made based on user-defined thresholds, determining whether to proceed forward or make directional adjustments to avoid potential collisions.

Throughout this project, the guidance and support of many other colleagues and departments proved instrumental in overcoming challenges and achieving success. This project underscored the significance of NAI's configurable hardware architecture, COSA, offering numerous options for tailoring solutions to specific project requirements. The compact form factor of the 9 X 4 inch computer showcased the flexibility and versatility of NAI's product line, contributing significantly to the project's success.

To conclude, implementing a goal with Lidar technology proved to be a promising and educational adventure within NAI’s configurable and well documented hardware eco system. The success of this project owes credit to the invaluable support and dedication of my mentors and colleagues, as well as NAI's configurable hardware options.



