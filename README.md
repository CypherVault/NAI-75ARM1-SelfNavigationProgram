# NAI-75ARM1-SelfNavigationProgram
Summer 2023 Intern Project Archive

# Lidar-Based Autonomous Driving Project

## Summary

In this portfolio project, I focused on exploring the practical applications of Lidar technology in autonomous driving systems. With the assistance of my mentors and colleagues, the project aimed to explore the potential of Lidar technology in autonomous driving systems, a domain currently gaining considerable attention. The primary objective was to achieve feasible and autonomous operations by leveraging available modules specialized in data handling and power generation.

Module selection was critical for successful implementation. With guidance from mentors and input from colleagues, I chose the MOD-SX2-RS-8CH and MOD-SX2-DA5-4CH modules, integrated into the 75ARM1 single-board arm computer. The combination of these modules provided the required computing power for autonomous functionality.

Further details on the 75ARM1 can be found [here](https://www.naii.com/Model/75ARM1), and details for the MOD-SX2-RS-8CH module can be found [here](https://www.naii.com/Model/SC3), while information about the MOD-SX2-DA5-4CH module is available [here](https://www.naii.com/Model/DA5).

## Configuration Process

The configuration process was an essential phase of the project, albeit with its challenges. With the guidance of my mentors, I successfully navigated through tasks such as flashing the board with firmware, programming the Infineon chips through I2C, configuring the main ZYNQ arm chip via SERIAL, and setting up general-purpose FPGAs. Additionally, formatting the onboard SATA chip and ensuring proper communication among the modules, with the assistance of my colleagues, were prerequisites for a successful implementation.

## Functionality Testing

To validate the project's viability, functionality testing was conducted. Guided by mentors, I integrated the Lidar with the SC3 module, demonstrating successful communication. Moreover, the MOD-SX2-DA5-4CH module, with the support of my colleagues, effectively operated the motor within predefined parameters, validating the proof of concept.

## XILINX SDK Development

Developing the XILINX SDK for the ZYNQ 7015 chip posed a challenge, necessitating logins, licenses, and access to the NAI software library and dependencies. Nevertheless, with the assistance of my mentors and colleagues, I successfully established a functional workspace, allowing access to the required source files and enabling effective coding for the modules and ZYNQ chip.

## Core Program Design

The core program was designed to operate autonomously by continuously reading data from the front left and right side Lidars within a while loop. Decisions were made based on user-defined thresholds, determining whether to proceed forward or make directional adjustments to avoid potential collisions. Feedback and suggestions from my colleagues were invaluable in refining the core program.

## Mentors and Colleague Support

Throughout this project, the guidance and support of my mentors and colleagues proved instrumental in overcoming challenges and achieving success. Their expertise and insights helped me navigate through complex tasks and provided valuable perspectives on various aspects of the project.

## NAI's Configurable Hardware Architecture (COSA)

The significance of NAI's configurable hardware architecture, COSA, was underscored throughout the project. It offered numerous options for tailoring solutions to specific project requirements, showcasing the flexibility and versatility of NAI's product line. The compact form factor of the 9 X 4 inch computer contributed significantly to the project's success.

## Conclusion

The autonomous driving implementation with Lidar technology, with the assistance of my mentors and colleagues, proved to be a robust and promising venture. With continuous advancements in Lidar technology, the potential for autonomous driving and various other applications appears promising. The success of this project owes credit to the invaluable support and dedication of my mentors and colleagues, as well as NAI's configurable hardware options. The project underscores the significance of NAI's COSA architecture, offering tailored solutions for specific project requirements, and highlights the potential for autonomous driving with Lidar technology in the future.
