# NAI-75ARM1-SelfNavigationProgram
Summer 2023 Intern Project Archive

# Lidar-Based Autonomous Driving Project

## Summary

In this portfolio project, I focused on exploring the practical applications of Lidar technology in autonomous driving systems. With the guidance and assistance of my mentors and colleagues, I aimed to leverage widely-used and efficient technology to achieve feasible autonomous operations.

---

## Table of Contents

- [Module Selection](#module-selection)
- [Configuration Process](#configuration-process)
- [Functionality Testing](#functionality-testing)
- [XILINX SDK Development](#xilinx-sdk-development)
- [Core Program Design](#core-program-design)
- [Mentors and Colleague Support](#mentors-and-colleague-support)
- [NAI's Configurable Hardware Architecture (COSA)](#nais-configurable-hardware-architecture-cosa)
- [Conclusion](#conclusion)

---

## Module Selection

Selecting the appropriate modules was crucial for the project's success. With input from mentors and colleagues, I opted for the MOD-SX2-RS-8CH and MOD-SX2-DA5-4CH modules integrated into the 75ARM1 single-board arm computer. These modules provided the required computing power, which was essential for the autonomous functionality of the system.

## Configuration Process

The configuration phase presented several challenges, but with the support of my mentors, I successfully tackled them. This phase involved flashing the board with firmware, programming Infineon chips via I2C, configuring the main ZYNQ arm chip through SERIAL, and setting up general-purpose FPGAs. Additionally, proper communication among the modules and formatting the onboard SATA chip were prerequisites for successful implementation.

## Functionality Testing

To verify the viability of the project, functionality testing was conducted with guidance from mentors. I successfully integrated the Lidar with the SC3 module and
