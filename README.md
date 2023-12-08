# firebot

ROS2-based Robogames Fire Fighter

## Setup (ROS 2 Iron, 22.04)

 * Build things in workspace:
   * https://github.com/mikeferguson/etherbotix
   * https://github.com/mikeferguson/firebot
   * https://github.com/mikeferguson/robot_controllers

## Launch Files and Nodes

 * drivers.launch.py - always needed for hardware interfaces.
 * build_map.launch.py - do not run localization and navigation at same time as building a map.
 * localization.launch.py - mostly working localization, can still blow up at times.
 * navigation.launch.py - in progress.
 * main.py - this is the main entry spot for fire fighting.

## Pinouts

 * A0 - left cliff sensor
 * A1 - center cliff sensor
 * A2 - right cliff sensor
 * D3 - ?
 * D4 - laser RX
 * D5 - laser PWM (TIM12_CH2)
 * D6 - fan enable
 * D7 - start button
