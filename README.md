# Mini Speaker
## Overview
The goal of this project is to convert the guitar amplifer from LEGO set no. 21379 into functional desktop speaker. Modifications include original and custom 3D-printed parts to support peripherals integration and PCB support. The LEGO amplifier is modeled in Fusion 360 in an effort to assist in the project's vizualization.

### [Design](3D_Model)
Features of the amp are exploited to increase the level of user-interaction with the device. Knobs along the front of the control panel house a small 3D-printed coupling device to actuate potentiometers that control volume and other effects. One of the knobs is replaced with a tactile switch to control the device's power state. A 2.5mm jack replaces its LEGO counterpart near the left end of the control panel, supported by two 3D-printed. An LED is placed behind the ruby near the right end of the control panel as a power indicator. A 57mm speaker is hidden within the speaker assembly, adhered to a 3D-printed plate at the center.

### [PCB](PCB)
The main PCB leverages an STM32F-series microcontroller and a 4-layer stackup with internal signal/power layers. It is pinned inside the rear enclosure of the amp using a spare knob from the control panel, supported by two 3D-printed bricks surrounding a USB-C port which powers the device. The auxilliary PCB with the ON/OFF switch is connected to the control panel PCB, which is connected to the main PCB via flex cable.

### [Firmware](MiniSpeaker_STM32)
Using ST's MCU software development tools, the main PCB is programmed to convert an analog signal to digital, apply some filtering and effects, and convert the digital signal back to analog where it is driven onto the output. Peripheral controls are also read via A/D converter and are implemented as volume and effects control parameters. Low-power mode, "standby," is leveraged to support the ON/OFF switch, events of which are read via GPIO and Wake-Up pins near-simultaneously (GPIO is hardware delayed).
