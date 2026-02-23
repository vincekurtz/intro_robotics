# Introduction to Robotics: CSE 375/475

This repository contains source code for the DePaul University course taught in
Winter 2026. 

> [!NOTE]
> This branch is for controlling the UR3e arms. For the turtlebots, see [this
> branch](https://github.com/vincekurtz/intro_robotics/tree/main).

## Setup

Clone this repository:
```
git clone -b ur3e https://github.com/vincekurtz/intro_robotics
cd intro_robotics
```

Create a virtual environment:
```
python3 -m venv .venv
```

Enter the virtual environment:
```
source .venv/bin/activate
```

Install prerequisite packages:
```
pip install -r requirements.txt
```

## Usage

- Turn on the robot.
- Connect the robot and PC directly with an Ethernet cable.
- Configure the wired connection on the PC (IPv4 Manual, Address
        `192.168.1.101`, Netmask `255.255.255.0`).
- Switch from "local" to "remote control" on the Teach Pendant.
- Check the robot's IP address on the Teach Pendant (hamburger menu, "About").
- Check that we can `ping` the robot from the PC.
- Enter the virtual environment.
- Run an example script, e.g., `python3 example_joint_control.py`.
