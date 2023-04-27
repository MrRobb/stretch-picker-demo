
# Stretch Picker Demo

## Installation

### Clone and change directory

```bash
git clone https://github.com/MrRobb/stretch-picker-demo.git
cd stretch-picker-demo
```

### Install requirements

```bash
pip install -r requirements
```

## Usage

### Setup the robot

#### Kill controller process

```bash
pkill -f stretch_xbox*
```

#### Check system

```bash
stretch_robot_system_check.py
```

#### Calibrate (if necessary)

```bash
stretch_robot_home.py
```

#### Run

Build the package:

```bash
# On your workspace
catkin_make && source devel/setup.sh
```

Run the script:

```bash
rosrun picker-demo demo.py
```

## Troubleshooting

### If the dynamixel motors fail

```bash
stretch_robot_dynamixel_reboot.py
```