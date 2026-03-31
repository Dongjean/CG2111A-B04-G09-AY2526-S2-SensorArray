#**Pins We Used**
Encoder right: 53
Encoder left: 52

Colour sensor
Output: A8 (PWM)
S0: 22
S1: 23
S2: 24
S3: 25

Robot arm
Base: A9
Right shoulder: A10
Left shoulder: A11
Gripper: A12

Estop: 19 (INT2)

#**How to Compile Arduino**
arduino-cli compile -b arduino:avr:mega -p /dev/ttyACM0 --upload sensor_miniproject_template.ino

#**How to Activate the Environment**
in the base directory:
source env/bin/activate

#**How to Run the Array**
in the base directory:
python3 pi_serial.py
