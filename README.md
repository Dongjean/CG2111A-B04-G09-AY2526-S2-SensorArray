#**Pins We Used**
###E-Stop Pin: PD2 (Digital Pin 19), INT0

###Colour Sensor:
  S0: PA0 (Digital Pin 22)
  S1: PA1 (Digital Pin 23)
  S2: PA2 (Digital Pin 24)
  S3: PA3 (Digital Pin 25)
  Out: PE5 (Digital Pin 3), PWM

#**How to Compile Arduino**
arduino-cli compile -b arduino:avr:mega -p /dev/ttyACM0 --upload sensor_miniproject_template.ino

#**How to Activate the Environment**
in the base directory:
source env/bin/activate

#**How to Run the Array**
in the base directory:
python3 pi_serial.py
