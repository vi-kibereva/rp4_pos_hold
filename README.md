# Position hold for FPV drone
This project implements a position hold system for FPV drone, based on the data from barometer, gyroscope and optical flow from down-facing camera, without using GPS.\
##System architecture:
### Sensors:
   Data from the drone's gyroscope and barometer is passed to Raspberry Pi, connected to the drone,  via MultiWii Serial Protocol.\
   The camera is connected directly to Raspberry Pi.\
### Determining drone's position

### Controlling drone's position
  Displacement vector calculated in optical flow algorithm is passed into PID controller, which calculates neccessary corrections(pitch and roll) as a weighted sum of proportional, integral and derivative parts.\
  The correction is then passed to flight controller via MSP, specifically MPS_SET_RAW_RC, which emulates the movement of the sticks on the RC transmitter.\
  When the drone is in MSP_OVERRIDE mode flight controller ignores roll and throttle input from the RC transmitter and instead executes commands from Raspberry Pi.\
  
