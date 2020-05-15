# INTRODUCTION
 The YAQP flight controller is an Arduino UNO based flight controller capable of flying in angle or rate mode with automatic ESC calibration. Support for GPS flying may be added in the future.  
 **FLight controller uses an X-configuration**
 # DEPENDENCIES
+ [Wire](https://www.arduino.cc/en/Reference/Wire)
+ [PID](https://github.com/DonnyCraft1/PIDArduino)
# CONNECTIONS
## ESCs 
 esc 1 to pin4 (front L -CW)  
 esc 2 to pin5 (front R -CCW)  
 esc 3 to pin6 (rear R  -CW)  
 esc 4 to pin7 (rear L  -CW)  
  
 ## GYROSCOPE(MPU-6050)
  SDA to A4(arduino)  
  SCL to A5(arduino)  
  3.3v to 3.3v(arduino)  
  GND to GND(arduino)  
  
 ## RECEIVER
  CHANNEL 1 to Pin8  
  CHANNEL 2 to Pin9  
  CHANNEL 3 to Pin10  
  CHANNEL 4 to Pin11   
  
# START AND STOP SEQUENCE
## START
1. left stick(bottom right) & right stick (bottom left)  
2. left stick(bottom left) & right stick (bottom right)  
3. Left stick (center bottom) & right stick (center)  
