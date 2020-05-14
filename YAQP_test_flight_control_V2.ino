// front is red of the drone
// you might need to switch the y and z (letters not the whole thing)
// x axis == pitch
// y axis == roll
// z axis == yaw
#include <MadgwickAHRS.h>
#include <PIDController.h>// https://github.com/DonnyCraft1/PIDArduino
#include <Servo.h>
#include <Wire.h>
#include<math.h>
#define M_PI  3.14159265358979323846  /* pi */
#define TO_DEG 57.2957795131
#define TO_RAD 0.0174533             //multiply by 

//Yemi- For refrence on ports manipulation(those weird ass functions that are using bit operations like PINB or summ)https://www.arduino.cc/en/Reference/PortManipulation
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
int throttle,receiver_yaw, receiver_pitch, receiver_roll;

//Declaring some global variables 
float error_pitch, error_roll;
double acc_error;  
float acc_x,acc_y,acc_z,temperature; 
float acc_pitch, acc_roll, acc_resultant;                                
int gyro_x, gyro_y, gyro_z;                                            //raw gyro input
float gyro_x_dps, gyro_y_dps, gyro_z_dps;
long gyro_x_offset, gyro_y_offset, gyro_z_offset;
boolean set_gyro_angles;
float yaw, pitch, roll;
unsigned long prevInterval, interval;
 
//PID declartions                      // the drone sorta flew at P=0.5, i=0, k=80
float p_k =0.6;   //p gain    0.5 // brooking says increase by 0.2 starting from 0.2
float i_k =0.0;   //i gain   0.02
float d_k =100;   //d gain   80 65

float yaw_p_k =1;   //p gain 3
float yaw_i_k =0.0;   //i gain 0.02
float yaw_d_k =0.0;   //d gain

float yaw_PID,pitch_PID,roll_PID;
double max_PID=250, min_PID=-250;

//variables for PID calculation
float pid_error;
float roll_PID_I, pid_roll_setpoint, gyro_roll_input, roll_prev_d_error;
float pitch_PID_I, pid_pitch_setpoint, gyro_pitch_input, pitch_prev_d_error;
float yaw_PID_I, pid_yaw_setpoint, gyro_yaw_input, yaw_prev_d_error;

//initializing receiver variables for the rate mode
int yaw_rate=0;
int roll_rate=0;
int pitch_rate=0;

// creating an instance of the Madgwick quaternion sensor fusion algo 
Madgwick imu;

//initializing receiver variables for the angle mode. 
//yaw_rate is still going to be used in the angle mode
double roll_angle=0;
double pitch_angle=0;

Servo ESC1;               // create servo object to control the ESC
Servo ESC2;
Servo ESC3;
Servo ESC4;

PIDController pid_y;
PIDController pid_p;
PIDController pid_r;

int motor1, motor2, motor3, motor4;

bool sssbool;            //start stop sequence boolean. for the while loop in the startup sequence
byte on_off=0;           //0 for on, 1 for on


int test_roll;

void setup() {

 Serial.begin(115200);
  
 setup_mpu_6050_registers(); //Setting Up the MPU-6050 sensor
 
 //setting up the escs 
 ESC1.attach(4,1000,2000); // attach pin4 to esc 1
 ESC2.attach(5,1000,2000); // attach pin5 to esc 2
 ESC3.attach(6,1000,2000); // attach pin6 to esc 3
 ESC4.attach(7,1000,2000); // attach pin7 to esc 4

//esc calibration - not sure if this is neccessary to run everytime the drone starts but it here ig lol
 ESC1.writeMicroseconds(2000);
 ESC2.writeMicroseconds(2000);
 ESC3.writeMicroseconds(2000);
 ESC4.writeMicroseconds(2000);
 delay(1000);    
 ESC1.writeMicroseconds(1000);
 ESC2.writeMicroseconds(1000);
 ESC3.writeMicroseconds(1000);
 ESC4.writeMicroseconds(1000);
 //end of esc calibration
 
//using pin 3 to power the receiver
  pinMode(3, OUTPUT);    // sets the digital pin 3 as output
  digitalWrite(3, HIGH); // sets the digital pin 3 on
//setting up the receiver interrupts
  PCICR |= (1 << PCIE0);                                       //Set PCIE0 to enable PCMSK0 scan.
  PCMSK0 |= (1 << PCINT0);                                     //Set PCINT0 (digital input 8) to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT1);                                     //Set PCINT1 (digital input 9)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT2);                                     //Set PCINT2 (digital input 10)to trigger an interrupt on state change.
  PCMSK0 |= (1 << PCINT3);                                     //Set PCINT3 (digital input 11)to trigger an interrupt on state change.

//some PID stuff...
  pid_y.begin();
  pid_p.begin();
  pid_r.begin();
  
  pid_y.tune(yaw_p_k,yaw_i_k,yaw_d_k);
  pid_p.tune(p_k,i_k,d_k);
  pid_r.tune(p_k,i_k,d_k);
  
  pid_y.limit(min_PID,max_PID);
  pid_p.limit(min_PID,max_PID);
  pid_r.limit(min_PID,max_PID);

  //loop_timer = millis() +4;
       
}

void loop() {  
/*
 * This is the start-stop sequence. Down-right-left on left control stick, down-left-right on right control stick
 * To reset or get out of this loop set pitch on controller to 2000us or full pitch
 */ 
if(throttle==1000 && receiver_yaw == 2000 && receiver_roll ==1000 && receiver_pitch==1000){
  sssbool=true;//set while loop breaker to true or 1 sssbool
  
  stop_motors(); //didnt want to crowd the place
  
  while(sssbool){               //wait for next pair of commands 
    
  if(throttle==1000 && receiver_yaw == 1000 && receiver_roll ==2000 && receiver_pitch==1000){
    
    while(sssbool){            //wait for next pair of commands    
         
    if(throttle==1000 && receiver_yaw == 1500 && receiver_roll ==1500 && receiver_pitch==1500){
      
      if(on_off==1){         // if on, turn off
        on_off=0;
        sssbool=false;
        Serial.println("Drone Off");
        }
        
      else if(on_off==0){    // if off, turn on
        on_off=1;
        sssbool=false;
        Serial.println("Drone On");
        } //end of startup routine
      }
      if(receiver_pitch==2000)sssbool=false;               //Reset the startup sequence
     }//2nd while loop end
    }
    if(receiver_pitch==2000)sssbool=false;                //Reset the startup sequence
   }//1st while loop end
  }
  
//on_off=1; //*********************************************** D E L E T E   A F T E R   D E B U G G I N G *********************************************************
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//If on_off is 1 keep the propellers running
if(on_off==1){ 
  

  yaw_rate=map(receiver_yaw,1000,2000,-50, 50);   //the yaw dps values is flipped
  roll_angle=map(receiver_roll,1000,2000,-25,25);    
  pitch_angle=map(receiver_pitch,1000,2000,-25,25);   //the pitch values in the yaqp_imu(i2cdevlib) is flipped
  
  pid_yaw_setpoint=yaw_rate;
  pid_pitch_setpoint=pitch_angle;
  pid_roll_setpoint=roll_angle;
    
 //Calculating time intervals
  interval = (millis() - prevInterval);                     
  prevInterval = millis();  
   
// 4ms is 250hz. I am using this frequency, because the arduino runs at about 333hz with this code. This frequency is also reflected in the madgwickahrs.cpp file
if(interval <= 4){  //it is less than because if the arduino has run out of time, you dont want to slow it down further by running this block of code again
  
   imu_rate_mode_update();
    
   imu.updateIMU(gyro_x_dps, gyro_y_dps, gyro_z_dps, acc_x, acc_y, acc_z);// no need to convert to radians here. the library does it. if you add it here, it'll just be extra slow


    // to be used for angle mode
     pid_y.setpoint(yaw_rate);
     yaw_PID= pid_y.compute((gyro_z_dps-0.60)*-1);

     pid_p.setpoint(pitch_angle);
     pitch_PID= pid_p.compute(imu.getPitch()+3.29); // compensating for error
 
     pid_r.setpoint(roll_angle);
     //test_roll=(int)imu.getRoll()-0.99;
     roll_PID= pid_r.compute(imu.getRoll()-0.99); //imu.getRoll()-0.99
Serial.println(roll_PID);
/////////////testing out the new PID function
  //  calculate_pid();
    //Setting ESC write boundaries to 1000 - 2000 us pulses
  
    if (throttle > 1800) {                   //We need some room to keep full control at full throttle.
      throttle = 1800;
      }  


    motor1 = (throttle * 1.0007) - pitch_PID + roll_PID - yaw_PID;
      if(motor1<1000){
      motor1=1000;                               //Keep the motors running
      }
      if(motor1>2000){
      motor1=2000;
      }    
    motor2 = (throttle * 0.97640) - pitch_PID - roll_PID + yaw_PID;
      if(motor2<1000){
      motor2=1000;                              //Keep the motors running
      }
      if(motor2>2000){
      motor2=2000;
      }
    motor3 = (throttle * 0.9834) + pitch_PID - roll_PID - yaw_PID;
      if(motor3<1000){
      motor3=1000;                             //Keep the motors running
      }
    if(motor3>2000){
      motor3=2000;
      }
    motor4 = (throttle * 1.0001) + pitch_PID + roll_PID + yaw_PID;
      if(motor4<1000){
      motor4=1000;                            //Keep the motors running
      }
      if(motor4>2000){
      motor4=2000;
      }

if(throttle==1000){
  stop_motors();
  }
else if(throttle>=1010){
     ESC1.writeMicroseconds(motor1);  //(front-left - CW)
     ESC2.writeMicroseconds(motor2);  //(front-right - CCW)
     ESC3.writeMicroseconds(motor3);  //(rear-right - CW)
     ESC4.writeMicroseconds(motor4);  //(rear-left - CCW)
} 
/*
     Serial.print(motor1);
     Serial.print(", ");
     Serial.print(motor2);
     Serial.print(", ");
     Serial.print( motor3);
     Serial.print(", ");
     Serial.println(motor4);
     */
  }
 }//end of 250Hz armed drone loop

//If on_off is 0, turn the propellers off (which is at 1000us pulse in this case)
 if(on_off==0){
 ESC1.writeMicroseconds(1000);
 ESC2.writeMicroseconds(1000);
 ESC3.writeMicroseconds(1000);
 ESC4.writeMicroseconds(1000);
  }
/*while(loop_timer > millis()){
do nothing... we are waiting for 
}

loop_timer+=4;
*/
 
} //end of loop


void calculate_pid(){
  //Roll calculations
  pid_error = (imu.getRoll()-0.65) - pid_roll_setpoint;
  roll_PID_I += i_k * pid_error;
  
  if(roll_PID_I > max_PID){
    roll_PID_I = max_PID;
    }
  else if(roll_PID_I < min_PID){
    roll_PID_I = min_PID;
  }
  
  roll_PID = p_k * pid_error + roll_PID_I + d_k * (pid_error - roll_prev_d_error);
  
  if(roll_PID > max_PID){
    roll_PID = max_PID;
  }
  else if(roll_PID < min_PID){
    roll_PID = min_PID;
  }
  
  roll_prev_d_error = pid_error;
  
  //Pitch calculations
  pid_error = (imu.getPitch()+3.09) - pid_pitch_setpoint;
  pitch_PID_I += i_k * pid_error;
  
  if(pitch_PID_I > max_PID){
    pitch_PID_I = max_PID;
  }
  else if(pitch_PID_I < min_PID){
    pitch_PID_I = min_PID;
  }
  
  pitch_PID = p_k * pid_error + pitch_PID_I + d_k * (pid_error - pitch_prev_d_error);
  
  if(pitch_PID > max_PID){
    pitch_PID = max_PID;
  }
  else if(pitch_PID < min_PID){
    pitch_PID = min_PID;
  }
    
  pitch_prev_d_error = pid_error;
    
  //Yaw calculations
  pid_error = ((gyro_z_dps-0.60)*-1) - pid_yaw_setpoint;
  yaw_PID_I += yaw_i_k * pid_error;
  
  if(yaw_PID_I > max_PID){
    yaw_PID_I = max_PID;
  }
  else if(yaw_PID_I < min_PID){
    yaw_PID_I = min_PID;
  }
  
  yaw_PID = yaw_p_k * pid_error + yaw_PID_I + yaw_d_k * (pid_error - yaw_prev_d_error);
  
  if(yaw_PID > max_PID){
    yaw_PID = max_PID;
  }
  else if(yaw_PID < min_PID){
    yaw_PID = min_PID;
  }
    
  yaw_prev_d_error = pid_error;
}


void stop_motors(){
 ESC1.writeMicroseconds(1000);
 ESC2.writeMicroseconds(1000);
 ESC3.writeMicroseconds(1000);
 ESC4.writeMicroseconds(1000);
 }
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void imu_rate_mode_setup(){

  Wire.begin();                                                        //Start I2C as master
 
  pinMode(13, OUTPUT);                                                 //Set output 13 (LED) as output
  
  setup_mpu_6050_registers();                                          //Setup the registers of the MPU-6050 (500dfs / +/-8g) and start the gyro

  digitalWrite(13, HIGH);                                              //Set digital output 13 high to indicate startup

  gyro_offset();   
}

void imu_rate_mode_update(){
  read_mpu_6050_data();                                                   //Read the raw acc and gyro data from the MPU-6050

  gyro_x -= gyro_x_offset;                                                //Subtract the offset calibration value from the raw gyro_x value
  gyro_y -= gyro_y_offset;                                                //Subtract the offset calibration value from the raw gyro_y value
  gyro_z -= gyro_z_offset;                                                //Subtract the offset calibration value from the raw gyro_z value

 //Converting gyro values to degrees per second   
  gyro_x_dps = gyro_x/65.5; 
  gyro_y_dps = gyro_y/65.5;
  gyro_z_dps = gyro_z/65.5;
 /*
    Serial.print(gyro_x_dps);
    Serial.print(" ");
    Serial.print(gyro_y_dps);
    Serial.print(" ");
    Serial.println(gyro_z_dps);
 */
}

void gyro_offset(){
 /*
  for (int cal_int = 0; cal_int < 2000 ; cal_int ++){                  //Calculate average for 2000 values
    read_mpu_6050_data();                                              //Read the raw acc and gyro data from the MPU-6050
    gyro_x_offset += gyro_x;                                              //Add the gyro x-axis offset to the gyro_x_cal variable
    gyro_y_offset += gyro_y;                                              //Add the gyro y-axis offset to the gyro_y_cal variable
    gyro_z_offset += gyro_z;                                              //Add the gyro z-axis offset to the gyro_z_cal variable
    delay(3);                                                          //Delay 3us to simulate the 250Hz program loop
  }
  gyro_x_offset /= 2000;                                                  //Divide the gyro_x_cal variable by 2000 to get the avarage offset
  gyro_y_offset /= 2000;                                                  //Divide the gyro_y_cal variable by 2000 to get the avarage offset
  gyro_z_offset /= 2000;                                                  //Divide the gyro_z_cal variable by 2000 to get the avarage offset
*/
//Yemi- these are the avg values that i got. We don't have to waste time at startup
  gyro_x_offset = 15;                                           
  gyro_y_offset = 2;                                                  
  gyro_z_offset = 0;                                                  
  }
  
void read_mpu_6050_data(){                                             //Subroutine for reading the raw gyro and accelerometer data
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x3B);                                                    //Send the requested starting register
  Wire.endTransmission();                                              //End the transmission
  Wire.requestFrom(0x68,14);                                           //Request 14 bytes from the MPU-6050
  while(Wire.available() < 14);                                        //Wait until all the bytes are received
  acc_x = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_x variable
  acc_y = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_y variable
  acc_z = Wire.read()<<8|Wire.read();                                  //Add the low and high byte to the acc_z variable
  temperature = Wire.read()<<8|Wire.read();                            //Add the low and high byte to the temperature variable
  
  gyro_x = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_x variable
  gyro_y = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_y variable
  gyro_z = Wire.read()<<8|Wire.read();                                 //Add the low and high byte to the gyro_z variable

}

void setup_mpu_6050_registers(){
  //Activate the MPU-6050
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x6B);                                                    //Send the requested starting register
  Wire.write(0x00);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the accelerometer (+/-8g)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1C);                                                    //Send the requested starting register
  Wire.write(0x10);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
  //Configure the gyro (500dps full scale)
  Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
  Wire.write(0x1B);                                                    //Send the requested starting register
  Wire.write(0x08);                                                    //Set the requested starting register
  Wire.endTransmission();                                              //End the transmission
}




///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//This routine is called every time input 8, 9, 10 or 11 changed state
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ISR(PCINT0_vect){                         //ISR() is the function that Arduino uses in AttachInterrupt())
  current_time = micros();                //yemi- we are checking to see if the pulse when interupted is low or high (keep in mind that this is pwm not analog so there's so the pulse is either high or low, no inbetween)
                                          //it only interrupts when theres a change in the input
  //Channel 1=========================================
  if(PINB & B00000001){                                        //Is input 8 high?                   //yemi- same as digital read()
    if(last_channel_1 == 0){                                   //Input 8 changed from 0 to 1
      last_channel_1 = 1;                                      //Remember current input state
      timer_1 = current_time;                                  //Set timer_1 to current_time        //yemi- used to start time calculation of the current pulse( when the value from interrupt changes from 0-1.)
    }
  }
  else if(last_channel_1 == 1){                                //Input 8 is not high and changed from 1 to 0
    last_channel_1 = 0;                                        //Remember current input state
    receiver_input_channel_1 = current_time - timer_1;         //Channel 1 is current_time - timer_1           // calculate length of pulse (after the pulse has ended)
  }

if(receiver_input_channel_1 > 1964){                      // here I am making sure the pulses dont go higher or lower than the boundaries. 
  receiver_input_channel_1 = 1964;
  }
if(receiver_input_channel_1 < 992){
  receiver_input_channel_1 = 992;
  }
receiver_roll = map(receiver_input_channel_1,992,1964,1000,2000);

if(receiver_roll<1525&&receiver_roll>1475){                  //Yemi- giving the controller a little deadband of 16us
  receiver_roll=1500;
  }
  
                                                    //yemi- notes on pulse widths: the higher the time, the longer the duty cycle or on time or high is. at its highest value 
                                                    // think of it as longer voltage bursts
                                                     
  //Channel 2=========================================
  if(PINB & B00000010 ){                                       //Is input 9 high?
    if(last_channel_2 == 0){                                   //Input 9 changed from 0 to 1
      last_channel_2 = 1;                                      //Remember current input state
      timer_2 = current_time;                                  //Set timer_2 to current_time
    }
  }
  else if(last_channel_2 == 1){                                //Input 9 is not high and changed from 1 to 0
    last_channel_2 = 0;                                        //Remember current input state
    receiver_input_channel_2 = current_time - timer_2;         //Channel 2 is current_time - timer_2
    
if(receiver_input_channel_2 > 1964){                      // here I am making sure the pulses dont go higher or lower than the boundaries. 
  receiver_input_channel_2 = 1964;
  }
if(receiver_input_channel_2 < 1004){
  receiver_input_channel_2 = 1004;
  }

    receiver_pitch = map(receiver_input_channel_2,1004,1964,1000,2000);

    if(receiver_pitch<1520&&receiver_pitch>1480){                  //Yemi- giving the controller a little deadband of 40us
  receiver_pitch=1500;
  }
    
  }
  //Channel 3=========================================
  if(PINB & B00000100 ){                                       //Is input 10 high?
    if(last_channel_3 == 0){                                   //Input 10 changed from 0 to 1
      last_channel_3 = 1;                                      //Remember current input state
      timer_3 = current_time;                                  //Set timer_3 to current_time
    }
  }
  else if(last_channel_3 == 1){                                //Input 10 is not high and changed from 1 to 0
    last_channel_3 = 0;                                        //Remember current input state
    receiver_input_channel_3 = current_time - timer_3;         //Channel 3 is current_time - timer_3

if(receiver_input_channel_3 > 1964){                      // here I am making sure the pulses dont go higher or lower than the boundaries. 
  receiver_input_channel_3 = 1964;
  }
if(receiver_input_channel_3 < 1008){
  receiver_input_channel_3 = 1008;
  }
  
  throttle = map(receiver_input_channel_3,1008,1964,1000,2000);
  
  }
  //Channel 4=========================================
  if(PINB & B00001000 ){                                       //Is input 11 high?
    if(last_channel_4 == 0){                                   //Input 11 changed from 0 to 1
      last_channel_4 = 1;                                      //Remember current input state
      timer_4 = current_time;                                  //Set timer_4 to current_time
    }
  }
  else if(last_channel_4 == 1){                                //Input 11 is not high and changed from 1 to 0
    last_channel_4 = 0;                                        //Remember current input state
    receiver_input_channel_4 = current_time - timer_4;         //Channel 4 is current_time - timer_4

  if(receiver_input_channel_4 > 1964){                      // here I am making sure the pulses dont go higher or lower than the boundaries. 
  receiver_input_channel_4 = 1964;
  }
  if(receiver_input_channel_4 < 996){
  receiver_input_channel_4 = 996;
  }

  receiver_yaw = map(receiver_input_channel_4,996,1964,1000,2000);

   if(receiver_yaw<1520&&receiver_yaw>1480){                  //Yemi- giving the controller a little deadband of 16us
  receiver_yaw=1500;
  }
  
  
  }
} 


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//yemi- still working on
/*

//for reference, https://www.euclideanspace.com/maths/geometry/rotations/conversions/eulerToQuaternion/index.htm
float c_y,c_p,c_r,s_y,s_p,s_r;

double w,x,y,z;

void toQuaternion(double yaw_q, double pitch_q, double roll_q){
    c_y=cos((yaw_q*TO_RAD)/2);
    c_p=cos((pitch_q*TO_RAD)/2);
    c_r=cos((roll_q*TO_RAD)/2);
    s_y=sin((yaw_q*TO_RAD)/2);
    s_p=sin((pitch_q*TO_RAD)/2);
    s_r=sin((roll_q*TO_RAD)/2);
    
    w = (c_y*c_p*c_r)-(s_y*s_p*s_r);
    x = (c_y*s_p*c_r)-(s_y*c_p*s_r);
    y = (s_y*s_p*c_r)+(c_y*c_p*s_r);
    z = (s_y*c_p*c_r)+(c_y*s_p*s_r);
  
}
*/

/*   
        //-------ONLY TO BE USED IN OFFSET CALIBRATION FOR IMU AT STARTUP(WHEN THE DRONE IS NOT MOVING)---------\\
   //Accelerometer angle calculations  --- I am using this to zero out the imu angles since the drone should be stable
  //   16384 -- this is the accelerometers sensitivity scale factor (check mpu600 datasheet). ie at 16384, the acc experiences 1g of gravitational force
      
     imu_rate_mode_update();
     //this part should only be run once.
     acc_x/=16440; // 16384 16440
     acc_y/=16440;
     acc_z/=16440;
     acc_resultant = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));  //Calculate the magnitude of the accelerometer
     acc_pitch = asin(acc_x/acc_resultant)* -TO_DEG;      //Calculate the roll angle
     acc_roll = asin(acc_y/acc_resultant) * TO_DEG ;      //Calculate the pitch angle 
     Serial.print("accel roll, ");
     Serial.println(error_roll);
 //---------------------------------------------------------------------------------------------------------------------\\     

*/

/*
// to be used for rate mode

 pid_y.setpoint(yaw_rate);
 yaw_PID= pid_y.compute(gyro_z_dps);

 pid_p.setpoint(pitch_rate);
 pitch_PID= pid_p.compute(gyro_y_dps);
 
 pid_r.setpoint(roll_rate);
 roll_PID= pid_r.compute(gyro_x_dps);
*/
  /*        
   *    To be used for the rate mode when you finally implement one
  yaw_rate=map(receiver_yaw,1000,2000,-500,500);   // we are creating the yaw rate for the receiver 
  roll_rate=map(receiver_roll,1000,2000,-500,500);
  pitch_rate=map(receiver_pitch,1000,2000,-500,500);
*/
