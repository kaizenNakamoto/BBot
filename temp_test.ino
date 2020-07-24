
#include <MPU6050_tockn.h>
#include <Wire.h>
#define sample 0.004
long temps=0;
float t11=0;
int left_motor, throttle_left_motor, throttle_counter_left_motor, throttle_left_motor_memory;
int right_motor, throttle_right_motor, throttle_counter_right_motor, throttle_right_motor_memory;
int stable,motor_out,motor_count,motor_mem;
int left_step=8,left_dir=9,right_step=10,right_dir=11,temp,pid_out,pres_error,prev_error;
float Ki,Kp,Kd;
int windup,toggle=0;
float start_angle,pid_error,pid_i,pid_p,pid_d;
int start,first=1;
MPU6050 mpu6050(Wire);
long tim;
void setup() {
  pinMode(left_step,OUTPUT);
  pinMode(right_step,OUTPUT);
  pinMode(left_dir,OUTPUT);
  pinMode(right_dir,OUTPUT);
  pinMode(13,OUTPUT);
  Serial.begin(115200);
  Wire.begin();
  Kp=15 ;
  Ki=1;
  Kd=30;
  TCCR2A = 0;                                                               //Make sure that the TCCR2A register is set to zero
  TCCR2B = 0;                                                               //Make sure that the TCCR2A register is set to zero
  TIMSK2 |= (1 << OCIE2A);                                                  //Set the interupt enable bit OCIE2A in the TIMSK2 register
  TCCR2B |= (1 << CS21);                                                    //Set the CS21 bit in the TCCRB register to set the prescaler to 8
  OCR2A = 186;                                                               //The compare register is set to 39 => 20us / (1s / (16.000.000MHz / 8)) - 1
  TCCR2A |= (1 << WGM21);                                                                  //Set counter 2 to CTC (clear timer on compare) mode
  
  //PORTB&=0b11110111;// PORTB = 8-bit0, 9-bit1,10-bit2, 11-bit3   
  digitalWrite(13,0);

  //Calibrating to get starting values.
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  //interrupts();              // enable all interrupts
  tim=millis()+3000;
  temp=300;
  start_angle=0;
  while(millis()<tim){ // give 1 seconds to start
  mpu6050.update();
  start_angle=mpu6050.getAngleY();
  }
  digitalWrite(13,1);
  delay(2000);
  //digitalWrite(13,0);
}

//Gyroscope- resitant vibtion it drifts
//ACCL - accurate at 0 viration

void loop() {
  
 tim=micros();
  mpu6050.update();
  Serial.println(mpu6050.getAngleY());
  pres_error=mpu6050.getAngleY()-start_angle;
//  Serial.println(pres_error);
  //Forward is negative angle
  //Backward is positive angle
  //Serial.print("\tangleY : ");
  //Serial.println(pres_error);
  pid_p=Kp*pres_error;
  pid_i+=(Ki*pres_error);
  if(pid_i>400)
  pid_i=400;
  else if(pid_i<-400)
  pid_i=-400;
  pid_d=Kd*((pres_error-prev_error));
  pid_error=pid_p+pid_i+pid_d;
  if(pid_error>400)
  pid_error=400;
  else if(pid_error<-400)
  pid_error=-400;
  prev_error=pres_error;
  //Serial.println(pid_error);
  
  if(pid_error<5 && pid_error>-5){
  digitalWrite(13,0);
  pid_error=0;
  pid_i=0;
  }
  else
  digitalWrite(13,1);

// LINEAR CALUCLATION (RANGE : 400 - 392) 
   if(pid_error > 0)pid_error = 405 - (1/(pid_error + 9)) * 5500;       
  else if(pid_error < 0)pid_error = -405 - (1/(pid_error - 9)) * 5500;


   // pid out MAX (400- 392) = 8

   
    
  if(pid_error > 0)pid_out = 400 - pid_error;          //DutyCycle=(20us/397*20us)
  else if(pid_error < 0)pid_out = -400 - pid_error;
  else pid_out = 0;
  throttle_left_motor=pid_out;
  throttle_right_motor=pid_out;
  Serial.print("ERROR = ");Serial.println(pid_out);
  while(micros()<tim+12500){mpu6050.update();
  t11=mpu6050.getAngleY();
  }
}

ISR(TIMER2_COMPA_vect){
  //Left motor pulse calculations
  
  throttle_counter_left_motor ++;                                         1  //Increase the throttle_counter_left_motor variable by 1 every time this routine is executed
  if(throttle_counter_left_motor > throttle_left_motor_memory){             //If the number of loops is larger then the throttle_left_motor_memory variable
    throttle_counter_left_motor = 0;                                        //Reset the throttle_counter_left_motor variable
    throttle_left_motor_memory = throttle_left_motor;                       //Load the next throttle_left_motor variable
    if(throttle_left_motor_memory < 0){                              //If the throttle_left_motor_memory is negative
      PORTB &= 0b11111101; //PORT B 1 1 1 1 1 1 1 1                                                 //Set output 3 low to reverse the direction of the stepper controller
      throttle_left_motor_memory *= -1;                                     //Invert the throttle_left_motor_memory variable
    }
    else PORTB |= 0b00000010; 
                                                   //Set output 3 high for a forward direction of the stepper motor
  }
  else if(throttle_counter_left_motor == 1)PORTB |= 0b00000001;             //Set output 2 high to create a pulse for the stepper controller
  else if(throttle_counter_left_motor == 2)PORTB &= 0b11111110;             //Set output 2 low because the pulse only has to last for 20us 
  
  //right motor pulse calculations
  throttle_counter_right_motor ++;                                          //Increase the throttle_counter_right_motor variable by 1 every time the routine is executed
  if(throttle_counter_right_motor > throttle_right_motor_memory){           //If the number of loops is larger then the throttle_right_motor_memory variable
    throttle_counter_right_motor = 0;                                       //Reset the throttle_counter_right_motor variable
    throttle_right_motor_memory = throttle_right_motor;                     //Load the next throttle_right_motor variable
    if(throttle_right_motor_memory < 0){                                    //If the throttle_right_motor_memory is negative
      PORTB |= 0b00001000;                                               //Set output 5 low to reverse the direction of the stepper controller
      throttle_right_motor_memory *= -1;                                    //Invert the throttle_right_motor_memory variable
    }
    else PORTB &= 0b11110111;                                               //Set output 5 high for a forward direction of the stepper motor
  }







































































































































































  
  else if(throttle_counter_right_motor == 1)PORTB |= 0b00000100;            //Set output 4 high to create a pulse for the stepper controller
  else if(throttle_counter_right_motor == 2)PORTB &= 0b11111011;            //Set output 4 low because the pulse only has to last for 20us
}
