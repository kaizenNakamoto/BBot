
#include <MPU6050_tockn.h>
#include <Wire.h>
// #include <avr/wdt.h>

unsigned long tim=0;
float t11=0,temps;
volatile int left_motor, throttle_left_motor, throttle_counter_left_motor, throttle_left_motor_memory;
volatile int right_motor, throttle_right_motor, throttle_counter_right_motor, throttle_right_motor_memory;
int left_step=8,left_dir=9,right_step=10,right_dir=11,temp,pid_out;
float pres_error,prev_error;
float Ki,Kp,Kd;
float start_angle,pid_error,pid_i,pid_p,pid_d;
MPU6050 mpu6050(Wire);
void setup() {
  pinMode(left_step,OUTPUT);
  pinMode(right_step,OUTPUT);
  pinMode(left_dir,OUTPUT);
  pinMode(right_dir,OUTPUT);
  pinMode(13,OUTPUT);
  Serial.begin(230400);
  Wire.begin();
  Kp= 15;
  Ki= 1;
  Kd= 30;
  TCCR2A = 0;                                                               //Timer 2 Counter Register is reset
  TCCR2B = 0;                                                               //TImer 2 Counter Register is reset
  TIMSK2 |= (1 << OCIE2A);                                                  //Set the interupt enable bit OCIE2A in the TIMSK2 register
  TCCR2B |= (1 << CS21);                                                    //Set the CS21 bit in the TCCRB register to set the prescaler to 8
  OCR2A = 186;                                                              //The compare register is set to 186 => 93us / (1s / (16.000.000MHz / 8)) - 1
  TCCR2A |= (1 << WGM21);                                                   //Set counter 2 to CTC (clear timer on compare) mode
  digitalWrite(13,0);
  //Calibrating to get starting values.
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  //interrupts();              // enable all interrupts

// Get Start Angle

  for(int i = 1 ; i< 1000 ; i ++)
  { mpu6050.update();
    temps= mpu6050.getAngleY();
    if(i<51)
      continue;
    else
    start_angle+=temps;
    }
  start_angle/=950;
 Serial.println(start_angle);
 digitalWrite(13,1);
  delay(1000);
}


void loop() {
 tim=millis();
 mpu6050.update();
 //Serial.println(mpu6050.getAngleY());
 pres_error=mpu6050.getAngleY()-start_angle;
 Serial.println(pres_error);
  
  //Forward motion is required for negative angle
  //Backward motion is required for positive angle
 
  
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

  //Tolerance Range
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
  //Serial.print("ERROR = ");Serial.println(pid_out);
  
  while(millis()<tim+5){mpu6050.update();
  t11=mpu6050.getAngleY();
  //Serial.println("....................CBD.........................");
 }
//Serial.println(micros()-tim);
//Serial.flush();
}

ISR(TIMER2_COMPA_vect){
  //Left motor pulse calculations
  
  throttle_counter_left_motor ++;                                          
  if(throttle_counter_left_motor > throttle_left_motor_memory){             
    throttle_counter_left_motor = 0;                                       
    throttle_left_motor_memory = throttle_left_motor;                       
    if(throttle_left_motor_memory < 0){                             
      PORTB &= 0b11111101; //PORT B 1 1 1 1 1 1 1 1                                            
      throttle_left_motor_memory *= -1;                                     
    }
    else PORTB |= 0b00000010; 
  }                                              
  else if(throttle_counter_left_motor == 1)PORTB |= 0b00000001;             
  else if(throttle_counter_left_motor == 2)PORTB &= 0b11111110;              
  //right motor pulse calculations
  throttle_counter_right_motor ++;                                          
  if(throttle_counter_right_motor > throttle_right_motor_memory){           
    throttle_counter_right_motor = 0;                                      
    throttle_right_motor_memory = throttle_right_motor;                    
    if(throttle_right_motor_memory < 0){                                   
      PORTB |= 0b00001000;                                               
      throttle_right_motor_memory *= -1;                                    
    }
    else PORTB &= 0b11110111;                                               
  }

  else if(throttle_counter_right_motor == 1)PORTB |= 0b00000100;           
  else if(throttle_counter_right_motor == 2)PORTB &= 0b11111011;                                            
 
 }
