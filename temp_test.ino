
#include <MPU6050_tockn.h>
#include <Wire.h>

int left_step=9,left_dir=8,right_step=10,right_dir=11;

MPU6050 mpu6050(Wire);
long tim;
void setup() {
  pinMode(left_step,OUTPUT);
  pinMode(right_step,OUTPUT);
  pinMode(left_dir,OUTPUT);
  pinMode(right_dir,OUTPUT);
  Serial.begin(115200);
  Wire.begin();

  noInterrupts();           // disable all interrupts

  TCCR1A = 0;       //timer 1 lower byte reset

  TCCR1B = 0;       //timer 1 higher byte resert

  TCNT1  = 0;       //Timer 1 count value is set to 0

  

  TCCR1B |= (1 << WGM12);   // CTC mode Clear timer after match.

  TCCR1B |= (1 << CS11);    // 8 prescaler 

  OCR1A = 1599; //Timer starts from 0, for 1millis second
  TIMSK1 |= (1 << OCIE1A);  // enable timer compare interrupt
  PORTB&=0b11111110;// PORTB = 8-bit0, 9-bit1,10-bit2, 11-bit3
  //digitalWrite(left_dir,0);
  //digitalWrite(right_dir,1);
  
  //mpu6050.begin();
  //mpu6050.calcGyroOffsets(false);
  interrupts();             // enable all interrupts

}

void loop() {

//  
//  tim=micros();
//  mpu6050.update();
//  Serial.print("\tangleY : ");
//  Serial.println(mpu6050.getAngleY());
//  //delay(10);
//  Serial.println(micros()-tim);
}

ISR(TIMER1_COMPA_vect){
  PORTB^=0b0000110;
}
