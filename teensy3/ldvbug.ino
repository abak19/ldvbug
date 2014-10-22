/* LDVBug
 * Author: Alexander Blum
 * Version: 1.0
 * compatible with Teensy 3.x
 */
 
#define DEBUG 0
 
#define ROBOTRADIUS 126 // int ; halber Radabstand in mm
#define ROBOTTICKSPERMM 3.75 
#define PI 3.1415926535897932384626433832795

#define MOTORLV 23 // Motor Links vorwärts Pin
#define MOTORLR 22 // Motor Links rückwärts Pin
#define MOTORRV 21 // Motor Rechts vorwärts Pin
#define MOTORRR 20 // Motor Rechts rückwärts Pin



#define MOTOR_R_DREH	14
#define MOTOR_R_STROM	15
#define MOTOR_L_DREH	16
#define MOTOR_L_STROM	17


#define KONTAKT_RECHTS	24
#define KONTAKT_VORNE	25
#define KONTAKT_LINKS	26
#define BUMPER_LINKS	27
#define BUMPER_RECHTS	28

#define MUX_A 32
#define MUX_B 33
#define MUX_Y A10
#define MUX_X A12

#define IR_PWM 5

/* OPEN DRAIN OUTPUT */
#define OUTPUT_OPEN_DRAIN		7
#define NO_GPIO		8
static uint8_t analog_write_res = 8;
void analogWriteOD(uint8_t pin, int val);
void pinModeOD(uint8_t pin, uint8_t mode);



byte SETTING_NEW_MOVEMENT = 0;
//int8_t motorSpeed, motorMaxSpeed, motorLMaxSpeed, motorRMaxSpeed, motorLDirection, motorRDirection, dirL, dirR;
int8_t motorSpeed, motorLDirection, motorRDirection, dirL, dirR;
int16_t motorMaxSpeed, motorLMaxSpeed, motorRMaxSpeed;
uint8_t motorLSpeed, motorRSpeed, motorIsMoving=0, motorMain;
int16_t motorLTarget, motorRTarget, motorPhiStart, motorPhiTarget;
volatile int16_t motorLSteps, motorRSteps;




int8_t motorSetup ();

int8_t setMovement(int8_t v, int16_t d, int16_t phi);

void reverseMovement();

void stopMovement();

void move();

int8_t isMoving(void);

//void ISR_MR ();

//void ISR_ML ();




long previousMillis = 1000; 
long interval = 2000;






//initialize Moror Pins
int8_t motorSetup () {

  
  pinMode(MOTORRR, OUTPUT);
  pinMode(MOTORRV, OUTPUT);
  pinMode(MOTORLR, OUTPUT);
  pinMode(MOTORLV, OUTPUT);
  pinMode(MOTOR_L_DREH, INPUT);
  pinMode(MOTOR_R_DREH, INPUT);
  pinMode(MOTOR_L_STROM, INPUT);
  pinMode(MOTOR_R_STROM, INPUT);
  analogWriteFrequency(MOTORRV, 150);
  
  //attachInterrupt(14, ISR_MR, CHANGE);
  //attachInterrupt(16, ISR_ML, CHANGE);
  
}


int8_t setMovement(int8_t v, int16_t d, int16_t phi) {
  uint8_t r = 0;
  if (motorIsMoving) {
    if (SETTING_NEW_MOVEMENT == 1) {
      stopMovement();
    } else {
      return 1;
    }
  }
  motorIsMoving=1;
  if (v>100) v=100;
  if (v<-100) v=-100;
  if (v < 0) {
    v=-v;
    d=-d;
  }
  motorMaxSpeed = v * 2.5;
  //if (phi!=0) r = d * 180.0 / PI / phi;
  //else r=0;
  
  if(DEBUG){
    Serial.print("sM1:\tv=");
    Serial.print(v);
    Serial.print("\td=");
    Serial.print(d);
    Serial.print("\tphi=");
    Serial.println(phi);
  }
  
  
  motorLSteps = 0;
  motorRSteps = 0;
  
  motorLTarget = (d - phi / 180.0 * PI * ROBOTRADIUS ) * ROBOTTICKSPERMM;
  motorRTarget = (d + phi / 180.0 * PI * ROBOTRADIUS ) * ROBOTTICKSPERMM;
  
  dirL = motorLDirection = (motorLTarget==0) ? 0 : (motorLTarget>0) ? 1 : -1 ;
  dirR = motorRDirection = (motorRTarget==0) ? 0 : (motorRTarget>0) ? 1 : -1 ;
  
  if(DEBUG){
    Serial.print("sM2:\tmLT=");
    Serial.print(motorLTarget);
    Serial.print("\tmRT=");
    Serial.print(motorRTarget);
    Serial.print("\tdirL=");
    Serial.print(dirL);
    Serial.print("\t=dirR");
    Serial.println(dirR);
  }
  
  if (phi<0 && d>0){ //rechtskurve
    motorLMaxSpeed = motorMaxSpeed;
    motorRMaxSpeed = motorMaxSpeed * motorRTarget / motorLTarget; // motorLTarget != 0 da Vorwärtsbewegung
    motorMain = 1;
  } else if (phi<0 && d==0) {
    motorLMaxSpeed = motorMaxSpeed;
    motorRMaxSpeed = -motorMaxSpeed;
    motorMain = 0;
  } else if (phi<0 && d<0) {
    motorLMaxSpeed = motorMaxSpeed * motorLTarget / motorRTarget;
    motorRMaxSpeed = motorMaxSpeed;
    motorMain = 2;
  } else if (phi==0 && d>0) {
    motorLMaxSpeed = motorMaxSpeed;
    motorRMaxSpeed = motorMaxSpeed;
    motorMain = 0;
  } else if (phi==0 && d==0) {
    motorLMaxSpeed = 0;
    motorRMaxSpeed = 0;
    motorIsMoving=0;
    motorMain = 0;
  } else if (phi==0 && d<0) {
    motorLMaxSpeed = -motorMaxSpeed;
    motorRMaxSpeed = -motorMaxSpeed;
    motorMain = 0;
  } else if (phi>0 && d>0) {
    motorLMaxSpeed = motorMaxSpeed * motorLTarget / motorRTarget;
    motorRMaxSpeed = motorMaxSpeed;
    motorMain = 2;
  } else if (phi>0 && d==0) {
    motorLMaxSpeed = -motorMaxSpeed;
    motorRMaxSpeed = motorMaxSpeed;
    motorMain = 0;
  } else {
    motorLMaxSpeed = motorMaxSpeed;
    motorRMaxSpeed = motorMaxSpeed * motorRTarget / motorLTarget;
    motorMain = 1;
  }
  
  if(DEBUG){
    Serial.print("sM3:\tmLMS=");
    Serial.print(motorLMaxSpeed);
    Serial.print("\tmRMS=");
    Serial.print(motorRMaxSpeed);
    Serial.print("\tmM=");
    Serial.println(motorMain);
  }
  return 0;
}

void reverseMovement(){
  //Not implemented
}

void stopMovement(){
  motorIsMoving = 0;
  motorLTarget = 0;
  motorRTarget = 0;
  motorLSpeed = 0;
  motorRSpeed = 0;
  analogWrite(MOTORLV, 0); 
  analogWrite(MOTORLV, 0);
  analogWrite(MOTORRV, 0); 
  analogWrite(MOTORRV, 0);
} 

void move() {
  if (motorIsMoving) {
    int speedL,speedR;  
    float pl,pr;
    int ldiff,rdiff;
    pl = (motorLTarget!=0) ? float(motorLSteps) / float(motorLTarget) : 1;
    pr = (motorRTarget!=0) ? motorRSteps / motorRTarget : 1;
    ldiff = motorLSteps - motorLTarget;
    rdiff = motorRSteps - motorRTarget;
    speedL = motorLMaxSpeed;
    speedR = motorRMaxSpeed;
    if(speedL<0) speedL = -speedL;
    if(speedR<0) speedR = -speedR;
    dirL = motorLDirection;
    dirR = motorRDirection;
    //init
    if(DEBUG){
      Serial.print("m1:\tpl=");
      Serial.print(pl);
      Serial.print("\tpr=");
      Serial.print(pr);
      Serial.print("\tldiff=");
      Serial.print(ldiff);
      Serial.print("\trdiff=");
      Serial.print(rdiff);
      Serial.print("\tspeedL=");
      Serial.print(speedL);
      Serial.print("\tspeedR=");
      Serial.print(speedR);
      Serial.println();
    }
    
    //Ausgleichsregelung TODO
    if(motorLSteps>200 || motorRSteps>200) {
      if (motorPhiTarget<0){ //rechtskurve
        if (pl<pr*0.99) {
          speedL = motorMaxSpeed;
          speedR--;
        } else if (pr<pl*0.99) {
          speedR++;
        }
      } else if (motorPhiTarget==0) {
        if (pr<pl*0.99) {
          speedR = motorMaxSpeed;
          speedL--;
        } else if (pl<pr*0.99) {
          speedL = motorMaxSpeed;
          speedR--;
        }
      } else {
        if (pr<pl*0.99) {
          speedR = motorMaxSpeed;
          speedL--;
        } else if (pl<pr*0.99) {
          speedL++;
        }
      }
    }

    

    //anfahren + bremsen
    if (motorMain == 1 || (motorMain == 0 && ldiff<rdiff)) {
      if (motorLSteps * motorLDirection<300) { //anfahren
        speedL = speedL * (motorLSteps + 200) / 400.0;
        speedR = speedR * (motorLSteps + 200) / 400.0;
      } else if (ldiff * motorLDirection > -300) { //bremsen
        speedL = speedL * (abs(ldiff) + 200) / 400.0;
        speedR = speedR * (abs(ldiff) + 200) / 400.0;
      }
    } else {
      if (motorRSteps * motorRDirection<300) { //anfahren
        speedL = speedL * (motorRSteps + 200) / 400.0;
        speedR = speedR * (motorRSteps + 200) / 400.0;
      } else if (rdiff * motorRDirection > -300) { //bremsen
        speedL = speedL * (abs(rdiff) + 200) / 400.0;
        speedR = speedR * (abs(rdiff) + 200) / 400.0;
      }
    }
    
    if(DEBUG){
      Serial.print("m2:\tspeedL=");
      Serial.print(speedL);
      Serial.print("\tspeedR=");
      Serial.print(speedR);
      Serial.println();
    }
    
    //*/
    //stehen
    if (ldiff < 4 && ldiff > -4) {
      dirL = 0;
    }
    if (rdiff < 4 && rdiff > -4) {
      dirR = 0;
    }
    
    //rückwärts
    /*
    if (ldiff * motorLDirection >= 10) {
      dirL = -motorLDirection;
    }
    if (rdiff * motorRDirection >= 10) {
      dirR = -motorRDirection;
    }
    
    if (speedL < 0) {
      speedL = -speedL;
      dirL = -dirL;
    }
    if (speedR < 0) {
      speedR = -speedR;
      dirR = -dirR;
    }
    */
    if (ldiff > 0 && motorLTarget > 0 || ldiff < 0 && motorLTarget < 0 ) {
      dirL = 0;
    }
    if (rdiff > 0 && motorRTarget > 0 || rdiff < 0 && motorRTarget < 0 ) {
      dirR = 0;
    }
    
    
    if (speedL>5 && speedL<60) {
      speedL=60;
    }
    if (speedR>5 && speedR<60) {
      speedR=60;
    }
    
    
    if (dirL==1) {
      analogWrite(MOTORLV, speedL);
      analogWrite(MOTORLR, 0);
    } else if (dirL==0) {
      analogWrite(MOTORLV, 0);
      analogWrite(MOTORLR, 0);
    } else {
      analogWrite(MOTORLV, 0);
      analogWrite(MOTORLR, speedL);
    }
    
    if (dirR==1) {
      analogWrite(MOTORRV, speedR);
      analogWrite(MOTORRR, 0);
    } else if (dirR==0) {
      analogWrite(MOTORRV, 0);
      analogWrite(MOTORRR, 0);
    } else {
      analogWrite(MOTORRV, 0);
      analogWrite(MOTORRR, speedR);
    }
    
    
    if (dirR==0 && dirL==0) {
      motorIsMoving=0;
    }

    if(DEBUG){
      Serial.print("m3:\tspeedL=");
      Serial.print(speedL);
      Serial.print("\tspeedR=");
      Serial.print(speedR);
      Serial.print("\tdirR=");
      Serial.print(dirR);
      Serial.print("\tdirL=");
      Serial.println(dirL);
    }
  }
}

int8_t isMoving(void) {
  //TODO
  if (motorIsMoving)return 1;
  return 0;
}



//interrupt for right motor counter
void ISRMR () {
  motorRSteps += dirR;
  //motorRSteps++;
  //Serial.print("a");
}

//interrupt for left motor counter
void ISRML () {
  motorLSteps += dirL;
}

void contacts(){
  stopMovement();
  byte t=0;
  if (digitalRead(KONTAKT_LINKS)) t+=1;
  if (digitalRead(KONTAKT_VORNE)) t+=2;
  if (digitalRead(KONTAKT_RECHTS)) t+=4;
  if (digitalRead(BUMPER_LINKS)) t+=8;
  if (digitalRead(BUMPER_RECHTS)) t+=16;
  Serial.write(byte(0xEA));
  Serial.write((byte)0x04);
  Serial.write((byte)0x69);
  Serial.write(t);
  Serial.println();
}

void setup() {
  motorSetup ();
  //Serial.begin(9600);
  Serial.begin(38400);
  pinMode(KONTAKT_RECHTS, INPUT);
  pinMode(KONTAKT_VORNE, INPUT);
  pinMode(KONTAKT_LINKS, INPUT);
  pinMode(BUMPER_LINKS, INPUT);
  pinMode(BUMPER_RECHTS, INPUT);
  
  //pinModeOD(IR_PWM, OUTPUT_OPEN_DRAIN);
  
  attachInterrupt(14, ISRMR, FALLING);
  attachInterrupt(16, ISRML, FALLING);
  
  
  attachInterrupt(KONTAKT_RECHTS, contacts, FALLING);
  attachInterrupt(KONTAKT_VORNE, contacts, FALLING);
  attachInterrupt(KONTAKT_LINKS, contacts, FALLING);
  attachInterrupt(BUMPER_LINKS, contacts, RISING);
  attachInterrupt(BUMPER_RECHTS, contacts, RISING);
  
  pinMode(MUX_A, OUTPUT);
  pinMode(MUX_B, OUTPUT);
  
  
  
  interrupts();
  //NVIC_ENABLE_IRQ(IRQ_PORTB);
  //NVIC_ENABLE_IRQ(IRQ_PORTD);
  
}
void loop(){
  
  move();
  sercom();
  
  
  unsigned long currentMillis = millis();
  if(currentMillis - previousMillis > interval && interval !=0) {
    previousMillis = currentMillis;
    readSensor();
    
      byte t[5];
      t[0] = 255 - (uint8_t)((Ground[0][1] - 60) / 2.0);
      t[1] = 255 - (uint8_t)((Ground[1][1] - 60) / 2.0);
      t[2] = 255 - (uint8_t)((Ground[2][1] - 60) / 2.0);
      t[3] = 255 - (uint8_t)((Ground[3][1] - 60) / 2.0);
      t[4] = 255 - (uint8_t)((Ground[4][1] - 60) / 2.0);
      Serial.write(byte(0xEA));
      Serial.write((byte)0x08);
      Serial.write((byte)0x66);
      Serial.write(t[0]);
      Serial.write(t[1]);
      Serial.write(t[2]);
      Serial.write(t[3]);
      Serial.write(t[4]);
      Serial.println();
      byte u;
      u = (uint8_t) (BattVolt*10);
      Serial.write(byte(0xEA));
      Serial.write((byte)0x04);
      Serial.write((byte)0x69);
      Serial.write(u);
      Serial.println();
    
  }
  
  
}



int Ground[5][2];
float BattVolt=15.0;



void readSensor () {
  int x0,y0,x1,y1,x2,y2;
  pinModeOD(IR_PWM,OUTPUT);
  digitalWriteFast(IR_PWM,LOW);
  delayMicroseconds(15);
  digitalWriteFast(MUX_A,LOW);
  digitalWriteFast(MUX_B,LOW);
  delayMicroseconds(1);
  x0 = analogRead(MUX_X);
  y0 = analogRead(MUX_Y);
  digitalWriteFast(MUX_A,HIGH);
  delayMicroseconds(1);
  x1 = analogRead(MUX_X);
  y1 = analogRead(MUX_Y);
  digitalWriteFast(MUX_A,LOW);
  digitalWriteFast(MUX_B,HIGH);
  delayMicroseconds(1);
  x2 = analogRead(MUX_X);
  y2 = analogRead(MUX_Y);
  
  //digitalWriteFast(IR_PWM,HIGH);
  
  Ground[0][0] = y0;
  Ground[1][0] = x2;
  Ground[2][0] = x0;
  Ground[3][0] = x1;
  Ground[4][0] = y1;
  
  //BattVolt = 0.9 * BattVolt + 0.1 * (y2 / 1024.0 * 3.3 * 12.0 / 2.0); // 10 bit Res
  BattVolt = 0.9 * BattVolt + 0.1 * (y2 * 0.0193359375*1.333333); ///1024*3.3*6
  
  digitalWriteFast(MUX_A,LOW);
  digitalWriteFast(MUX_B,LOW);
  delayMicroseconds(140);
  x0 = analogRead(MUX_X);
  y0 = analogRead(MUX_Y);
  digitalWriteFast(MUX_A,HIGH);
  delayMicroseconds(1);
  x1 = analogRead(MUX_X);
  y1 = analogRead(MUX_Y);
  digitalWriteFast(MUX_A,LOW);
  digitalWriteFast(MUX_B,HIGH);
  delayMicroseconds(1);
  x2 = analogRead(MUX_X);
  y2 = analogRead(MUX_Y);
  
  
  
  digitalWriteFast(IR_PWM,HIGH);
  pinModeOD(IR_PWM,NO_GPIO);
  //digitalWriteFast(IR_PWM,LOW);
  
  
  
  Ground[0][1] = y0;
  Ground[1][1] = x2;
  Ground[2][1] = x0;
  Ground[3][1] = x1;
  Ground[4][1] = y1;
  
  
  if (DEBUG) {
    Serial.print("G0:\t");
    Serial.print(Ground[0][0]);
    Serial.print("\t");
    Serial.print(Ground[1][0]);
    Serial.print("\t");
    Serial.print(Ground[2][0]);
    Serial.print("\t");
    Serial.print(Ground[3][0]);
    Serial.print("\t");
    Serial.println(Ground[4][0]);
    
    Serial.print("G1:\t");
    Serial.print(Ground[0][1]);
    Serial.print("\t");
    Serial.print(Ground[1][1]);
    Serial.print("\t");
    Serial.print(Ground[2][1]);
    Serial.print("\t");
    Serial.print(Ground[3][1]);
    Serial.print("\t");
    Serial.println(Ground[4][1]);
    
    
    Serial.print("Akku:\t");
    Serial.println(BattVolt);
  }
  
  
  
}



byte incomingByte[40];
byte index1 = 0;


void sercom () {
  
  if (Serial.available() > 0) {
            while (Serial.available() > 0){
              incomingByte[index1] = Serial.read();
              
              if(incomingByte[0] == 234){
                index1 ++;
                
                if (incomingByte[1] == index1 || index1 > 37){
                  index1 = 0;
                  

                  switch (incomingByte[2]){
                    case 1:{ // drive
                      int speeed = incomingByte[3];
                      int distancee = int(incomingByte[4]<<8)+int(incomingByte[5]);
                      int phi = int(incomingByte[6]<<8)+int(incomingByte[7]);
                      //drive(speeed,distancee,phi);
                      //statusdrive = 1;
                      if(DEBUG){
                        Serial.print("s\t");
                        Serial.print(byte(speeed));
                        Serial.print("\td\t");
                        Serial.print(distancee);
                        Serial.print("\tp\t");
                        Serial.print(phi);
                        Serial.print("\n");
                      }
                      if(setMovement(byte(speeed), distancee, phi)){
                        //TODO: error, Movement in progress
                      }
                    }
                    break;
                    case 2:{ //led
                      
                    }
                    break;
                    case 3:{ // sensoren
                      switch(incomingByte[3]){
                        case 0:{ //groundsensors 101
                          readSensor ();
                          byte t[4];
                          t[0] = 255 - (uint8_t)((Ground[0][1] - 60) / 2.0);
                          t[1] = 255 - (uint8_t)((Ground[1][1] - 60) / 2.0);
                          t[2] = 255 - (uint8_t)((Ground[2][1] - 60) / 2.0);
                          t[3] = 255 - (uint8_t)((Ground[3][1] - 60) / 2.0);
                          Serial.write(byte(0xEA));
                          Serial.write((byte)0x07);
                          Serial.write((byte)0x65);
                          Serial.write(t[0]);
                          Serial.write(t[1]);
                          Serial.write(t[2]);
                          Serial.write(t[3]);
                          Serial.println();
                          
                          
                        }
                        break;
                        case 1:{//distancesensors 102
                          readSensor ();
                          byte t[5];
                          t[0] = 255 - (uint8_t)((Ground[0][1] - 60) / 2.0);
                          t[1] = 255 - (uint8_t)((Ground[1][1] - 60) / 2.0);
                          t[2] = 255 - (uint8_t)((Ground[2][1] - 60) / 2.0);
                          t[3] = 255 - (uint8_t)((Ground[3][1] - 60) / 2.0);
                          t[4] = 255 - (uint8_t)((Ground[4][1] - 60) / 2.0);
                          Serial.write(byte(0xEA));
                          Serial.write((byte)0x08);
                          Serial.write((byte)0x66);
                          Serial.write(t[0]);
                          Serial.write(t[1]);
                          Serial.write(t[2]);
                          Serial.write(t[3]);
                          Serial.write(t[4]);
                          Serial.println();
                        }
                        break;
                        case 2:{//batteryvoltage 105
                          byte t;
                          t = (uint8_t) (BattVolt*10);
                          Serial.write(byte(0xEA));
                          Serial.write((byte)0x04);
                          Serial.write((byte)0x69);
                          Serial.write(t);
                          Serial.println();
                        }
                        break;
                        case 3:{//contactswitches 104
                          byte t=0;
                          if (digitalRead(KONTAKT_LINKS)) t+=1;
                          if (digitalRead(KONTAKT_VORNE)) t+=2;
                          if (digitalRead(KONTAKT_RECHTS)) t+=4;
                          if (!digitalRead(BUMPER_LINKS)) t+=8;
                          if (!digitalRead(BUMPER_RECHTS)) t+=16;
                          Serial.write(byte(0xEA));
                          Serial.write((byte)0x04);
                          Serial.write((byte)0x69);
                          Serial.write(t);
                          Serial.println();
                        }
                        
                        break;
                      }                        
                      
                    }
                    break;
                    
                    case 4:{ // typ + status
                      switch(incomingByte[3]){
                        case 0:{
                          Serial.write(0xEA);
                          Serial.write(0x05);
                          Serial.write(0x64);
                          Serial.write(0x00);
                          Serial.write(motorIsMoving);
                          Serial.println();
                          
                          
                        }
                        break;
                      }                  
                      
                    }
                    break;
                    case 5:{ //stop
                      stopMovement();
                    }
                    break;
                    case 6:{ //...
                      
                    }
                    break;
                    case 200:{ //set interval
                      interval= incomingByte[3]<<8 + incomingByte[4];
                    }
                    case 203:{ //set movement interrupt
                      SETTING_NEW_MOVEMENT = incomingByte[3];
                    }
                //case 3:
                
                //break;
                  }
                 }
                }
               
                
                
                

                
                
              }
                    

        
}
  
}











/** Test **/


void testSpeed () {
  dirR = 1;
  dirL = 1;

  analogWrite(MOTORLV, 0);
  analogWrite(MOTORLR, 0);
  analogWrite(MOTORRV, 0);
  analogWrite(MOTORRR, 0);
  delay(1000);
  int i=0;
  volatile int16_t ls,rs;
  
  Serial.println("LV:");
  for (i=30;i<256;i++) {
    analogWrite(MOTORLV, i);
    //analogWrite(MOTORRV, i);
    delay(100);
    motorLSteps = 0;
    //motorRSteps = 0;
    delay(1000);
    ls = motorLSteps;
    //rs = motorRSteps;
    Serial.print(i);
    Serial.print(":\t");
    Serial.println(ls);
    //Serial.print("\t");
    //Serial.println(rs);
  }
  
  analogWrite(MOTORLV, 0);
  analogWrite(MOTORLR, 0);
  analogWrite(MOTORRV, 0);
  analogWrite(MOTORRR, 0);
  delay(1000);
  
  Serial.println("RV:");
  for (i=30;i<256;i++) {
    //analogWrite(MOTORLV, i);
    analogWrite(MOTORRV, i);
    delay(100);
    //motorLSteps = 0;
    motorRSteps = 0;
    delay(1000);
    //ls = motorLSteps;
    rs = motorRSteps;
    Serial.print(i);
    Serial.print(":\t");
    //Serial.print(ls);
    //Serial.print("\t");
    Serial.println(rs);
  }
  
  analogWrite(MOTORLV, 0);
  analogWrite(MOTORLR, 0);
  analogWrite(MOTORRV, 0);
  analogWrite(MOTORRR, 0);
  
  
  delay(1000);
  
  Serial.println("LR:");
  for (i=30;i<256;i++) {
    analogWrite(MOTORLR, i);
    //analogWrite(MOTORRR, i);
    delay(100);
    motorLSteps = 0;
    //motorRSteps = 0;
    delay(1000);
    ls = motorLSteps;
    //rs = motorRSteps;
    Serial.print(i);
    Serial.print(":\t");
    Serial.println(ls);
    //Serial.print("\t");
    //Serial.println(rs);
  }
  
  analogWrite(MOTORLV, 0);
  analogWrite(MOTORLR, 0);
  analogWrite(MOTORRV, 0);
  analogWrite(MOTORRR, 0);
  delay(1000);
  
  Serial.println("RR:");
  for (i=30;i<256;i++) {
    //analogWrite(MOTORLR, i);
    analogWrite(MOTORRR, i);
    delay(100);
    //motorLSteps = 0;
    motorRSteps = 0;
    delay(1000);
    //ls = motorLSteps;
    rs = motorRSteps;
    Serial.print(i);
    Serial.print(":\t");
    //Serial.print(ls);
    //Serial.print("\t");
    Serial.println(rs);
  }
  
  analogWrite(MOTORLV, 0);
  analogWrite(MOTORLR, 0);
  analogWrite(MOTORRV, 0);
  analogWrite(MOTORRR, 0);
  
}









/* OPEN DRAIN OUTPUT */

//Analog Write with Open-Drain support
void analogWriteOD(uint8_t pin, int val)
{
	uint32_t cval, max;

#if defined(__MK20DX256__)
	if (pin == A14) {
		uint8_t res = analog_write_res;
		if (res < 12) {
			val <<= 12 - res;
		} else if (res > 12) {
			val >>= res - 12;
		}
		analogWriteDAC0(val);
		return;
	}
#endif

	max = 1 << analog_write_res;
	val = max - val;
	if (val <= 0) {
		digitalWrite(pin, LOW);
		pinModeOD(pin, OUTPUT_OPEN_DRAIN);	// TODO: implement OUTPUT_LOW
		return;
	} else if (val >= max) {
		digitalWrite(pin, HIGH);
		pinModeOD(pin, OUTPUT_OPEN_DRAIN);	// TODO: implement OUTPUT_HIGH
		return;
	}

	//serial_print("analogWrite\n");
	//serial_print("val = ");
	//serial_phex32(val);
	//serial_print("\n");
	//serial_print("analog_write_res = ");
	//serial_phex(analog_write_res);
	//serial_print("\n");
	if (pin == 3 || pin == 4) {
		cval = ((uint32_t)val * (uint32_t)(FTM1_MOD + 1)) >> analog_write_res;
#if defined(__MK20DX256__)
	} else if (pin == 25 || pin == 32) {
		cval = ((uint32_t)val * (uint32_t)(FTM2_MOD + 1)) >> analog_write_res;
#endif
	} else {
		cval = ((uint32_t)val * (uint32_t)(FTM0_MOD + 1)) >> analog_write_res;
	}
	//serial_print("cval = ");
	//serial_phex32(cval);
	//serial_print("\n");
	switch (pin) {
	  case 3: // PTA12, FTM1_CH0
		FTM1_C0V = cval;
		CORE_PIN3_CONFIG = PORT_PCR_MUX(3) | PORT_PCR_DSE | PORT_PCR_ODE | PORT_PCR_SRE;
		break;
	  case 4: // PTA13, FTM1_CH1
		FTM1_C1V = cval;
		CORE_PIN4_CONFIG = PORT_PCR_MUX(3) | PORT_PCR_DSE | PORT_PCR_ODE | PORT_PCR_SRE;
		break;
	  case 5: // PTD7, FTM0_CH7
		FTM0_C7V = cval;
		CORE_PIN5_CONFIG = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_ODE | PORT_PCR_SRE;
		break;
	  case 6: // PTD4, FTM0_CH4
		FTM0_C4V = cval;
		CORE_PIN6_CONFIG = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_ODE | PORT_PCR_SRE;
		break;
	  case 9: // PTC3, FTM0_CH2
		FTM0_C2V = cval;
		CORE_PIN9_CONFIG = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_ODE | PORT_PCR_SRE;
		break;
	  case 10: // PTC4, FTM0_CH3
		FTM0_C3V = cval;
		CORE_PIN10_CONFIG = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_ODE | PORT_PCR_SRE;
		break;
	  case 20: // PTD5, FTM0_CH5
		FTM0_C5V = cval;
		CORE_PIN20_CONFIG = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_ODE | PORT_PCR_SRE;
		break;
	  case 21: // PTD6, FTM0_CH6
		FTM0_C6V = cval;
		CORE_PIN21_CONFIG = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_ODE | PORT_PCR_SRE;
		break;
	  case 22: // PTC1, FTM0_CH0
		FTM0_C0V = cval;
		CORE_PIN22_CONFIG = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_ODE | PORT_PCR_SRE;
		break;
	  case 23: // PTC2, FTM0_CH1
		FTM0_C1V = cval;
		CORE_PIN23_CONFIG = PORT_PCR_MUX(4) | PORT_PCR_DSE | PORT_PCR_ODE | PORT_PCR_SRE;
		break;
#if defined(__MK20DX256__)
	  case 32: // PTB18, FTM2_CH0
		FTM2_C0V = cval;
		CORE_PIN32_CONFIG = PORT_PCR_MUX(3) | PORT_PCR_DSE | PORT_PCR_ODE | PORT_PCR_SRE;
		break;
	  case 25: // PTB19, FTM1_CH1
		FTM2_C1V = cval;
		CORE_PIN25_CONFIG = PORT_PCR_MUX(3) | PORT_PCR_DSE | PORT_PCR_ODE | PORT_PCR_SRE;
		break;
#endif
	  default:
		digitalWrite(pin, (val > 127) ? HIGH : LOW);
		pinModeOD(pin, OUTPUT_OPEN_DRAIN);
	}
}


//pinMode with Open-Drain support
void pinModeOD(uint8_t pin, uint8_t mode)
{
	volatile uint32_t *config;

	if (pin >= CORE_NUM_DIGITAL) return;
	config = portConfigRegister(pin);

	if (mode == OUTPUT) {
		*portModeRegister(pin) = 1;
		*config = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
	} else if (mode == OUTPUT_OPEN_DRAIN) {
		*portModeRegister(pin) = 1;
		*config = PORT_PCR_SRE | PORT_PCR_DSE | PORT_PCR_MUX(1);
		*config = PORT_PCR_ODE | PORT_PCR_MUX(1);
    } else if (mode == NO_GPIO) {
		*portModeRegister(pin) = 0;
		*config = PORT_PCR_MUX(0);
	} else {
		*portModeRegister(pin) = 0;
		if (mode == INPUT) {
			*config = PORT_PCR_MUX(1);
		} else {
			*config = PORT_PCR_MUX(1) | PORT_PCR_PE | PORT_PCR_PS; // pullup
		}
	}
}
