#include <MotorWheel.h> 
#include <Omni4WD.h> 
#include <PID_Beta6.h> 
#include <PinChangeInt.h> 
#include <PinChangeIntConfig.h>

#include <Wire.h>
# define I2C_SLAVE_ADDRESS 8
#define PAYLOAD_SIZE 50

irqISR(irq1,isr1);   // Intterrupt function.on the basis of the pulse,work for wheel1 
MotorWheel wheel1(3,2,4,5,&irq1); //This will create a MotorWheel object called Wheel1   
//Motor PWM:Pin5, DIR:Pin4, Encoder A:Pin12, B:Pin13 
irqISR(irq2,isr2); 
MotorWheel wheel2(11,12,14,15,&irq2);  
irqISR(irq3,isr3); 
MotorWheel wheel3(9,8,16,17,&irq3);      
irqISR(irq4,isr4); 
MotorWheel wheel4(10,7,6,13,&irq4);      //18, 19 est devenu 13, 6
Omni4WD Omni(&wheel1,&wheel2,&wheel3,&wheel4); // This will create a Omni4WD object called Omni4WD. 



float V;
float W;


char d='X';



void setup() {   //TCCR0B=TCCR0B&0xf8|0x01; // warning!! it will change millis()   
  Serial.begin(115200);
  while (!Serial); 
  TCCR1B=TCCR1B&0xf8|0x01; // Pin9,Pin10 PWM 31250Hz   
  TCCR2B=TCCR2B&0xf8|0x01; // Pin3,Pin11 PWM 31250Hz  
  Omni.PIDEnable(0.31,0.01,0,10); // Enable PID 

  Wire.begin(I2C_SLAVE_ADDRESS);
  delay(1000);               
  Wire.onRequest(requestEvents);
  Wire.onReceive(receiveEvents);

  } 

void girar_derecha(int v) {
  Omni.setCarRotateRight(0);  
  Omni.setCarSpeedMMPS(200,500);  
  Omni.delayMS(50);
  analogWrite(3,v);   
  analogWrite(9,v);  
  analogWrite(10,v);  
  analogWrite(11,v); 
}

void girar_izquierda(int v) {
  Omni.setCarRotateLeft(0);  
  Omni.setCarSpeedMMPS(200,500);  
  Omni.delayMS(50);
  analogWrite(3,v);   
  analogWrite(9,v);  
  analogWrite(10,v);  
  analogWrite(11,v);     
}


void avanzar(int v) {
  Omni.setCarAdvance(0);  
  Omni.setCarSpeedMMPS(200,500);  
  Omni.delayMS(50);
  analogWrite(3,v);   
  analogWrite(9,v);  
  analogWrite(10,v);  
  analogWrite(11,v); 
  
}

void retroceder(int v) {
  Omni.setCarBackoff(0);  
  Omni.setCarSpeedMMPS(200,500);  
  Omni.delayMS(50);  
  analogWrite(3,v);   
  analogWrite(9,v);  
  analogWrite(10,v);  
  analogWrite(11,v); 
}

void parar() {
  digitalWrite(3,LOW);   
  digitalWrite(9,LOW);  
  digitalWrite(10,LOW);  
  digitalWrite(11,LOW);  
}

void dir(float V, float W){
  if((W>98.00)||(V>98.00)){
    d='X';
  }
  else if ((W==0)&&(V==0)){
    d=d;
  }  
  else if (W!=0){
    if (W>0){
      d='Q';
    }
    else{
      d='D';
    }
  }
  else if(V!=0){
    if(V>0){
      d='Z';
    }
    else{
      d='S';
    }
  }
  else{
    d=d;
  }
}

void loop() {

   
//  avanzar();
//  retroceder();
//  girar_derecha();
//  girar_izquierda();
   
/*
dir(V,W);
Serial.print("W= ");
Serial.println(W);
Serial.print("V ");
Serial.println(V);
*/

    

    //en continue
    switch (d) {
      // the keyboard
      case 'Z':
        avanzar(255);
        break;
        
      case 'S':
        retroceder(255);
        break;
        
      case 'Q':
        girar_izquierda(125);      
        break;
        
      case 'D':
        girar_derecha(125);
        break;
        
      case 'X':
        parar();
        break;
    }

}

void requestEvents(){
  /*
  Serial.print("X= ");
  Serial.println(x);
  Serial.print("Y= ");
  Serial.println(y);
  Serial.print("T= ");
  Serial.println(theta);
  */
  String vs=String(Omni.getCarSpeedMMPS());
  String ws=String((int)(Omni.getCarSpeedRad()*100));

  String s=vs+"/"+ws;

  char l[7];
  s.toCharArray(l,7);
  Serial.println(s);
  Wire.write(l);
}

void receiveEvents(int howMany)
{
  if (howMany>0){
  char buf[15] = "000000000000";
  int i = 0;
  while(0 < Wire.available()) // loop through all but the last
  {
    char c = Wire.read(); // receive byte as a character

    buf[i] = c;
    i++;
  }
  //int x = Wire.read();    // receive byte as an integer

  //Serial.print("array:");
  //Serial.println(buf[11]);
  char nb1[7] = "00000";
  int sgn1 = 1;     //1 is positive, 0 is negative
  char nb2[7] = "00000";
  int sgn2 = 1;
  int jStop = 0;
  for(int j=1; j<12; j++){

    if (jStop == 0){        //if we are still reading first nb
      if (buf[j] == 45){
      sgn1 = 0;
      //Serial.println(sgn1);
      }
      else if (buf[j] == 47){
        jStop = j;
        //Serial.println(jStop);
      }
      else{
        nb1[j] = buf[j];
      }
    }
    else{                   //in this case we start reading second number
      if (buf[j] == 45){
      sgn2 = 0;

      }
      else{

        nb2[j-jStop] = buf[j];
      }
    }
  }


  if (sgn1 == 1){
    V = String(nb1).toFloat();
  }
  else{
    V = -1.0 * String(nb1).toFloat();
  }
  if (sgn2 == 1){
    W = String(nb2).toFloat();
  }
  else{
    W = -1.0 * String(nb2).toFloat();
  }
  dir(V,W);
  /*
  Serial.print("V= ");
  Serial.println(V);
  Serial.print("W= ");
  Serial.println(W); 
  */
  /*
  Serial.print("X= ");
  Serial.println(x);
  Serial.print("Y= ");
  Serial.println(y);
  Serial.print("T= ");
  Serial.println(theta);
*/
}
}
