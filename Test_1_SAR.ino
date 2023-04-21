#include <MotorWheel.h> 
#include <Omni3WD.h> 
#include <Omni4WD.h> 
#include <PID_Beta6.h> 
#include <PinChangeInt.h> 
#include <PinChangeIntConfig.h>

irqISR(irq1,isr1);   // Intterrupt function.on the basis of the pulse,work for wheel1 
MotorWheel wheel1(3,2,4,5,&irq1); //This will create a MotorWheel object called Wheel1   
//Motor PWM:Pin5, DIR:Pin4, Encoder A:Pin12, B:Pin13 
irqISR(irq2,isr2); 
MotorWheel wheel2(11,12,14,15,&irq2);  
irqISR(irq3,isr3); 
MotorWheel wheel3(9,8,16,17,&irq3);  
irqISR(irq4,isr4); 
MotorWheel wheel4(10,7,18,19,&irq4);  
Omni4WD Omni(&wheel1,&wheel2,&wheel3,&wheel4); // This will create a Omni4WD object called Omni4WD. 
//You can then use any of its methods; for instance,   
// to control a Omni4WD attached to pins, you could write   
void setup() {   //TCCR0B=TCCR0B&0xf8|0x01; // warning!! it will change millis()   
  Serial.begin(115200);
  while (!Serial); 
  TCCR1B=TCCR1B&0xf8|0x01; // Pin9,Pin10 PWM 31250Hz   
  TCCR2B=TCCR2B&0xf8|0x01; // Pin3,Pin11 PWM 31250Hz  
  Omni.PIDEnable(0.31,0.01,0,10); // Enable PID 
  
  } 

int tourne_droite() {
  Omni.setCarRotateRight(0);  
  Omni.setCarSpeedMMPS(200,500);  
  Omni.delayMS(2000);   
  Omni.setCarSlow2Stop(500);
}

void tourne_gauche() {
  Omni.setCarRotateLeft(0);  
  Omni.setCarSpeedMMPS(200,500);  
  Omni.delayMS(2000);  
  Omni.setCarSlow2Stop(500);
}

void avance() {
  Omni.setCarAdvance(0);  
  Omni.setCarSpeedMMPS(200,500);  
  Omni.delayMS(2000);  
  Omni.setCarSlow2Stop(500);
}

void recule() {
  Omni.setCarBackoff(0);  
  Omni.setCarSpeedMMPS(200,500);  
  Omni.delayMS(2000);  
  Omni.setCarSlow2Stop(500);
}

void arret() {
  digitalWrite(3,LOW);   
  digitalWrite(9,LOW);  
  digitalWrite(10,LOW);  
  digitalWrite(11,LOW);  
}

void loop() {
   
   avance();
   recule();
   tourne_droite();
   tourne_gauche();
   
/* Controle clavier
   if (Serial.available() > 0) {
    int r = Serial.read();
    

    //en continue
    switch (r) {
      // the keyboard
      case 'Z':
        avance();
        break;
      case 'S':
        recule();
        break;
      case 'Q':
        tourne_gauche();
        break;
      case 'D':
        tourne_droite();
        break;
      case 'X':
        arret();
        break;
    }
    
    //s'arrete au bour de 2s 
    switch (r) {
      // the keyboard
      case 'Z':
        avance();
        arret();
        break;
      case 'S':
        recule();
        arret();
        break;
      case 'Q':
        tourne_gauche();
        arret();
        break;
      case 'D':
        tourne_droite();
        arret();
        break;   
   }
  }

  */
}



