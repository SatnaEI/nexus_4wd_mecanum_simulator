#include <MotorWheel.h> 
#include <Omni3WD.h> 
#include <Omni4WD.h> 
#include <PID_Beta6.h> 
#include <PinChangeInt.h> 
#include <PinChangeIntConfig.h>
 // Include the header files 
 /*  
 \ /  wheel1 \ / wheel4  
 Left \ / Right      
 / \  wheel2 / \ wheel3  
 Right / \ Left 
 */ 
 irqISR(irq1,isr1);   // Intterrupt function.on the basis of the pulse,work for wheel1 
 MotorWheel wheel1(3,2,4,5,&irq1); //This will create a MotorWheel object called Wheel1   
 //Motor PWM:Pin5, DIR:Pin4, Encoder A:Pin2, B:Pin3 
 irqISR(irq2,isr2); 
 MotorWheel wheel2(11,12,14,15,&irq2);  
 irqISR(irq3,isr3); 
 MotorWheel wheel3(9,8,16,17,&irq3);  
 irqISR(irq4,isr4); 
 MotorWheel wheel4(10,7,18,19,&irq4);  
 Omni4WD Omni(&wheel1,&wheel2,&wheel3,&wheel4); // This will create a Omni4WD object called Omni4WD. //You can then use any of its methods; for instance,   // to control a Omni4WD attached to pins, you could write   
 void setup() { 
  Serial.begin(115200);
  while (!Serial);  
  //TCCR0B=TCCR0B&0xf8|0x01; 
  // warning!! it will change millis()   
  TCCR1B=TCCR1B&0xf8|0x01; // Pin9,Pin10 PWM 31250Hz   
  TCCR2B=TCCR2B&0xf8|0x01; // Pin3,Pin11 PWM 31250Hz  
  Omni.PIDEnable(0.31,0.01,0,10); // Enable PID  
  } 
  void loop() {
//démo 
    //Omni.demoActions(200,500,500,false); //Call the demoActions from the Class Omni4WD  
    //speedMMPS=200   duration=5000  
    //uptime =500     debug=false  



//avancer, reculer, tourner      
//    Omni.setCarAdvance(0);  
//    Omni.setCarSpeedMMPS(200,500);  
//    Omni.delayMS(5000);  
//    Omni.setCarSlow2Stop(500);   
//    Omni.setCarBackoff(0);  
//    Omni.setCarSpeedMMPS(200,500);  
//    Omni.delayMS(5000);   
//    Omni.setCarSlow2Stop(500); 
//    
//    Omni.setCarRotateLeft(0); 
//    Omni.setCarSpeedMMPS(200,500);  
//    Omni.delayMS(5000);  
//    Omni.setCarSlow2Stop(500);
//    Omni.setCarRotateRight(0); 
//    Omni.setCarSpeedMMPS(200,500);  
//    Omni.delayMS(5000);  
//    Omni.setCarSlow2Stop(500);
 
 
 
// controle clavier
 if (Serial.available() > 0) {
    int r = Serial.read();
    switch (r) {
      // the keyboard
      case 'Z':
        Omni.setCarAdvance(0);
        Omni.setCarSpeedMMPS(200,500);
        break;
      case 'S':
        Omni.setCarBackoff(0);
        Omni.setCarSpeedMMPS(200,500);
        break;
      case 'Q':
        Omni.setCarRotateLeft(0);
        Omni.setCarSpeedMMPS(200,500);
        break;
      case 'D':
        Omni.setCarRotateRight(0);
        Omni.setCarSpeedMMPS(200,500);
        break;
    }
 }
}
