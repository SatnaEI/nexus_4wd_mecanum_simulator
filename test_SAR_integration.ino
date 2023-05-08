#include <MotorWheel.h> 
//#include <Omni3WD.h> 
#include <Omni4WD.h> 
#include <PID_Beta6.h> 
#include <PinChangeInt.h> 
#include <PinChangeIntConfig.h>


#include <ros.h>
#include <geometry_msgs/Point.h>

ros::NodeHandle  nh;

geometry_msgs::Point str_msg;
ros::Publisher chatter("chatter", &str_msg);

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

float x=0;
float y=0;
float theta=PI/2;
int b=0;
char d='X';


unsigned long t0;
unsigned long t1;


//You can then use any of its methods; for instance,   
// to control a Omni4WD attached to pins, you could write   
void setup() {   //TCCR0B=TCCR0B&0xf8|0x01; // warning!! it will change millis()   
  Serial.begin(115200);
  while (!Serial); 
  TCCR1B=TCCR1B&0xf8|0x01; // Pin9,Pin10 PWM 31250Hz   
  TCCR2B=TCCR2B&0xf8|0x01; // Pin3,Pin11 PWM 31250Hz  
  Omni.PIDEnable(0.31,0.01,0,10); // Enable PID 

 
  nh.initNode();
  nh.advertise(chatter);
  
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


void loop() {
   
//  avanzar();
//  retroceder();
//  girar_derecha();
//  girar_izquierda();
   
//Controle clavier
int r = Serial.read();
if (Serial.available() > 0){
  d=r;
}

if (b==1){
  Serial.println(d);
  t1=millis();
  if (d=='Z'){
     x=x+(255.0/422.0)*((t1-t0)/1000.0)*cos(theta);
     y=y+(255.0/422.0)*((t1-t0)/1000.0)*sin(theta);
     theta=theta;

  }
  else if (d=='S'){
    t1=millis();
    x=x-(50.0/422.0)*((t1-t0)/1000.0)*cos(theta);
    y=y-(50.0/422.0)*((t1-t0)/1000.0)*sin(theta);
    theta=theta;

  }
  else if (d=='Q'){
    t1=millis();
    x=x;
    y=y;
    theta=theta+(125.0/1250.0)*((t1-t0)/1000.0);

  }
  else if (d=='D'){
    t1=millis();
    x=x;
    y=y;
    theta=theta-(125.0/1250.0)*((t1-t0)/1000.0);

  }
  else{
    x=x;
    y=y;
    theta=theta;

  }
  Serial.print("X= ");
  Serial.print(x);
  Serial.print(";   Y=");
  Serial.print(y);
  Serial.print(";   theta=");
  Serial.println(theta);
  
}
    

 
    //en continue
    switch (d) {
      // the keyboard
      case 'Z':
        if (b==0){
          t0=millis();
        }
        avanzar(255);
       
        break;
      case 'S':
        if (b==0){
          t0=millis();
        }
        retroceder(50);
        
        break;
      case 'Q':
        if (b==0){
          t0=millis();
        }
        girar_izquierda(125);
        
        break;
      case 'D':
        if (b==0){
          t0=millis();
        }
        girar_derecha(125);

        break;
      case 'X':
        parar();

        break;
    }
 
    b=1;
    t0=t1;

  str_msg.x = x;
  str_msg.y = y;
  str_msg.z = theta;
  chatter.publish( &str_msg );
  nh.spinOnce();



 
}
