/*
  TSL1401test --- Taos TSL1401 image sensor chip 2010-07-24
  datasheet: http://www.ams.com/eng/content/download/250163/975677/file/TSL1401CL.pdf
  
  trace: http://ap.urpi.fei.stuba.sk/sensorwiki/index.php/TSL1401_Line_Sensor
  other inos:
    - http://forums.parallax.com/showthread.php/125594-TSL1401-and-Arduino
    - https://github.com/ap-tech/Getting-started-with-the-TSL1401CL-linescan-camera-with-arduino-and-processing.-/blob/master/TSL1401CL%20linescan%20camera%20code./Linescane_camera_code/Linescane_camera_code.ino
    
  
*/
int cruzamentos = 15; //para um circuito sem cruzamento
int contador_cruzamentos = 0; //conta o número de faixas à deireita
#include <TimerOne.h>
#define sensor_cruzamento 9

// Controlador PID
float Kp = 3.4;//(3.5)

float Ki = 0.5; //1

float Kd = 70; //(65)75

#define led 13
#define botao 8
#define rightMotor 5
#define rightGnd 10

#define leftMotor 6
#define leftGnd 11

int sensor = 64, leituras = 0, calibracao = 0; //melhor usar 115 //variaveis usadas pela função pdi
int setpoint = 50;
int P = 0, I = 0, D = 0, previousError = 0, error = 0; ////variaveis usadas pela função pid
float PIDvalue = 0;
int rightMotorSpeed = 0, leftMotorSpeed = 0, rightBaseSpeed = 50, leftBaseSpeed = 50;
int ultima_leitura = 50;

                     // Sensor interface: 
#define AOpin  A0     // Analog output - yellow
#define SIpin  3     // Start Integration - orange
#define CLKpin 2     // Clock - red
                     // Vcc - brown
                     // GND - black
 
#define NPIXELS 128  // No. of pixels in array
 
int Pixel[NPIXELS]; // Field for measured values <0-255>
int segundos = 0; 
void timerIsr(); 
#define FASTADC 1   
 // defines for setting and clearing register bits
 #define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
 #define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
 
 
 
void setup(void)
{
   Timer1.initialize(1000000); // set a timer of length 100000 microseconds (or 0.1 sec - or 10Hz => the led will blink 5 times, 5 cycles of on-and-off, per second)
   Timer1.attachInterrupt( timerIsr );
  
   pinMode(SIpin, OUTPUT);
   pinMode(CLKpin, OUTPUT);
   //pinMode (AOpin, INPUT);

  pinMode(sensor_cruzamento, INPUT);
  pinMode(led, OUTPUT);
  pinMode (botao, INPUT_PULLUP);
  pinMode(rightMotor, OUTPUT);
  pinMode(leftMotor, OUTPUT);
  pinMode(rightGnd, OUTPUT);
  pinMode(leftGnd, OUTPUT);
  
   digitalWrite(SIpin, LOW);   // IDLE state
   digitalWrite(CLKpin, LOW);  // IDLE state
 
#if FASTADC
  // set prescale to 16
  sbi(ADCSRA,ADPS2);
  cbi(ADCSRA,ADPS1);
  cbi(ADCSRA,ADPS0);
#endif
 
   Serial.begin (115200);


 //digitalWrite(rightMotor, HIGH);
  digitalWrite(rightMotor, LOW);
  digitalWrite(rightGnd, LOW);
  //digitalWrite(leftMotor, HIGH);
  digitalWrite(leftMotor, LOW);
  digitalWrite(leftGnd, LOW);
  delay(3000);
  calibra();

/*  int configura = 0;
  while(configura < 1)
  {
    if(digitalRead(botao) == 0)
    {
      delay(1000);
      if(digitalRead(botao) == 0)
      {
        digitalWrite (led, HIGH);
        delay(300);
        digitalWrite (led, LOW);
        delay(300);
        digitalWrite (led, HIGH);
        configura = 1;
        while(digitalRead(botao) == 0){}
        delay(500);
      }
      else
      {
        digitalWrite (led, HIGH);
        delay(300);
        digitalWrite (led, LOW);
        calibracao = calibracao + 10;
        while(digitalRead(botao) == 0){}
      }
     
      
     }
     Serial.println(calibracao);
   }*/
}
void timerIsr()
{
    // Toggle LED
    segundos++;
} 
 
 void calibra()
 {
  int maximo = 0;
  int minimo = 1023;
  int media[50] = {};
  calibracao = 0;
  
  for(int c = 0; c < 50; c++)
  {
    maximo = 0;
    minimo = 1023;

    delayMicroseconds (1);  // Integration time in microseconds 
    delay(10);              // Integration time in miliseconds  
        
    digitalWrite (CLKpin, LOW);
    digitalWrite (SIpin, HIGH);
    digitalWrite (CLKpin, HIGH);
    digitalWrite (SIpin, LOW);
    
    delayMicroseconds (1);            

    for (int i = 0; i < NPIXELS; i++)
    {
       Pixel[i] = analogRead (AOpin);
       digitalWrite (CLKpin, LOW);
       delayMicroseconds (1);
       digitalWrite (CLKpin, HIGH);
      if(Pixel[i]>maximo)
      {
          maximo = Pixel[i];
       }
      if(Pixel[i]<minimo)
      {
          minimo = Pixel[i];
      }
    }
    media[c] = (maximo + minimo)/2;
    Serial.println(media[c]);
  }
  for(int c = 5; c < 50; c++) //descarta as primeiras leituras erradas
  {
    calibracao = calibracao + media[c];
  }
  calibracao = calibracao/45; //não usa as duas primeiras leituras
  Serial.print("calibracao: ");
  Serial.println(calibracao);
 }

 void calibra_fast()
 {
  int maximo = 0;
  int minimo = 1023;
  calibracao = 0;
  
    maximo = 0;
    minimo = 1023;

    delayMicroseconds (1);  // Integration time in microseconds 
    delay(10);              // Integration time in miliseconds  
        
    digitalWrite (CLKpin, LOW);
    digitalWrite (SIpin, HIGH);
    digitalWrite (CLKpin, HIGH);
    digitalWrite (SIpin, LOW);
    
    delayMicroseconds (1);            

    for (int i = 0; i < NPIXELS; i++)
    {
       Pixel[i] = analogRead (AOpin);
       digitalWrite (CLKpin, LOW);
       delayMicroseconds (1);
       digitalWrite (CLKpin, HIGH);
      if(Pixel[i]>maximo)
      {
          maximo = Pixel[i];
       }
      if(Pixel[i]<minimo)
      {
          minimo = Pixel[i];
      }
    }
    calibracao = (maximo + minimo)/2;
  }
 

 /************************************************************************/

 void calculatePID()
  {
    error = sensor - setpoint;
    P = error;
    I = I + error;
    
    if(I >= 50)
      I = 50;

    if(I <= -50)
      I = -50;
    
    D = error-previousError;

    if(D >= 50)
      D = 50;

    if(D <= -50)
      D = -50;
      
    PIDvalue = (Kp*P) + (Ki*I) + (Kd*D);

    if(PIDvalue >= 50)
      PIDvalue = 50;

    if(PIDvalue <= -50)
      PIDvalue = -50;
      
    previousError = error;

    //Serial.println(PIDvalue);
  }

  /************************************************************************/


  void motor(){

  //if(sensor > 0)
  //{
    rightMotorSpeed = rightBaseSpeed - PIDvalue;
    leftMotorSpeed = leftBaseSpeed + PIDvalue;

//    delta = delta - PIDvalue;
   
    if(rightMotorSpeed >= 100)
      rightMotorSpeed = 100;

    if(rightMotorSpeed <= 0)
      rightMotorSpeed = 0;
    
    if(leftMotorSpeed >= 100)
      leftMotorSpeed = 100;

    if(leftMotorSpeed <= 0)
      leftMotorSpeed = 0;

/*      digitalWrite(rightMotor, LOW);
      analogWrite(rightGnd, delta);

      digitalWrite(leftMotor, LOW);
      analogWrite(leftGnd, delta);
      
      delay(10);

      delta = PIDvalue;
      
      digitalWrite(rightGnd, LOW);
      digitalWrite(leftGnd, LOW);
  */ 
      rightMotorSpeed = map(rightMotorSpeed, 0, 100, 0, 255);
      leftMotorSpeed = map(leftMotorSpeed, 0, 100, 0, 255);

      digitalWrite(leftGnd, LOW);
      digitalWrite(rightGnd, LOW);
      analogWrite(rightMotor, rightMotorSpeed);
      analogWrite(leftMotor, leftMotorSpeed);
  /*}
  else
  {
    digitalWrite(rightMotor, LOW);
    analogWrite(rightGnd, (rightMotorSpeed+leftMotorSpeed)/2);
    digitalWrite(leftMotor, LOW);
    analogWrite(leftGnd, (rightMotorSpeed+leftMotorSpeed)/2);
   
  }*/

  }

  
void loop (void)
{
  if(segundos > 34){
      digitalWrite(leftGnd, LOW);
      digitalWrite(rightGnd, LOW);
      analogWrite(rightMotor, LOW);
      analogWrite(leftMotor, LOW);
      delay(10000);
  }
  /*if(digitalRead(sensor_cruzamento) == LOW)
  {
    contador_cruzamentos++;
    delay(60);
  }
  
  if(contador_cruzamentos >= cruzamentos)
  {
      digitalWrite(leftGnd, LOW);
      digitalWrite(rightGnd, LOW);
      analogWrite(rightMotor, LOW);
      analogWrite(leftMotor, LOW);
      delay(10000);
    }*/
   ultima_leitura = sensor;
   sensor = 0;


   int expTime;
 
 
   delayMicroseconds (1);  // Integration time in microseconds 
   delay(10);              // Integration time in miliseconds  
 
 
   digitalWrite (CLKpin, LOW);
   digitalWrite (SIpin, HIGH);
   digitalWrite (CLKpin, HIGH);
   digitalWrite (SIpin, LOW);
 
   delayMicroseconds (1);            
 
/* and now read the real image */
  
   for (int i = 0; i < NPIXELS; i++) {
     Pixel[i] = analogRead (AOpin);
     digitalWrite (CLKpin, LOW);
     delayMicroseconds (1);
     digitalWrite (CLKpin, HIGH);
    if(Pixel[i]>calibracao)
    {
        leituras++;
        sensor = sensor + i;
     }
   }

   sensor = sensor/leituras; //define onde esta o centro do robô
   //Serial.println(sensor);
   
   leituras = 0;
   sensor = map(sensor, 0, 127, 0, 100);//remapeia os valores númericos para que se possa implementalos nos proximos calculos, definindo 0 como centro
   //Serial.println(sensor);
     
   if(sensor<=0)
   sensor=ultima_leitura;
   
      
    //if(sensor > 0)
    calculatePID();
    
    
   
   //Serial.println ((byte)0);            // sync byte = 0
   for (int i = 0; i < NPIXELS; i++) {
       
       //Serial.println (Pixel[i]);
   }
   
   //Serial.println(sensor);
   //Serial.println(PIDvalue);
   motor();

   //if((sensor >40) &&(sensor<60))//49 e 51 respectivamente
   if(PIDvalue > -15 && PIDvalue < 15)
   {
    calibra_fast();
   }
}
