#include <Servo.h>
Servo myservo;  
void setup() 
{
  myservo.attach(9);
  Serial.begin(115200);
}

void loop() 
{
  if(Serial.available())
  {
    char option = Serial.read();
    if(option=='W')
    {
      while(!Serial.available());
      myservo.write(Serial.parseInt());
      delay(10);
      
    }
    else if(option=='R')
    {
      Serial.println(myservo.read()); 
      delay(10);     
    }
  }
}
