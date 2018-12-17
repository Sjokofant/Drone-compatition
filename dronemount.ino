#include <Servo.h>
int pos;
Servo servo;

void setup() {
  servo.attach(14);
  Serial.begin(9600);
  pos = 30;                             // Sets orientation of the camera  
  servo.write(pos); 
}

void loop() {
    while(Serial.available()){          // checks if serial is available 
        int data = Serial.read();       // Reads from port

        switch (data){
        case 'U':
            pos++;
            servo.write(pos);           // tell servo to go to position in variable 'pos'
            delay(10); 
        break;
    
        case 'D':
            pos--;
            servo.write(pos);               
            
            delay(10); 
        break;
        }
    }
} 
