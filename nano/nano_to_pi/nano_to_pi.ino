#include <Wire.h>
#include <Servo.h> 
 
Servo myservo; 

#define SLAVE_ADDRESS 0x04

int number = 0;
int state  = 0;
int led = 13;

void setup() {
  myservo.attach(9);
  
  pinMode(led, OUTPUT);
  Serial.begin(9600); // start serial for output

  Wire.begin(SLAVE_ADDRESS);

  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);

  Serial.println("Ready!");
}


void loop() {
  delay(100);
  float f = number*180.0/256.0;
  myservo.write((int) f);
  
  //myservo.write(0);              // tell servo to go to position in variable 'pos' 
  //delay(500);                       // waits 15ms for the servo to reach the position 
  //myservo.write(180);
  //delay(500);
}

// callback for received data
void receiveData(int byteCount) {

  while(Wire.available()) {
    number = Wire.read();
    Serial.print("data received:");
    Serial.println(number);

    analogWrite(led, number);  
    /*if (number < 128) {
      if (state == 0) {
        digitalWrite(13, HIGH); // set the LED on
        state = 1;
      }
      else {
        digitalWrite(13, LOW); // set the LED off
        state = 0;
      }
    }*/
  }
}

// callback for sending data
void sendData(){
  Wire.write(number);
}

