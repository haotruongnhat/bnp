#define Xpin 9
#define Ypin 10
 
#include <VarSpeedServo.h> 

VarSpeedServo Xservo;
VarSpeedServo Yservo;

void setup() {
  // initialize serial:
  Serial.begin(9600);
  
  // make the pins outputs:
  Xservo.attach(Xpin);
  Yservo.attach(Ypin);
  Xservo.write(102);
  Yservo.write(57);
  delay(2);
}

void loop() {
  
  // if there's any serial available, read it:
  while (Serial.available() > 0) {

    // look for the next valid integer in the incoming serial stream:
    int Xval = Serial.parseInt();
    // do it again:
    int Yval = Serial.parseInt();

    // look for the newline. That's the end of your
    // sentence:
    if (Serial.read() == '\n') {
      // constrain the values to 0 - 255 and invert
      // if you're using a common-cathode LED, just use "constrain(color, 0, 255);"
      //Xval = map(Xval, 0, 255, 120, 180);
      //Yval = map(Yval, 0, 255, 180, 120);

      // fade the red, green, and blue legs of the LED:
      Xservo.write(Xval);
      Yservo.write(Yval);

      // print the three numbers in one string as hexadecimal:
      //Serial.print(Xval, HEX);
      //Serial.println(Yval, HEX);
      //delayMicroseconds(20);
    }
  }
}
