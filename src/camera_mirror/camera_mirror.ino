#include <Servo.h>

Servo mirrorR_X; // left/right rotation
Servo mirrorR_Y; // up/down rotation

Servo mirrorL_X;
Servo mirrorL_Y; 

// set 0 positions for servos
int mirrorR_XHome = 23;
int mirrorR_YHome = 55;
int mirrorL_XHome = 80;
int mirrorL_YHome = 95;

int mirrorR_XAngle = mirrorR_XHome; 
int mirrorR_YAngle = mirrorR_YHome;
int mirrorL_XAngle = mirrorL_XHome; 
int mirrorL_YAngle = mirrorL_YHome;

void setup() {
  mirrorR_X.attach(9); 
  mirrorR_Y.attach(10);
  mirrorL_X.attach(12); 
  mirrorL_Y.attach(13);
  
  // start at home position
  mirrorR_X.write(mirrorR_XAngle);
  mirrorR_Y.write(mirrorR_YAngle);
  mirrorL_X.write(mirrorL_XAngle);
  mirrorL_Y.write(mirrorL_YAngle);  

  Serial.begin(9600);  
}

void loop() {
  if (Serial.available() > 0) {
    // input format: <device> <angle>\n
    // angle is from -90 to 90 degrees
    String device = Serial.readStringUntil(' ');
    int angle = Serial.readStringUntil('\n').toInt();
    Serial.println("Received: " + angle);
    // angle = constrain(angle, -90, 90);

    if (device == "MR_X") {
      mirrorR_XAngle = mirrorR_XHome + angle;
      mirrorR_X.write(mirrorR_XAngle);
    } else if (device == "MR_Y") {
      mirrorR_YAngle = mirrorR_YHome - angle; // +/- is switched for some reason
      mirrorR_Y.write(mirrorR_YAngle);
    } else if (device == "ML_X") {
      mirrorL_XAngle = mirrorL_XHome + angle;
      mirrorL_X.write(mirrorL_XAngle);
    } else if (device == "ML_Y") {
      mirrorL_YAngle = mirrorL_YHome + angle;
      mirrorL_Y.write(mirrorL_YAngle);
    }  
  }
}
