
#include <ros.h>
#include <math.h>
#include <std_msgs/Float32.h>

ros::NodeHandle nh;

std_msgs::Float32 p_msg;

ros::Publisher chatter("pressure", &p_msg);

int sensorValue;

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(57600);

  // int sensorValue;
  
  nh.initNode();
  nh.advertise(chatter);
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input on analog pin 0:
  sensorValue = analogRead(A0);

  Serial.println(sensorValue);
  // print out the value you read:
  
  p_msg.data = log(sensorValue + 1.0);

  chatter.publish( &p_msg );
  
  nh.spinOnce();

  delay(1);        // delay in between reads for stability
}
