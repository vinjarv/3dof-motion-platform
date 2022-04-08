#include <Servo.h>    //modified from https://forum.arduino.cc/t/serial-input-basics-updated/382007/3

const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];   // temporary array for use when parsing

int servoPosArray[3];    // variables to hold the parsed data
boolean newData = false;

Servo servo1; //create the servo objects
Servo servo2;
Servo servo3;

void setup() {
    Serial.begin(250000);
    Serial.println("This program expects 3 integers");
    Serial.println("Enter the angles formated like this (35, -12, 24)  ");
    Serial.println();
    Serial.setTimeout(100);

    servo1.attach(9);  //attach the servo objects to arduino pins
    servo2.attach(10);
    servo3.attach(11);
    servo1.write(90);   //set servos to centerposition
    servo2.write(90);
    servo3.write(90);
}

void loop() {
    recvWithStartEndMarkers();
    if (newData == true) {
        strcpy(tempChars, receivedChars);
            //this temporary copy is necessary to protect the original data
            //because strtok() used in parseData() replaces the commas with \0
        parseData();
        useParsedData();
        newData = false;
    }
}

void recvWithStartEndMarkers() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '(';
    char endMarker = ')';
    char rc;

    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                receivedChars[ndx] = rc;
                ndx++;
                if (ndx >= numChars) {
                    ndx = numChars - 1;
                }
            }
            else {
                receivedChars[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
                newData = true;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

void parseData() {      //split serial-data into parts

    char * strtokIndx; //this is used by strtok() as an index

    strtokIndx = strtok(tempChars,","); //get the first value from the Serial-string 
    servoPosArray[0] = atoi(strtokIndx); //copy it to the positionarray
 
    strtokIndx = strtok(NULL, ","); //this continues where the previous call left off
    servoPosArray[1] = atoi(strtokIndx); //adds 2nd value to positionarray

    strtokIndx = strtok(NULL, ",");
    servoPosArray[2] = atoi(strtokIndx); //adds 3rd value to positionarray
}

void useParsedData() {
    servo1.write(servoPosArray[0] + 90); //add 90 degrees to rotate the servo around its midpoint
    servo2.write(servoPosArray[1] + 90);
    servo3.write(servoPosArray[2] + 90);
}
