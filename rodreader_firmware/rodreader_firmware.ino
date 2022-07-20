/*
 * released under GNU GPL v3 
 *
 *  RODREADER 03 (v1.3)
 *  written by Alex Timiney
 *  April 2014
 * 
 * last updated: 20/07/2022
 * 
 * This is the state of the firmware as of April 2014
 * 
 * Target Hardware: Arduino Uno, unipolar stepper motor, HMC5883L magnetometer
 * The intention was to use two steppers so that the distance from the rod surface to the sensor could be adjusted
 * but in practice, the distance was set manually and noted in the GUI.
 * 
 * The original rod reader had additional hall sensors at the tips of the item under test, however the source code for that has been lost.
 * It also massively over complicated the design of the reader. Future iterations will not include the tip sensors and tilt function.
 * Instead a dedicated hardware design will be developed for this purpose.
 * 
 *  update 20/07/2022
 *  added sync command 'U' 0x55
 *   - responds with the device id string
 *   - device id moved to its own function
 *   
 * Todo:
 * 
 * replace arduino stepper library with stepstick library
 * include linear transfer ratio (steps per mm) and specify moves in real world coordinates
 * add second axis
 * rationalise code base
 * write to sd card option
 * wireless option / smartphone connectivity
 * 
 * increment major version number when backward compatibility is lost
 *   
 *   
 * Original header comment:
 * 
 *  Accepts commands via serial and operates a stepper motor, reads data from a HMC5883L magnetometer,
 *  and transmits the data back over serial as it is read.
 *  Operating parameters can be read, set, loaded from and saved to eeprom, cleared and set to default.
 *  
 *  Parameters are:
 *  
 *  scan start  - position to start sanning from in motor steps        TODO
 *  scan points - number of positions at which to take measurements
 *  scan steps  - number of motor steps between each point
 *  scan speed  - rpm of motor when moving between points
 *  scan delay  - an optional delay between reading and moving
 *  scan rapid  - rpm of motor when returning to zero
 *  samples     - number of samples to take at each point
 *  
 *  additional parameters will be added for X and TILT axis as the hardware is built up
 *  
 *  Analog input 4 I2C SDA
 *  Analog input 5 I2C SCL
 * 
 *  
 */

#include <Stepper.h>
#include <avr/pgmspace.h> //for storing data in program memory... possibly defunct by using F()
#include <EEPROM.h>

#include <Wire.h> //I2C Arduino Library

#define magAddress 0x1E //0011110b, I2C 7bit magAddress of HMC5883

// change this to the number of steps on your motor
#define STEPS 200

// create an instance of the stepper class, specifying
// the number of steps of the motor and the pins it's
// attached to
Stepper stepper(STEPS, 8, 9, 10, 11);

// analog pins for hall sensors:
const int tiltPin = A0;  // Analog input pin that the potentiometer is attached to

//digital pin for 3axis sensor data ready
const int drdyPin = 2;

//number of steps for full vertical traversal
const int scanTravelMax = 2200;
//maximum movement for manual move;
const int moveMax = 5000;
//maximum accepted values for parameters
const int scanPointsMax = 2100; //maximum number of points to be scanned
const int scanStepsMax = 2100; //maximum distance in steps between scan points
const int scanSpeedMax = 100; //maximum rpm whilst scanning
const int scanDelayMax = 1000; //maxmum delay in ms between scans
const int scanRapidMax = 100; //maximum rpm when not scannnig

const int samplesMax = 255;  //defunct (maximum analog averaging)



// parameter variables and their defaults
int scanStart;
int scanPoints;
int scanSteps;
int scanSpeed;
int scanDelay;
int scanRapid;

byte samples;

//current position of vertical carriage
int scanPos = 0;

//I2C sensor configuration bytes, set using functions for gain, rate and averaging
byte sensorConfigA;
byte sensorConfigB;


void printDeviceID() {
  Serial.println(">RODREADER 03, APRIL 2014");
}

//arduino setup procedure
void setup() {
  // initialize serial communications at 115200 bps: (why anyone uses less than this is beyond me)
  Serial.begin(115200); 
  // load parameters if any are stored, otherwise load defaults
  Wire.begin();

  //initialise interupt pin
  pinMode(drdyPin, INPUT);
  
  //print information about this firmware to the serial port
  printDeviceID();

  //load the default, or saved parameters
  loadParams();
  //inital speed of the stepper
  stepper.setSpeed(scanRapid);
}

void loop() {
  //serial input buffer
  char serialInput[20] = "";
  //wait until a new line character is received, or more than 20 char received
  if(Serial.readBytesUntil('\n', serialInput, 20) > 0){
    //echo the received string back to the serial port, adding '>' to the begining as a comment delimiter
    Serial.print(">");
    Serial.println(serialInput);

    String inString = serialInput; //explicitly convert to a String object
    String param;                  //declare a string for storing command parameteres
    //compare the input string against the various commands available
    if(inString.equals("RESET")){    //move all motors to the home positions
      reset();
    }
    else if(inString.equals("SCAN")){  //perform a vertical scan
      scan();
    }
    else if(inString.equals("TILT")){  //perform a tilt scan
      tilt();
    }
    else if(inString.startsWith("STEPS")){  //set the number of steps between scan points
      param = inString.substring(5);         //take the remainder of the string as the parameter

      int value = getParam(param, 1, scanStepsMax); //get the value of the parameter
      if (value != -1){                            // a value greater than 0 is valid
        scanSteps = value;                        // store the value to the parameter
        //if(scanSteps * scanPoints > scanTravelMax - scanStart){  //check to see if this change will exceed the maximum vertical travel
        //  Serial.println(">Warning: Scan travel exceeds maximum. Scan will be truncated"); //warn the user via serial
        //}
      }
    }
    else if(inString.startsWith("POINTS")){ //set the number of points to scan (a value of 1 will result in no movement)
      param = inString.substring(6);          

      int value = getParam(param, 1, scanPointsMax);
      if (value != -1){
        scanPoints = value;
        //if(scanSteps * scanPoints > scanTravelMax - scanStart){
        //  Serial.println(">Warning: Scan travel exceeds maximum. Scan will be truncated");
        //}
      }
    }
    else if(inString.startsWith("SAMPLES")){ //set the number of analog samples to take per point
      param = inString.substring(7);

      int value = getParam(param, 1, samplesMax);
      if (value != -1){
        samples = value;
      }
    }
    else if(inString.startsWith("SPEED")){  //set the speed at which to move between points
      param = inString.substring(5);

      int value = getParam(param, 1, scanSpeedMax);
      if (value != -1){
        scanSpeed = value;
      }
    }
    else if(inString.startsWith("RAPID")){  //set the speed at which to return to zero
      param = inString.substring(5);

      int value = getParam(param, 1, scanRapidMax);
      if (value != -1){
        scanRapid = value;
      }
    }
    else if(inString.startsWith("DELAY")){  //set the optional delay between points
      param = inString.substring(5);

      int value = getParam(param, 0, scanDelayMax);
      if (value != -1){                      // here, 0 is a valid result - ie no delay
        scanDelay = value;
      }
    }
    else if(inString.equals("GETPARAMS")){  //print all the parameters to serial
      printParams();
    }
    else if (inString.equals("SAVE")){  //store the parameters to EEPROM
      saveParams();
    }
    else if (inString.equals("LOAD")){  //load parameters from eeprom, and them to serial
      loadParams();
      printParams();
    }
    else if (inString.equals("DEFAULT")){ //reset the parameters to their defaults
      defaultParams();      
    }
    else if (inString.equals("CLEAR")){  //clear the flag stored in eeprom that indicates  
      EEPROM.write(0,255);                       // the presence of stored parameters
      Serial.println("CLEARED");
    } 
    else if(inString.startsWith("MOVE")){  //set the optional delay between points
      param = inString.substring(4);
      int go;
      if(param[0] == '-'){
        go = -1;
        param = param.substring(1);
      } 
      else {
        go = 1;
      }
      int value = getParam(param,  1, moveMax);
      if(value != -1){
        go*=value;
        stepper.setSpeed(scanRapid);
        sMove(go);        
      }
    } 
    else if(inString.startsWith("MOVETO")){  //set the optional delay between points
      param = inString.substring(6);
      int pos;
      if(param[0] == '-'){
        pos = -1;
        param = param.substring(1);
      } 
      else {
        pos = 1;
      }
      int value = getParam(param,  0, moveMax);
      if(value != -1){
        pos*=value;
        stepper.setSpeed(scanRapid);
        sMoveTo(pos);        
      }
    }
    else if(inString.startsWith("START")){ //set the number of analog samples to take per point
      if(sizeof(inString)==5){
        scanStart = scanPos;
        Serial.println(">Seting scanStart to current position");
      }
      else{
        param = inString.substring(5);

        int value = getParam(param, 0, scanTravelMax);
        if (value != -1){
          scanStart = value;
          //if(scanSteps * scanPoints > scanTravelMax - scanStart){
          //  Serial.println(">Warning: Scan travel exceeds maximum. Scan will be truncated");
          //}
        }
      }
      /**/
    } else if (inString.equals("ZERO")){
      scanPos = 0;
      Serial.println("OK\n>Current position set as ZERO");

    } else if (inString.equals("GETPOS")){
        Serial.print("POSITION ");
        Serial.println(scanPos);
    } else if (inString.startsWith("SENSORGAIN")){
      param = inString.substring(10);

      byte value = getParam(param, 0, 7);
      if (value != -1){                      // here, 0 is a valid result - ie no delay
        setSensorGain(value);
      }

    } else if (inString.startsWith("SENSORRATE")){
    param = inString.substring(10);

      byte value = getParam(param, 0, 6);
      if (value != -1){                      // here, 0 is a valid result - ie no delay
        setSensorRate(value);
      }

    } else if (inString.startsWith("SENSORAVG")){
      param = inString.substring(9);

      byte value = getParam(param, 0, 3);
      if (value != -1){                      // here, 0 is a valid result - ie no delay
        setSensorAvg(value);
      }

    } else if (inString.startsWith("U")){
      //print information about this firmware to the serial port
      printDeviceID();
    }

  }


}

/*
} else 
 */
void printParams(){                // print all the parameters to serial
  Serial.println("PARAMETERS");
  Serial.print("POINTS ");
  Serial.println(scanPoints);
  Serial.print("STEPS ");
  Serial.println(scanSteps);
  Serial.print("START ");
  Serial.println(scanStart);
  Serial.print("SAMPLES ");
  Serial.println(samples);
  Serial.print("DELAY ");
  Serial.println(scanDelay);
  Serial.print("SPEED ");
  Serial.println(scanSpeed);
  Serial.print("RAPID ");
  Serial.println(scanRapid);

  Serial.print("SENSOR CONFIG A ");
  Serial.println(sensorConfigA, HEX);
  Serial.print("SENSOR CONFIG B ");
  Serial.println(sensorConfigB, HEX);
  
  Serial.println("END");      //finish with "END"

}
void loadParams(){            //load params from EEPROM
  int n = 1;                  //address counter
  if(EEPROM.read(0) != 55){          //if the first byte in EEPROM is not 55, load defaults instead
    defaultParams();
  }
  else{
    samples = EEPROM.read(n++);      //read from the address counter, then increment
    scanPoints = word(EEPROM.read(n++),EEPROM.read(n++));  //EEPROM stores BYTES not WORDS, 
    scanSteps = word(EEPROM.read(n++),EEPROM.read(n++));    //so two bytes must be EEPROM.read out and combined
    scanSpeed = word(EEPROM.read(n++),EEPROM.read(n++));
    scanDelay = word(EEPROM.read(n++),EEPROM.read(n++));
    scanRapid = word(EEPROM.read(n++),EEPROM.read(n++));
    scanStart = word(EEPROM.read(n++),EEPROM.read(n++));   
    sensorConfigA = EEPROM.read(n++);
    sensorConfigB = EEPROM.read(n++);
    
    Serial.println("LOADED");
  }

}

void saveParams(){          //save params to EEPROM... make sure this is the same order as load!
  int n = 1;                //address counter
  EEPROM.write(n++,samples);        //write to the address counter, then increment
  EEPROM.write(n++,highByte(scanPoints));  //again, EEPROM stores BYTES not WORDS,
  EEPROM.write(n++,lowByte(scanPoints));   //so INT types must be split in two
  EEPROM.write(n++,highByte(scanSteps));   //and saved as high, then low bytes
  EEPROM.write(n++,lowByte(scanSteps));
  EEPROM.write(n++,highByte(scanSpeed));
  EEPROM.write(n++,lowByte(scanSpeed));
  EEPROM.write(n++,highByte(scanDelay));
  EEPROM.write(n++,lowByte(scanDelay));  
  EEPROM.write(n++,highByte(scanRapid));
  EEPROM.write(n++,lowByte(scanRapid));
  EEPROM.write(n++,highByte(scanStart));
  EEPROM.write(n++,lowByte(scanStart));
  EEPROM.write(n++,sensorConfigA);
  EEPROM.write(n++,sensorConfigB);

  EEPROM.write(0,55);                      //when finished, write 55 into the first memory address
  //to indicate that there are parameters stored in EEPROM
  Serial.println("SAVED");                                
}

void defaultParams(){        //reset params to defaults without saving to EEPROM
  scanPoints = 160;
  scanSteps = 10;
  scanSpeed = 25;
  scanDelay = 0;
  scanRapid = 40;
  samples = 1;
  scanStart = 0;
  sensorConfigA = 0x18;  //no oversampling, max data rate, no bias
sensorConfigB = 0x20;  //2nd most sensitive gain setting  
  
  Serial.println("DEFAULT PARAMS"); 
}

/* set sampling rate on 3axis magnetometer: 
avg,  rate (Hz)
0     0.75
1     1.5
2     3
3     7.5
4     15
5     30
6     75
*/
void setSensorRate(byte rate){
  if(rate >= 0 && rate < 7){
    sensorConfigA &= 0xE3;    
    sensorConfigA |= rate << 2;
  }
}

/* set sensor gain on 3axis magnetometer: 
avg,  field range (+/- Ga)
0     0.88
1     1.3
2     1.9
3     2.5
4     4.0
5     4.7
6     5.6
7     8.1
*/
void setSensorGain(byte gain){
  if(gain >= 0 && gain < 8){
    sensorConfigB = gain << 5;
  }
}

/* set averaging samples on 3axis magnetometer: 
avg,  samples
0     1
1     2
2     4
3     8
*/
void setSensorAvg(byte avg){
if(avg >= 0 && avg <= 3){
    sensorConfigA &= 0x9F;    
    sensorConfigA |= avg << 5;
  }
}


int getParam(String param, unsigned int minValue, unsigned int maxValue){  //convert a string to an int, upto maxValue
  param.trim();                        //remove whitespace
  if(isDigit(param[0])){               //check we actually have a digit
    long value = param.toInt();        //do the conversion
    if (value >= minValue && value <= maxValue){  //check the value is in range

        Serial.println("OK");    //send an acknolagement via serial 
      return(int(value));      //return the value
    } 
    else {
//      value not in range
      Serial.println("Out of range");

    }
  }
  else{                        //the input parameter didn't start with a digit,
    Serial.println("Not a real number\n>(I imagine)");
  }
    return(-1);                //reject


}

void reset(){    //move each motor back toward zero until it hits a hmoe switch, 
  //then move forward till it's off the switch
  //this way, we need only one input for all switches
  //however, if the switches are active at the start of this function
  //theres no way of knowing which axis is hitting the switch
  //this may require the user to remove power and monually move the motors off their switches
  //investigation required as the hardware is developed
  //other calibration processes may be done here, eg, checking tilt level, checking for correct magnetic alignment
  //base scan reading chould be organised by the pc software by performing a scan with no rod fited

//no switches yet, so just go down and stall the motor against the end stop, then go up 1 mm
      stepper.setSpeed(scanSpeed);
      sMove(-moveMax);
      sMove(8);
      scanPos = 0;


}

void tilt(){      //tilt the rod fixture until the base hall sensor reads zero and return the angle
}

void scan(){      //move a hall sensor up the rod taking hall sensor readings and tx'ing to serial

  Wire.beginTransmission(magAddress); //open communication with HMC5883
  Wire.write(0x00); //select comfigA register
  Wire.write(sensorConfigA); //continuous measurement mode
  Wire.endTransmission();
  
  Wire.beginTransmission(magAddress); //open communication with HMC5883
  Wire.write(0x01); //select configB register
  Wire.write(sensorConfigB); //continuous measurement mode
  Wire.endTransmission();

  Wire.beginTransmission(magAddress); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();

  if(scanPos != scanStart){  //check that the carriage is in the start position
    stepper.setSpeed(scanRapid);  //if not, set the stepper to rapid speed
    sMoveTo(scanStart);
  }

  stepper.setSpeed(scanSpeed);    //set stepper to scan speed
  int value;                      //declare a variable to stor the hall sensor reading                      

  Serial.println("SCAN");          //let the PC know a scan has started
  for(int point = 0; point < scanPoints-1; point++){  //go through each point
    Serial.print("LOOP: ");
    Serial.println(point);
    readMagXYZ();
    printMagXYZ();
    sMove(scanSteps);
    delay(scanDelay);                //optional delay before proceding
    //  if(scanPos + scanSteps > scanTravelMax){  //check that the next move wont crash the reader into the top
    //    break;                          //break out of the for loop if it will
    //  }
  }
  Serial.print("LOOP: ");
  Serial.println(scanPoints);
  readMagXYZ();
  printMagXYZ();  
  Serial.println("END");            //let the PC know we've finished

  delay(500);                        //short pause before returning the position to zero
  stepper.setSpeed(scanRapid);       //set the motor to rapid speed
  sMoveTo(0);
  
  Wire.beginTransmission(magAddress); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x02); //idle mode
  Wire.endTransmission();

}


void sMove(int go){
  stepper.step(go);
  scanPos += go;
}

void sMoveTo(int pos){
  stepper.step(pos - scanPos);
  scanPos = pos; 
}

int analogAvg(int pin){    //reads the analog pin a number of times and takes the average
  // even with hardware filtering, this is good practice
  int sensorValue = 0;

  for(int n = 0; n< samples; n++){    //loop 'samples' number of times
    sensorValue += analogRead(pin);   //take the measurement and add to the running total
    delay(2);                          //delay for the ADC to stabilize
  }
  sensorValue /= samples;              //divide by the number of samples take to find the average

  return(sensorValue);                //return the average

}


int magData[3]; //triple axis data

void readMagXYZ(){

  //Serial.println(">Waiting for DRDY pin to go low");
  while(digitalRead(drdyPin) == HIGH); //wait for data to be ready;
  //Serial.println(">DRDY is low, selecting register 0x03");
  //Tell the HMC5883 where to begin reading data
  Wire.beginTransmission(magAddress);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();
  //Serial.println(">Register selected, beginning read");


  //Read data from each axis, 2 registers per axis
  Wire.requestFrom(magAddress, 6);
  if(6<=Wire.available()){
    magData[0] = Wire.read()<<8; //X msb
    magData[0] |= Wire.read(); //X lsb
    magData[2] = Wire.read()<<8; //Z msb
    magData[2] |= Wire.read(); //Z lsb
    magData[1] = Wire.read()<<8; //Y msb
    magData[1] |= Wire.read(); //Y lsb
    //Serial.println("Data read");
  }else{
   Serial.println("SENSOR ERROR");
   Serial.println(">Not enough data recieved from Sensor"); 
    
  }

  //Print out values of each axis
  //Serial.print("x: ");


}

void printMagXYZ(){

    Serial.print(scanPos);
    Serial.print(": ");

  Serial.print(magData[0]);
  Serial.print(",");//  y: ");
  Serial.print(magData[1]);
  Serial.print(",");//  z: ");
  Serial.println(magData[2]);

}
