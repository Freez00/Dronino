#include <Wire.h>               //Include the Wire.h library so we can communicate with the gyro
#include <EEPROM.h>             //Include the EEPROM.h library so we can store information onto the EEPROM
#include <WiFiNINA.h>

//Declaring Global Variables
byte lowByte, highByte, type, gyro_address, error, clockspeed_ok;
byte roll_axis, pitch_axis, yaw_axis;
byte gyro_check_byte;
int address, cal_int, power_value;
unsigned long timer;
float gyro_pitch, gyro_roll, gyro_yaw;
float gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;

//Declare Network vars
int status = WL_IDLE_STATUS;
char ssid[] = "DroneSetup";
char pass[] = "1234567890";
unsigned int localPort = 2390;
char packetBuffer[256];

WiFiUDP Udp;


//Setu
void setup(){
  pinMode(LED_BUILTIN, OUTPUT);

  //Start the I2C as master
  Wire.begin();

  //Start the serial connetion @ 57600bps
  Serial.begin(57600);

  if (WiFi.status() == WL_NO_MODULE) {
    Serial.println("Communication with WiFi module failed!");
    // don't continue
    while (true);
  }

  //Print the network name (SSID);
  Serial.print("Creating access point named: ");
  Serial.println(ssid);

  status = WiFi.beginAP(ssid, pass);
  if (status != WL_AP_LISTENING) {
    while (true){
    }
  }

  while(status != WL_AP_CONNECTED){
    status = WiFi.status();
    Serial.println("Waiting for a device to connect and turn on the app!");
    delay(2000);
  }

  Serial.println(WiFi.localIP());
  Udp.begin(localPort);
  delay(250);

}

struct control_vars {
  char _x;
  short x;
  char _y;
  short y;
  char rotate_l[2];
  char rotate_r[2];
  char _p;
  short throttle;
};

//Main program
void loop(){
  //Show the YMFC-3D V2 intro
  
  Serial.println(F(""));
  Serial.println(F("==================================================="));
  Serial.println(F("System check"));
  Serial.println(F("==================================================="));
  delay(1000);
  Serial.println(F("Checking I2C clock speed."));
  delay(1000);
  
  Wire.setClock(400000);
  
  #if F_CPU == 16000000L
    clockspeed_ok = 1;
  #endif
    
  if(error == 0){
    //Searching MPU-6050 address
    Serial.println(F(""));
    Serial.println(F("==================================================="));
    Serial.println(F("Gyro search"));
    Serial.println(F("==================================================="));
    delay(2000);
    
    Serial.println(F("Searching for MPU-6050 on address 0x68/104"));
    delay(1000);
    if(search_gyro(0x68, 0x75) == 0x68){
      Serial.println(F("MPU-6050 found on address 0x68"));
      type = 1;
      gyro_address = 0x68;
    }
    
    if(type == 0){
      Serial.println(F("Searching for MPU-6050 on address 0x69/105"));
      delay(1000);
      if(search_gyro(0x69, 0x75) == 0x68){
        Serial.println(F("MPU-6050 found on address 0x69"));
        type = 1;
        gyro_address = 0x69;
      }
    }
    
    if(type == 0){
      Serial.println(F("No gyro device found!!! (ERROR 3)"));
      error = 1;
    }
    
    else{
      delay(3000);
      Serial.println(F(""));
      Serial.println(F("==================================================="));
      Serial.println(F("Gyro register settings"));
      Serial.println(F("==================================================="));
      start_gyro(); //Setup the gyro for further use
    }
  }
  
  //If the gyro is found we can setup the correct gyro axes.
  if(error == 0){
    delay(3000);
    Serial.println(F(""));
    Serial.println(F("==================================================="));
    Serial.println(F("Gyro calibration"));
    Serial.println(F("==================================================="));
    Serial.println(F("Don't move the quadcopter!! Calibration starts in 3 seconds"));
    delay(3000);
    Serial.println(F("Calibrating the gyro, this will take +/- 8 seconds"));
    Serial.print(F("Please wait"));

    //Taking multiple gyro data samples so we can determine the average gyro offset (calibration)
    for (cal_int = 0; cal_int < 2000 ; cal_int ++){
      if(cal_int % 100 == 0)Serial.print(F("."));
      gyro_signalen();
      gyro_roll_cal += gyro_roll;
      gyro_pitch_cal += gyro_pitch;
      gyro_yaw_cal += gyro_yaw;
      delay(4);
    }

    //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset
    gyro_roll_cal /= 2000;
    gyro_pitch_cal /= 2000;
    gyro_yaw_cal /= 2000;
    
    //Show the calibration results
    Serial.println(F(""));
    Serial.print(F("Axis 1 offset="));
    Serial.println(gyro_roll_cal);
    Serial.print(F("Axis 2 offset="));
    Serial.println(gyro_pitch_cal);
    Serial.print(F("Axis 3 offset="));
    Serial.println(gyro_yaw_cal);
    Serial.println(F(""));
    
    Serial.println(F("==================================================="));
    Serial.println(F("Gyro axes configuration"));
    Serial.println(F("==================================================="));
    

    control_vars* packet = (control_vars*) packetBuffer;
    //Detect the left wing up movement
    Serial.println(F("Lift the left side of the quadcopter to a 45 degree angle within 10 seconds"));
    //Check axis movement
    check_gyro_axes(1);
    if(error == 0){
      Serial.println(F("OK!"));
      Serial.print(F("Angle detection = "));
      Serial.println(roll_axis & 0b00000011);
      if(roll_axis & 0b10000000)Serial.println(F("Axis inverted = yes"));
      else Serial.println(F("Axis inverted = no"));
      Serial.println(F("Put the quadcopter back in its original position"));
      Serial.println(F("Press RotateL to continue!"));
      GetMovement('T');
      if (packet->rotate_l[1] == 'T'){ 
        Serial.print(F("=================================================="));
      }
      Serial.println("Releas RotateL Button");
      GetMovement('F');
      if (packet->rotate_l[1] == 'F'){ 
        Serial.print(F("=================================================="));
      }

      //Detect the nose up movement
      Serial.println(F(""));
      Serial.println(F(""));
      Serial.println(F("Lift the nose of the quadcopter to a 45 degree angle within 10 seconds"));
      //Check axis movement
      check_gyro_axes(2);
    }
    if(error == 0){
      Serial.println(F("OK!"));
      Serial.print(F("Angle detection = "));
      Serial.println(pitch_axis & 0b00000011);
      if(pitch_axis & 0b10000000)Serial.println(F("Axis inverted = yes"));
      else Serial.println(F("Axis inverted = no"));
      Serial.println(F("Put the quadcopter back in its original position"));
      Serial.println(F("Press RotateL to continue!"));
      GetMovement('T');
      if (packet->rotate_l[1] == 'T'){ 
        Serial.print(F("=================================================="));
      }
      Serial.println("Releas RotateL Button");
      GetMovement('F');
      if (packet->rotate_l[1] == 'F'){ 
        Serial.print(F("=================================================="));
      }
      
      //Detect the nose right movement
      Serial.println(F(""));
      Serial.println(F(""));
      Serial.println(F("Rotate the nose of the quadcopter 45 degree to the right within 10 seconds"));
      //Check axis movement
      check_gyro_axes(3);
    }
    if(error == 0){
      Serial.println(F("OK!"));
      Serial.print(F("Angle detection = "));
      Serial.println(yaw_axis & 0b00000011);
      if(yaw_axis & 0b10000000)Serial.println(F("Axis inverted = yes"));
      else Serial.println(F("Axis inverted = no"));
      Serial.println(F("Put the quadcopter back in its original position"));
      Serial.println(F("Press RotateL to continue!"));
      GetMovement('T');
      if (packet->rotate_l[1] == 'T'){ 
        Serial.print(F("=================================================="));
      }
      Serial.println("Releas RotateL Button");
      GetMovement('F');
      if (packet->rotate_l[1] == 'F'){ 
        Serial.print(F("=================================================="));
      }
    }
  }
  if(error == 0){
    Serial.println(F(""));
    Serial.println(F("==================================================="));
    Serial.println(F("LED test"));
    Serial.println(F("==================================================="));
    digitalWrite(LED_BUILTIN, HIGH);
    Serial.println(F("The LED should now be lit"));
    delay(5000);
    digitalWrite(LED_BUILTIN, LOW);
  }
  
  Serial.println(F(""));
  
  if(error == 0){
    Serial.println(F("==================================================="));
    Serial.println(F("Final setup check"));
    Serial.println(F("==================================================="));
    delay(1000);
    if(gyro_check_byte == 0b00000111){
      Serial.println(F("Gyro axes ok"));
    }
    else{
      Serial.println(F("Gyro exes verification failed!!! (ERROR 7)"));
      error = 1;
    }
  }     
  
  if(error == 0){
    //If all is good, store the information in the EEPROM
    Serial.println(F(""));
    Serial.println(F("==================================================="));
    Serial.println(F("Storing EEPROM information"));
    Serial.println(F("==================================================="));
    Serial.println(F("Writing EEPROM"));
    delay(1000);
    Serial.println(F("Done!"));
    EEPROM.write(0, roll_axis);
    EEPROM.write(1, pitch_axis);
    EEPROM.write(2, yaw_axis);
    EEPROM.write(3, type);
    EEPROM.write(4, gyro_address);
    //Write the EEPROM signature
    EEPROM.write(5, 'T'); 
    EEPROM.write(6, 'N');
    EEPROM.write(7, 'F');
        
    
    //To make sure evrything is ok, verify the EEPROM data.
    Serial.println(F("Verify EEPROM data"));
    delay(1000);
    if(roll_axis != EEPROM.read(0))error = 1;
    if(pitch_axis != EEPROM.read(1))error = 1;
    if(yaw_axis != EEPROM.read(2))error = 1;
    if(type != EEPROM.read(3))error = 1;
    if(gyro_address != EEPROM.read(4))error = 1;
    
    if('T' != EEPROM.read(5))error = 1;
    if('N' != EEPROM.read(6))error = 1;
    if('F' != EEPROM.read(7))error = 1;
  
    if(error == 1)Serial.println(F("EEPROM verification failed!!! (ERROR 5)"));
    else Serial.println(F("Verification done"));
  }
  
  
  if(error == 0){
    Serial.println(F("Setup is finished."));
    Serial.println(F("You can now calibrate the esc's and upload the YMFC-AL code."));
  }
  else{
   Serial.println(F("The setup is aborted due to an error."));
   Serial.println(F("Check the Q and A page of the YMFC-AL project on:"));
   Serial.println(F("www.brokking.net for more information about this error."));
  }
  while(1);
}

//Search for the gyro and check the Who_am_I register
byte search_gyro(int gyro_address, int who_am_i){
  Wire.beginTransmission(gyro_address);
  Wire.write(who_am_i);
  Wire.endTransmission();
  Wire.requestFrom(gyro_address, 1);
  timer = millis() + 100;
  while(Wire.available() < 1 && timer > millis());
  lowByte = Wire.read();
  address = gyro_address;
  return lowByte;
}

void start_gyro(){
  //Setup the L3G4200D or L3GD20H
  if(type == 2 || type == 3){
    Wire.beginTransmission(address);                          
    Wire.write(0x20);                                         
    Wire.write(0x0F);                                      
    Wire.endTransmission();                                

    Wire.beginTransmission(address);                    
    Wire.write(0x20);                                       
    Wire.endTransmission();                                    
    Wire.requestFrom(address, 1);                              
    while(Wire.available() < 1);                             
    Serial.print(F("Register 0x20 is set to:"));
    Serial.println(Wire.read(),BIN);

    Wire.beginTransmission(address);                          
    Wire.write(0x23);                                        
    Wire.write(0x90);                                        
    Wire.endTransmission();                                   
    
    Wire.beginTransmission(address);                           
    Wire.write(0x23);                                          
    Wire.endTransmission();                                   
    Wire.requestFrom(address, 1);                            
    while(Wire.available() < 1);                               
    Serial.print(F("Register 0x23 is set to:"));
    Serial.println(Wire.read(),BIN);

  }
  //Setup the MPU-6050
  if(type == 1){
    
    Wire.beginTransmission(address);                        
    Wire.write(0x6B);                                      
    Wire.write(0x00);                                        
    Wire.endTransmission();                                  
    
    Wire.beginTransmission(address);                       
    Wire.write(0x6B);                                    
    Wire.endTransmission();                                  
    Wire.requestFrom(address, 1);                            
    while(Wire.available() < 1);                              
    Serial.print(F("Register 0x6B is set to:"));
    Serial.println(Wire.read(),BIN);
    
    Wire.beginTransmission(address);                        
    Wire.write(0x1B);                                         
    Wire.write(0x08);                                       
    Wire.endTransmission();                                    
    
    Wire.beginTransmission(address);                          
    Wire.write(0x1B);                                       
    Wire.endTransmission();                                 
    Wire.requestFrom(address, 1);                           
    while(Wire.available() < 1);                            
    Serial.print(F("Register 0x1B is set to:"));
    Serial.println(Wire.read(),BIN);

  }
}

void gyro_signalen(){
  if(type == 2 || type == 3){
    Wire.beginTransmission(address);                        
    Wire.write(168);                                          
    Wire.endTransmission();                                   
    Wire.requestFrom(address, 6);                            
    while(Wire.available() < 6);                               
    lowByte = Wire.read();                                   
    highByte = Wire.read();                                   
    gyro_roll = ((highByte<<8)|lowByte);                    
    if(cal_int == 2000)gyro_roll -= gyro_roll_cal;           
    lowByte = Wire.read();                                 
    highByte = Wire.read();                                   
    gyro_pitch = ((highByte<<8)|lowByte);                   
    if(cal_int == 2000)gyro_pitch -= gyro_pitch_cal;          
    lowByte = Wire.read();                                   
    highByte = Wire.read();                              
    gyro_yaw = ((highByte<<8)|lowByte);                    
    if(cal_int == 2000)gyro_yaw -= gyro_yaw_cal;            
  }
  if(type == 1){
    Wire.beginTransmission(address);                         
    Wire.write(0x43);                                  
    Wire.endTransmission();                           
    Wire.requestFrom(address,6);                   
    while(Wire.available() < 6);              
    gyro_roll=Wire.read()<<8|Wire.read();                   
    if(cal_int == 2000)gyro_roll -= gyro_roll_cal;         
    gyro_pitch=Wire.read()<<8|Wire.read();                    
    if(cal_int == 2000)gyro_pitch -= gyro_pitch_cal;          
    gyro_yaw=Wire.read()<<8|Wire.read();
    if(cal_int == 2000)gyro_yaw -= gyro_yaw_cal;
  }
}

//Check if the angular position of a gyro axis is changing within 10 seconds
void check_gyro_axes(byte movement){
  byte trigger_axis = 0;
  float gyro_angle_roll, gyro_angle_pitch, gyro_angle_yaw;
  //Reset all axes
  gyro_angle_roll = 0;
  gyro_angle_pitch = 0;
  gyro_angle_yaw = 0;
  gyro_signalen();
  timer = millis() + 10000;    
  while(timer > millis() && gyro_angle_roll > -30 && gyro_angle_roll < 30 && gyro_angle_pitch > -30 && gyro_angle_pitch < 30 && gyro_angle_yaw > -30 && gyro_angle_yaw < 30){
    gyro_signalen();
    if(type == 2 || type == 3){
      gyro_angle_roll += gyro_roll * 0.00007;
      gyro_angle_pitch += gyro_pitch * 0.00007;
      gyro_angle_yaw += gyro_yaw * 0.00007;
    }
    if(type == 1){
      gyro_angle_roll += gyro_roll * 0.0000611;
      gyro_angle_pitch += gyro_pitch * 0.0000611;
      gyro_angle_yaw += gyro_yaw * 0.0000611;
    }
    
    delayMicroseconds(3700);
  }

  //Assign the moved axis to the orresponding function (pitch, roll, yaw)
  if((gyro_angle_roll < -30 || gyro_angle_roll > 30) && gyro_angle_pitch > -30 && gyro_angle_pitch < 30 && gyro_angle_yaw > -30 && gyro_angle_yaw < 30){
    gyro_check_byte |= 0b00000001;
    if(gyro_angle_roll < 0)trigger_axis = 0b10000001;
    else trigger_axis = 0b00000001;
  }
  if((gyro_angle_pitch < -30 || gyro_angle_pitch > 30) && gyro_angle_roll > -30 && gyro_angle_roll < 30 && gyro_angle_yaw > -30 && gyro_angle_yaw < 30){
    gyro_check_byte |= 0b00000010;
    if(gyro_angle_pitch < 0)trigger_axis = 0b10000010;
    else trigger_axis = 0b00000010;
  }
  if((gyro_angle_yaw < -30 || gyro_angle_yaw > 30) && gyro_angle_roll > -30 && gyro_angle_roll < 30 && gyro_angle_pitch > -30 && gyro_angle_pitch < 30){
    gyro_check_byte |= 0b00000100;
    if(gyro_angle_yaw < 0)trigger_axis = 0b10000011;
    else trigger_axis = 0b00000011;
  }
  
  if(trigger_axis == 0){
    error = 1;
    Serial.println(F("No angular motion is detected in the last 10 seconds!!! (ERROR 4)"));
  }
  else
  if(movement == 1)roll_axis = trigger_axis;
  if(movement == 2)pitch_axis = trigger_axis;
  if(movement == 3)yaw_axis = trigger_axis;
  
}

void GetMovement(char charrr){
  control_vars* packet = (control_vars*) packetBuffer;
  while(packet->rotate_l[1] != charrr){
    int packetSize = Udp.parsePacket();

    if (packetSize) {
      int len = Udp.read(packetBuffer, 255);

      if (len > 0) {
        packetBuffer[len] = 0;
      }
    }
  }  
}