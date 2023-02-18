#include <Wire.h>
#include <EEPROM.h>
#include <WiFiNINA.h>

//Network vars
String ssid;
String pass;

unsigned int localPort = 2390;
char packetBuffer[256];
int status = WL_IDLE_STATUS;
int factory_reset_count = EEPROM.read(8);
int packetSize;

WiFiUDP Udp;

//PID gain and limit settings
float pid_p_gain_roll = 1;
float pid_i_gain_roll = 0.02;
float pid_d_gain_roll = 10;
int pid_max_roll = 400;

float pid_p_gain_pitch = pid_p_gain_roll;
float pid_i_gain_pitch = pid_i_gain_roll;
float pid_d_gain_pitch = pid_d_gain_roll;
int pid_max_pitch = pid_max_roll;

float pid_p_gain_yaw = 4.0;
float pid_i_gain_yaw = 0.02;
float pid_d_gain_yaw = 0.0;
int pid_max_yaw = 400;

//Declaring global variables
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
byte eeprom_data[9];
int esc_1, esc_2, esc_3, esc_4;
int throttle, stick_x, stick_y, battery_voltage;
int cal_int, start, gyro_address;
int temperature;
int acc_axis[4], gyro_axis[4];
float roll_level_adjust, pitch_level_adjust;

long acc_x, acc_y, acc_z, acc_total_vector;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
unsigned long loop_timer;
double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_axis_cal[4];
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll, temp_pitch, temp_roll;
bool gyro_angles_set, rotate_r, rotate_l;

//Motors
int LF = 10;
int RF = 5;
int LR = 12;
int RR = 4;

//Setup 
void setup(){
  factory_reset_count++;

  //Set digital pins 3, 4, 5, 11, 12 as output
  DDRD |= B00111000;
  DDRB |= B00010100; 
  
  //Access point factory reset and EEPROM reading
  EEPROM.write(8, factory_reset_count);
  if(factory_reset_count == 3){
    writeStringToEEPROM(9, "ArduinoDrone");
    writeStringToEEPROM(22, "1234567890");
    EEPROM.write(8, 0);
  }

  ssid = readStringFromEEPROM(9);
  pass = readStringFromEEPROM(9 + 1 + ssid.length());

  //Starting the Access Point (Acts as a router)
  status = WiFi.beginAP(ssid.c_str(), pass.c_str());
  if (status != WL_AP_LISTENING) {
    // Slow blink to indicate an error with creating Access point
    while (true){
      digitalWrite(3, HIGH);
      delay(1500);
      digitalWrite(3, LOW);
      delay(1500);
    }
  }

  delay(1000);

  //Starting UDP commmunication
  Udp.begin(localPort);

  //Waiting for a device to connect and turn on the app with led indication
  while(status != WL_AP_CONNECTED){
    status = WiFi.status();

    digitalWrite(3, HIGH);
    delay(200);
    digitalWrite(3, LOW);
    delay(200);
    digitalWrite(3, HIGH);
    delay(2000);
  }
  while(packetBuffer[0] != 'S'){

    packetSize = Udp.parsePacket();
    if (packetSize) {
      // read the packet into packetBufffer
      int len = Udp.read(packetBuffer, 255);
      if (len > 0) {
      packetBuffer[len] = 0;
      }

      if (packetBuffer[0] == 'N'){
        int ssid_len = packetBuffer[1];
        int pass_len = packetBuffer[1 + ssid_len];
        int ptr = 9;

        EEPROM.write(ptr++, ssid_len);
        for (int i = 0; i < ssid_len; i++){
          EEPROM.write(ptr++, packetBuffer[2 + i]);
        }

        EEPROM.write(ptr++, pass_len);
        for (int i = 0; i < pass_len; i++){
          EEPROM.write(ptr++, packetBuffer[3 + ssid_len + i]);
        }
        pinMode(13, OUTPUT);
      }
    }
  }
  EEPROM.write(8, 0);

  //Copy the EEPROM data for fast access data
  for(start = 0; start <= 8; start++)eeprom_data[start] = EEPROM.read(start);
  start = 0;
  gyro_address = eeprom_data[4];

  //Start the I2C as master
  Wire.begin();

  //Set the I2C clock speed to 400kHz.
  Wire.setClock(400000);

  //Use the led for data verification
  digitalWrite(3, HIGH);

  //Check the EEPROM signature to make sure that the setup program was executed
  while(eeprom_data[5] != 'T' || eeprom_data[6] != 'N' || eeprom_data[7] != 'F')delay(10);
  if(eeprom_data[3] == 2 || eeprom_data[3] == 3)delay(10);

  //Use the led for data verification
  digitalWrite(3, LOW);

  //Set the specific gyro registers
  set_gyro_registers();

  //Wait 5 seconds before continuing.
  for (cal_int = 0; cal_int < 1250 ; cal_int ++){

    //Set port 4, 5, 11, 12 to HIGH
    PORTD |= B00110000;
    PORTB |= B00010100;

    delayMicroseconds(1000);

    //Set port 4, 5, 11, 12 to LOW
    PORTD &= B00110000;
    PORTB &= B00010100;

    delayMicroseconds(3000);
  }

  //Gyro offset calibration (2000 measures taken)
  for (cal_int = 0; cal_int < 2000 ; cal_int ++){  

    //Change the led status to indicate calibration
    if(cal_int % 15 == 0)digitalWrite(3, !digitalRead(3));

    //Read Gyro
    gyro_signalen();
    gyro_axis_cal[1] += gyro_axis[1];
    gyro_axis_cal[2] += gyro_axis[2];
    gyro_axis_cal[3] += gyro_axis[3];
  }
  //Devide the taken measures by 2000 to get average
  gyro_axis_cal[1] /= 2000;
  gyro_axis_cal[2] /= 2000;
  gyro_axis_cal[3] /= 2000;

  start = 0;

  //Load the battery voltage to the battery_voltage variable
  battery_voltage = (analogRead(0) + 65) * 1.2317;

  loop_timer = micros();

  //When everything is done, turn off the led
  digitalWrite(3, LOW);

}

struct control_vars {
  short x;
  short y;
  char rotate_l;
  char rotate_r;
  short throttle;
};

//Main program loop
void loop(){
  packetSize = Udp.parsePacket();

  if (packetSize) {
    // read the packet into packetBufffer
    int len = Udp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = 0;
    }
    
    //Set control vars by reading the UDP packet
    control_vars* packet = (control_vars*) packetBuffer;
    stick_x = packet->x;
    stick_y = packet->y;
    throttle = packet->throttle;
    rotate_l = packet->rotate_l == 'T';
    rotate_r = packet->rotate_r == 'T';
    
  }

  //Convert raw Gyro data to degrees/sec
  gyro_roll_input = (gyro_roll_input * 0.7) + ((gyro_roll / 65.5) * 0.3);   
  gyro_pitch_input = (gyro_pitch_input * 0.7) + ((gyro_pitch / 65.5) * 0.3);
  gyro_yaw_input = (gyro_yaw_input * 0.7) + ((gyro_yaw / 65.5) * 0.3);      

  //Gyro angle calculations
  angle_pitch += gyro_pitch * 0.0000611;
  angle_roll += gyro_roll * 0.0000611;

  angle_pitch -= angle_roll * sin(gyro_yaw * 0.000001066);
  angle_roll += angle_pitch * sin(gyro_yaw * 0.000001066);

  //Accelerometer angle calculations
  acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));
  
  if(abs(acc_y) < acc_total_vector){
    angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;
  }
  if(abs(acc_x) < acc_total_vector){
    angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;
  }
  //Correct Gyro drift and error
  angle_pitch_acc -= 0.0;
  angle_roll_acc -= 0.0;
  
  angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;
  angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;

  //temp_pitch = angle_pitch - 2;
  //temp_roll = angle_roll - 4;
  pitch_level_adjust = angle_pitch * 15;
  roll_level_adjust = angle_roll * 15;
  

  //Turns on the motors if conditions are met
  if(throttle < 1050 && rotate_l)start = 1;

  if(start == 1 && throttle < 1050 && !rotate_l){
    start = 2;

    angle_pitch = angle_pitch_acc;
    angle_roll = angle_roll_acc;
    gyro_angles_set = true;

    //Reset the PID controllers for a smooth take-off
    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;
  }
  //Stops the motors if conditions are met
  if(start == 2 && throttle < 1050 && rotate_r)start = 0;

  //Roll controls
  pid_roll_setpoint = 0;
  
  if(stick_x > 1508)pid_roll_setpoint = stick_x - 1508;
  else if(stick_x < 1492)pid_roll_setpoint = stick_x - 1492;

  pid_roll_setpoint -= roll_level_adjust;
  pid_roll_setpoint /= 3.0;
  //Pitch controls
  pid_pitch_setpoint = 0;

  if(stick_y > 1508)pid_pitch_setpoint = stick_y - 1508;
  else if(stick_y < 1492)pid_pitch_setpoint = stick_y - 1492;

  pid_pitch_setpoint -= pitch_level_adjust;
  pid_pitch_setpoint /= 3.0;
  //Rotation controls
  pid_yaw_setpoint = 0;
  
  //Prevent rotation while turning off the motors
  if(throttle > 1050){
    if(rotate_r)pid_yaw_setpoint = (1750 - 1508)/3.0;
    else if(rotate_l)pid_yaw_setpoint = (1250 - 1492)/3.0;
  }
  
  //Calculates PID
  calculate_pid();
  //Warn if battery voltage is low(no charge)
  battery_voltage = battery_voltage * 0.92 + (analogRead(0) + 65) * 0.09853;
  if(battery_voltage < 1000 && battery_voltage > 600)digitalWrite(3, HIGH);


  //Starts the motors and calculates their pulse (speed of spinning) with min pulse of 1000us(not spinning) and max pulse of 2000us(full throttle)
  if (start == 2){
    if (throttle > 1800) throttle = 1800;
    esc_1 = throttle - pid_output_pitch + pid_output_roll - pid_output_yaw;
    esc_2 = throttle + pid_output_pitch + pid_output_roll + pid_output_yaw;
    esc_3 = throttle + pid_output_pitch - pid_output_roll - pid_output_yaw;
    esc_4 = throttle - pid_output_pitch - pid_output_roll + pid_output_yaw;
    //Correct pulse in case of voltage drops to keep the flight consistent

    //Keep the motors spinning
    if (esc_1 < 1100) esc_1 = 1100;
    if (esc_2 < 1100) esc_2 = 1100;
    if (esc_3 < 1100) esc_3 = 1100;
    if (esc_4 < 1100) esc_4 = 1100;
    if(esc_1 > 2000)esc_1 = 2000;
    if(esc_2 > 2000)esc_2 = 2000;
    if(esc_3 > 2000)esc_3 = 2000;
    if(esc_4 > 2000)esc_4 = 2000; 
  }

  //Keep the motors off
  else{
    esc_1 = 1000;
    esc_2 = 1000;
    esc_3 = 1000;
    esc_4 = 1000;
  }


  //Wait until 4ms of loop time has passed to give a consistent pusle to the motors 
  while(micros() - loop_timer < 4000);

  loop_timer = micros();

  PORTD |= B00110000;
  PORTB |= B00010100;

  ///Calculate the time of the faling edge of the esc-1 pulse.
  timer_channel_1 = esc_1 + loop_timer;
  timer_channel_2 = esc_2 + loop_timer;
  timer_channel_3 = esc_3 + loop_timer;
  timer_channel_4 = esc_4 + loop_timer;
  
  gyro_signalen();

  //Wait untill pin 4, 5, 11 and 12 are low
  while(digitalRead(LR) == HIGH || digitalRead(RR) == HIGH || digitalRead(LF) == HIGH || digitalRead(RF) == HIGH){
    esc_loop_timer = micros();
    if(timer_channel_1 <= esc_loop_timer)PORTD &= B11011111;
    if(timer_channel_2 <= esc_loop_timer)PORTD &= B11101111;
    if(timer_channel_3 <= esc_loop_timer)PORTB &= B11101111;
    if(timer_channel_4 <= esc_loop_timer)PORTB &= B11111011;
  }
}

//Subroutine for reading the gyro
void gyro_signalen(){
  //Read the MPU-6050
  if(eeprom_data[3] == 1){
    Wire.beginTransmission(gyro_address);
    Wire.write(0x3B);
    Wire.endTransmission();
    Wire.requestFrom(gyro_address,14);
    
    while(Wire.available() < 14);
    acc_axis[1] = Wire.read()<<8|Wire.read();
    acc_axis[2] = Wire.read()<<8|Wire.read();
    acc_axis[3] = Wire.read()<<8|Wire.read();
    temperature = Wire.read()<<8|Wire.read();
    gyro_axis[1] = Wire.read()<<8|Wire.read();
    gyro_axis[2] = Wire.read()<<8|Wire.read();
    gyro_axis[3] = Wire.read()<<8|Wire.read();
  }

  if(cal_int == 2000){
    gyro_axis[1] -= gyro_axis_cal[1];
    gyro_axis[2] -= gyro_axis_cal[2];
    gyro_axis[3] -= gyro_axis_cal[3];
  }
  gyro_roll = gyro_axis[eeprom_data[0] & 0b00000011];
  if(eeprom_data[0] & 0b10000000)gyro_roll *= -1;
  gyro_pitch = gyro_axis[eeprom_data[1] & 0b00000011];
  if(eeprom_data[1] & 0b10000000)gyro_pitch *= -1;
  gyro_yaw = gyro_axis[eeprom_data[2] & 0b00000011];
  if(eeprom_data[2] & 0b10000000)gyro_yaw *= -1;

  acc_x = acc_axis[eeprom_data[1] & 0b00000011];
  if(eeprom_data[1] & 0b10000000)acc_x *= -1;
  acc_y = acc_axis[eeprom_data[0] & 0b00000011];
  if(eeprom_data[0] & 0b10000000)acc_y *= -1;
  acc_z = acc_axis[eeprom_data[2] & 0b00000011];
  if(eeprom_data[2] & 0b10000000)acc_z *= -1;
}

//Subroutine for calculating pid outputs
void calculate_pid(){

  //Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;

  //Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;
}

void set_gyro_registers(){
  //Setup the MPU-6050
  if(eeprom_data[3] == 1){
    Wire.beginTransmission(gyro_address);
    Wire.write(0x6B);
    Wire.write(0x00);
    Wire.endTransmission();

    Wire.beginTransmission(gyro_address);
    Wire.write(0x1B);
    Wire.write(0x08);
    Wire.endTransmission();

    Wire.beginTransmission(gyro_address);
    Wire.write(0x1C);
    Wire.write(0x10);
    Wire.endTransmission();

    //Double check reguster values
    Wire.beginTransmission(gyro_address);
    Wire.write(0x1B);
    Wire.endTransmission();
    Wire.requestFrom(gyro_address, 1);
    while(Wire.available() < 1);
    if(Wire.read() != 0x08){
      digitalWrite(3,HIGH);
      while(1)delay(10);
    }

    Wire.beginTransmission(gyro_address);
    Wire.write(0x1A);
    Wire.write(0x03);
    Wire.endTransmission();

  }  
}

//Methid for reading strings from the EEPROM (ssid, pass)
String readStringFromEEPROM(int addrOffset){
  int newStrLen = EEPROM.read(addrOffset);
  char data[newStrLen + 1];

  for (int i = 0; i < newStrLen; i++)
  {
    data[i] = EEPROM.read(addrOffset + 1 + i);
  }
  data[newStrLen] = '\0';

  return String(data);
}

//Methid for writing strings to the EEPROM (ssid, pass)
void writeStringToEEPROM(int addrOffset, const String &strToWrite){
  byte len = strToWrite.length();
  EEPROM.write(addrOffset, len);

  for (int i = 0; i < len; i++)
  {
    EEPROM.write(addrOffset + 1 + i, strToWrite[i]);
  }
}
