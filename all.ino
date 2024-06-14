#include <WiFi.h>
#include <Wire.h>
#include <MPU6050.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

#define MCU_SIM_BAUDRATE        115200
#define GPS_BAUDRATE 115200   // The default baudrate of NEO-6M is 9600
#define simSerial               Serial2
#define MCU_SIM_TX_PIN              16
#define MCU_SIM_RX_PIN              17
#define PHONE_NUMBER                "+84898371548"

HardwareSerial gpsSerial(0); // Create a HardwareSerial object for GPS module (Serial1)  
WiFiServer server(80);
MPU6050 mpu;
TinyGPSPlus gps;             // The TinyGPS++ object

const char* ssid = "BLUEIS";
const char* password = "chanceorchange1";
const int limitSwitchPin = 32; // Chân số của Arduino được kết nối với chân NO của công tắc hành trình
const int MPU_addr = 0x68; // I2C address of the MPU-6050

float ax = 0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0;
float lat = 0, lng = 0;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
int angleChange = 0;
int MQ3pin = 34; 
bool fall = false; //stores if a fall has occurred
bool trigger1 = false; //stores if first trigger (lower threshold) has occurred
bool trigger2 = false; //stores if second trigger (upper threshold) has occurred
bool trigger3 = false; //stores if third trigger (orientation change) has occurred
byte trigger1count = 0; //stores the counts past since trigger 1 was set true
byte trigger2count = 0; //stores the counts past since trigger 2 was set true
byte trigger3count = 0; //stores the counts past since trigger 3 was set true

float MQ3() 
{
  int ADC = analogRead(MQ3pin); // Đọc giá trị analog từ cảm biến MQ-3
  int ppm = ((((3.3 *10)/(ADC*3.3/4095))-10)/0.11)*4.84-2.68;
  //float voltage = sensorValue * (3.3 / 1023.0); // Chuyển đổi giá trị analog sang điện áp (5V)
  //float alcoholConcentration = map(voltage, 0.0, 3.3, 0.0, 10.0); // Chuyển đổi điện áp thành nồng độ cồn (0-10 mg/L)

  Serial.print("Nồng độ cồn trong hơi thở: ");
  Serial.print(ppm);
  //Serial.print(", Voltage: ");
  //Serial.print(voltage);
  //Serial.print("Nồng độ cồn trong hơi thở: ");
  //Serial.print(alcoholConcentration);
  Serial.println(" ppm");

  delay(1000); // Đợi 1 giây trước khi đọc lại giá trị từ cảm biến
  return ppm;
}

bool SwitchState() {
  int limitSwitchState = digitalRead(limitSwitchPin); // Đọc trạng thái của công tắc hành trình
  bool State = false;
  //Serial.println(limitSwitchState);
  if (limitSwitchState == 0) { // Nếu công tắc hành trình đóng (Normally Open)
    Serial.println("Công tắc hành trình đã đóng");
    State = true;
  } 
  
  delay(1000); // Đợi một khoảng thời gian ngắn trước khi đọc lại trạng thái
  return State;
}

void ControlMotor() {
  // Thêm mã lập trình của bạn tại đây
  WiFiClient client = server.available();   // listen for incoming clients
  client.println("ON");
  client.println();
  delay(10000);
  // close the connection:
  client.stop();
  Serial.println("Client Disconnected.");
}

void FallDetect() 
 {
   mpu_read();
   ax = (AcX - 2050) / 16384.00;
   ay = (AcY - 77) / 16384.00;
   az = (AcZ - 1947) / 16384.00;
   gx = (GyX + 270) / 131.07;
   gy = (GyY - 351) / 131.07;
   gz = (GyZ + 136) / 131.07;
   // calculating Amplitute vactor for 3 axis
   float Raw_Amp = pow(pow(ax, 2) + pow(ay, 2) + pow(az, 2), 0.5);
   int Amp = Raw_Amp * 10;  // Mulitiplied by 10 bcz values are between 0 to 1
   Serial.println(Amp);
 
if (Amp <= 2 && trigger2 == false) { //if AM breaks lower threshold (0.4g)     
trigger1 = true;     
Serial.println("TRIGGER 1 ACTIVATED");   
}   
if (trigger1 == true) {     
trigger1count++;     
if (Amp >= 12) { //if AM breaks upper threshold (3g)
       trigger2 = true;
       Serial.println("TRIGGER 2 ACTIVATED");
       trigger1 = false; trigger1count = 0;
     }
   }
   if (trigger2 == true) {
     trigger2count++;
     angleChange = pow(pow(gx, 2) + pow(gy, 2) + pow(gz, 2), 0.5); Serial.println(angleChange);
     if (angleChange >= 30 && angleChange <= 100) { //if orientation changes by between 80-100 degrees       
trigger3 = true; trigger2 = false; trigger2count = 0;       
Serial.println(angleChange);       
Serial.println("TRIGGER 3 ACTIVATED");     
}   
}   
if (trigger3 == true) {     
trigger3count++;     
if (trigger3count >= 10) {
       angleChange = pow(pow(gx, 2) + pow(gy, 2) + pow(gz, 2), 0.5);
       //delay(10);
       Serial.println(angleChange);
       if ((angleChange >= 0) && (angleChange <= 10)) { //if orientation changes remains between 0-10 degrees         
fall = true; trigger3 = false; trigger3count = 0;         
Serial.println(angleChange);       }       
else { //user regained normal orientation         
trigger3 = false; trigger3count = 0;         
Serial.println("TRIGGER 3 DEACTIVATED");       
}     
}   
}   
if (fall == true) { //in event of a fall detection     
Serial.println("FALL DETECTED");    
}   
if (trigger2count >= 6) { //allow 0.5s for orientation change
     trigger2 = false; trigger2count = 0;
     Serial.println("TRIGGER 2 DECACTIVATED");
   }
   if (trigger1count >= 6) { //allow 0.5s for AM to break upper threshold
     trigger1 = false; trigger1count = 0;
     Serial.println("TRIGGER 1 DECACTIVATED");
   }
   delay(100);
 }
void mpu_read() {
   Wire.beginTransmission(MPU_addr);
   Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
   Wire.endTransmission(false);
   Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
   AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
   AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
   AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
   Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
   GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
   GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
   GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
 }

void GPS() {
  //updateSerial();
  while (gpsSerial.available() > 0)
    if (gps.encode(gpsSerial.read()))
      displayInfo();
      lat = gps.location.lat();
      lng = gps.location.lng();
  if (millis() > 5000 && gps.charsProcessed() < 10)
  {
    Serial.println(F("No GPS detected: check wiring."));
    while (true);
  }
}
void displayInfo()
{
  Serial.print(F("Location: "));
  if (gps.location.isValid()){
    Serial.print("Lat: ");
    Serial.print(gps.location.lat(), 6);
    Serial.print(F(","));
    Serial.print("Lng: ");
    Serial.print(gps.location.lng(), 6);
  Serial.println();
  }  
  else
  {
    Serial.print(F("INVALID"));
  }
}
void updateSerial()
{
  delay(500);
  while (Serial.available())
  {
    gpsSerial.write(Serial.read());//Forward what Serial received to Software Serial Port
  }
  while (gpsSerial.available())
  {
    Serial.write(gpsSerial.read());//Forward what Software Serial received to Serial Port
  }
}

void sim_at_wait()
{
    delay(100);
    while (simSerial.available()) {
        Serial.write(simSerial.read());
    }
}

bool sim_at_cmd(String cmd){
    simSerial.println(cmd);
    sim_at_wait();
    return true;
}

bool sim_at_send(char c){
    simSerial.write(c);
    return true;
}

void SMS()
{
    sim_at_cmd("AT+CMGF=1");
    String temp = "AT+CMGS=\"";
    temp += (String)PHONE_NUMBER;
    temp += "\"";
    sim_at_cmd(temp);
    //sim_at_cmd("AT+CSCS=\"GSM\"");
    //sim_at_cmd((String)PHONE_NUMBER);
    sim_at_cmd("Fall detected at location:");
    sim_at_cmd("https://www.google.com/maps/search/?api=1&query=");
    sim_at_cmd("(string)lat");
    sim_at_cmd(",");
    sim_at_cmd("(string)lng, 6");
    // End charactor for SMS
    sim_at_send(0x1A);
}
void call()
{
    String temp = "ATD";
    temp += PHONE_NUMBER;
    temp += ";";
    sim_at_cmd(temp); 

    delay(20000);

    // Hang up
    sim_at_cmd("ATH"); 
}
/*void SMS()
{
SIM_SERIAL.begin(MCU_SIM_BAUDRATE, SERIAL_8N1, MCU_SIM_RX_PIN, MCU_SIM_TX_PIN);
        delay(1000);
        SIM_SERIAL.println("AT+CMGF=1");    // Sets the GSM Module in Text Mode
        delay(500);
        SIM_SERIAL.print("AT+CMGS=\"");
        SIM_SERIAL.print(PHONE_NUMBER);
        SIM_SERIAL.println("\"");
        delay(500);
        SIM_SERIAL.print("Fall detected at location: ");
        SIM_SERIAL.print("https://www.google.com/maps/search/?api=1&query=");
        SIM_SERIAL.print();
        SIM_SERIAL.print(",");
        SIM_SERIAL.print(gpsData.longitude, 6);
        delay(500);
        SIM_SERIAL.write(26); // Ctrl+Z to send SMS
        delay(500);

        
       
      
    vTaskDelay(100 / portTICK_PERIOD_MS); // Delay 100ms
}*/

void setup() {
  Serial.begin(115200);
  Serial.println("MQ3 warming up!");
  delay(20000); // allow the MQ3 to warm up

  pinMode(limitSwitchPin, INPUT_PULLUP); // Thiết lập chân là INPUT_PULLUP
gpsSerial.begin(9600);
  delay(3000);
  WiFi.begin(ssid, password);  // Kết nối với mạng WiFi

  while (WiFi.status() != WL_CONNECTED) {  // Chờ kết nối thành công
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }

  Serial.println("Connected to WiFi");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  server.begin();

  Wire.begin();
   mpu.initialize();
  
  // Đặt cấu hình cho MPU6050
   mpu.setSleepEnabled(false);
   Wire.beginTransmission(MPU_addr);
   Wire.write(0x6B);  // PWR_MGMT_1 register
   Wire.write(0);     // set to zero (wakes up the MPU-6050)
   Wire.endTransmission(true);
   Serial.println("Wrote to IMU");

   gpsSerial.begin(GPS_BAUDRATE);
  Serial.println(F("Arduino Uno R3 - GPS module"));

  delay(20);
 
  Serial.println("\n\n\n\n-----------------------\nSystem started!!!!");
    // Delay 8s for power on
    delay(8000);
    //simSerial.begin(MCU_SIM_BAUDRATE, SERIAL_8N1, MCU_SIM_RX_PIN, MCU_SIM_TX_PIN);

    // Check AT Command
    sim_at_cmd("AT");

    // Product infor
    sim_at_cmd("ATI");

    // Check SIM Slot
    sim_at_cmd("AT+CPIN?");

    // Check Signal Quality
    sim_at_cmd("AT+CSQ");

    sim_at_cmd("AT+CIMI");

    pinMode(2,OUTPUT); 
    digitalWrite(2,HIGH);

    // Delay 5s
    delay(5000);   
}

/*void loop() {
  if (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isValid()) {
        Serial.print(F("- latitude: "));
        Serial.println(gps.location.lat());

        Serial.print(F("- longitude: "));
        Serial.println(gps.location.lng());

        Serial.print(F("- altitude: "));
        if (gps.altitude.isValid())
          Serial.println(gps.altitude.meters());
        else
          Serial.println(F("INVALID"));
      } else {
        Serial.println(F("- location: INVALID"));
      }

      Serial.print(F("- speed: "));
      if (gps.speed.isValid()) {
        Serial.print(gps.speed.kmph());
        Serial.println(F(" km/h"));
      } else {
        Serial.println(F("INVALID"));
      }

      Serial.print(F("- GPS date&time: "));
      if (gps.date.isValid() && gps.time.isValid()) {
        Serial.print(gps.date.year());
        Serial.print(F("-"));
        Serial.print(gps.date.month());
        Serial.print(F("-"));
        Serial.print(gps.date.day());
        Serial.print(F(" "));
        Serial.print(gps.time.hour());
        Serial.print(F(":"));
        Serial.print(gps.time.minute());
        Serial.print(F(":"));
        Serial.println(gps.time.second());
      } else {
        Serial.println(F("INVALID"));
      }

      Serial.println();
    }
  }

}*/



//SMS

void loop() 
{   
  bool helmet = SwitchState(); 
  float alcon = MQ3();
  if (helmet == true)
  {
    if (alcon < 350)
    {
    ControlMotor();
    FallDetect();
    if (fall == true)
    {
      GPS();
      SMS();
      fall = false;
    }
      if (Serial.available()){
          char c = Serial.read();
          simSerial.write(c);
      }
      sim_at_wait();
    } 
  }
}