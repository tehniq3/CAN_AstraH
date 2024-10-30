// based on https://randompagedesu.wordpress.com/2020/05/25/data-logging-canbus-obd2-part1/
// small changes by Nicu FLORICA (niq_ro)
// to be test: https://github.com/sepp89117/Opel-Astra-H-opc-CAN-Gauge
// to be test: https://github.com/sepp89117/Opel_Astra_H_opc_CAN-GaugeV2
// LIST OF PIDs AND VARIABLE DECLARATION /////////////////////////////////////


#include <Wire.h> 
#include <LiquidCrystal_I2C.h>


#define PID_REQUEST    0x7DF
#define PID_REPLY   0x7E8

#define CAN_ID_PID 0x7DF //0x7E0 
#define PID_ENGINE_LOAD 0x04
#define PID_COOLANT_TEMP 0x05 //0x11 // 0x05
#define PID_AMBIENT_TEMP 0x46 //69 ?
#define PID_ENGINE_RPM  0x0C // 0x10 // 0x0C
#define PID_VEHICLE_SPEED 0x0D
#define PID_THROTTLE 0x14 //0x11
#define PID_VBATT  0x42 // 66 ?
 
#include <mcp_can.h>  // https://github.com/coryjfowler/MCP_CAN_lib
#include <SPI.h>
#define CAN0_INT 2
MCP_CAN CAN0(10);
long unsigned int ID;
unsigned char len = 0;
unsigned char buf[8];
char msgString[128];
int delay1 = 5;

int temperatura = 0;
int temperatura2 = 0;
float baterie = 0;
int turatie = 0;

byte grad[8] = {
  B01110,
  B10001,
  B10001,
  B01110,
  B00000,
  B00000,
  B00000,
};
 
// SEND PID //////////////////////////////////////////////////////////////////
void sendPID(unsigned char __pid) {
  unsigned char tmp[8] = {0x02, 0x01, __pid, 0, 0, 0, 0, 0};
  byte sndStat = CAN0.sendMsgBuf(CAN_ID_PID, 0, 8, tmp);
}
 
 
// RECEIVE PID ///////////////////////////////////////////////////////////////
void receivePID(unsigned char __pid) {
  if (!digitalRead(CAN0_INT)) {
    CAN0.readMsgBuf(&ID, &len, buf);
 
    switch (__pid) {
      case PID_COOLANT_TEMP:
        if (buf[2] == PID_COOLANT_TEMP) {
          uint8_t temp = buf[3] - 40;
          temperatura = temp;
          Serial.print("Temperatura = ");
          Serial.println(temp, DEC);
        }
        break;
        
       case PID_AMBIENT_TEMP:
        if (buf[2] == PID_AMBIENT_TEMP) {
          uint8_t temp2 = buf[3] - 40;
          temperatura2 = temp2;
          Serial.print("Temperatura AFARA = ");
          Serial.println(temp2, DEC);
        }
        break;

        case PID_ENGINE_RPM:
        if (buf[2] == PID_ENGINE_RPM) {
          uint16_t rpm = ((256 * buf[3]) + buf[4]) / 4.;  // (256*A+B)/4
          turatie = rpm;
          Serial.print("RPM = ");
          Serial.println(rpm, DEC);
        }
        break;

 
      case PID_VBATT:
        if (buf[2] == PID_VBATT) {
          uint16_t vbatt = ((256 * buf[3]) + buf[4]) / 1000.;  // (256*A+B)/1000
          baterie = vbatt;
          Serial.print("Tens. baterie = ");
          Serial.print(vbatt, DEC);
        }
        break;

      case PID_THROTTLE:
        if (buf[2] == PID_THROTTLE) {
          uint8_t throttle = (buf[3] * 100) / 255;
          Serial.print(throttle, DEC);
        }
        break;
 
      case PID_VEHICLE_SPEED:
        if (buf[2] == PID_VEHICLE_SPEED) {
          uint8_t vehspeed = buf[3];
          Serial.print(vehspeed, DEC);
        }
        break;
 
      case PID_ENGINE_LOAD:
        if (buf[2] == PID_ENGINE_LOAD) {
          uint8_t load = (buf[3] * 100) / 255;
          Serial.print(load, DEC);
        }
        break;
    }
  }
}
 

// Set the LCD address to 0x3F (0x27) for a 16 chars and 2 line display
LiquidCrystal_I2C lcd(0x3F, 16, 2);


 
// SETUP /////////////////////////////////////////////////////////////////////
void setup() {
  Serial.begin(115200);
  Serial.println("...");
  Serial.println("CANniq");

// initialize the LCD
  //lcd.begin();
   lcd.init(); // for other :D
  lcd.createChar(0, grad);
  // Turn on the blacklight and print a message.
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0);
  lcd.print(" Astra H - CAN  ");
  lcd.setCursor(0,1);
  lcd.print("   by niq_ro    ");
  delay(3000);
  lcd.clear();
  
  if (CAN0.begin(MCP_STDEXT, CAN_500KBPS, MCP_8MHZ) == CAN_OK) {
    Serial.println("MCP2515 initialized successfully!");
    lcd.setCursor(0,0);
    lcd.print("HW internal = OK");
    delay(1000);
    lcd.clear();
  }
  else {
    Serial.println("Error Initializing MCP2515...");
    lcd.print("HW internal =nOK");
    delay(1000);
    lcd.clear();
    while (1);
  }
 
  CAN0.init_Mask(0, 0, 0x07000000);
  CAN0.init_Mask(1, 0, 0x07000000);
  CAN0.init_Filt(0, 0, 0x07000000);
  CAN0.init_Filt(1, 0, 0x07000000);
  CAN0.init_Filt(2, 0, 0x07000000);
  CAN0.init_Filt(3, 0, 0x07000000);
  CAN0.init_Filt(4, 0, 0x07000000);
  CAN0.init_Filt(5, 0, 0x07000000);
 
  CAN0.setMode(MCP_NORMAL);
  pinMode(CAN0_INT, INPUT);
  Serial.println("Sending and receiving OBD-II PIDs...");
  lcd.clear();
}
 
 
// LOOP //////////////////////////////////////////////////////////////////////
void loop() {
  
  Serial.print(millis());
  Serial.print(",");
  
  sendPID(PID_COOLANT_TEMP);
  delay(delay1);
  receivePID(PID_COOLANT_TEMP);
  Serial.print(",");
  
  sendPID(PID_ENGINE_RPM);
  delay(delay1);
  receivePID(PID_ENGINE_RPM);
  Serial.print(",");
  
  sendPID(PID_THROTTLE);
  delay(delay1);
  receivePID(PID_THROTTLE);
  Serial.print(",");
  
  sendPID(PID_VBATT);
  delay(delay1);
  receivePID(PID_VBATT);
  Serial.print(",");
  
  sendPID(PID_AMBIENT_TEMP);
  delay(delay1);
  receivePID(PID_AMBIENT_TEMP);
  Serial.println();
  delay(delay1);


lcd.setCursor(0,0);
lcd.print("Coolant  RPM");
lcd.setCursor(1,1);
if (temperatura < 100) lcd.print(" ");
if (temperatura < 10) lcd.print(" ");
lcd.print(temperatura);
  lcd.write(byte(0));
lcd.print("C");

lcd.setCursor(9,1);
if (turatie < 1000) lcd.print(" ");
if (turatie < 100) lcd.print(" ");
if (turatie < 10) lcd.print(" ");
lcd.print(turatie);

} // end main loop
