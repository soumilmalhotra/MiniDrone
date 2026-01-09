// #include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// === LCD ===
LiquidCrystal_I2C lcd(0x27, 16, 2); // adjust if I2C address is different

// === NRF24L01 ===
RF24 radio(9, 10); // CE, CSN
const byte address[6] = "00001";

// === Joystick Pins ===
const int joy1_x = A1;
const int joy1_y = A0;
const int joy2_x = A3;
const int joy2_y = A2;

// === Rotary Encoder Pins ===
const int clk = 3;
const int dt = 5;
const int sw = 4;

// === Encoder Variables ===
int counter = 0;
int lastClkState;
bool lastButtonState = HIGH;
unsigned long lastDebounce = 0;

// === Data Structure ===
struct DataPacket {
  int yaw;//j1x
  int throttle;//j1y
  int roll;//j2x
  int pitch;//j2y
};
  // int encoderVal;
  // bool encoderBtn;

DataPacket data;
// DataPacket data = {512, 512, 512, 512, 100, false};

void setup() {
  Serial.begin(9600);

  // LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Remote Ready");

  // Joysticks
  pinMode(joy1_x, INPUT);
  pinMode(joy1_y, INPUT);
  pinMode(joy2_x, INPUT);
  pinMode(joy2_y, INPUT);

  // Encoder
  // pinMode(clk, INPUT);
  // pinMode(dt, INPUT);
  // pinMode(sw, INPUT_PULLUP);
  // lastClkState = digitalRead(clk);

  // NRF24L01
  radio.begin();
  radio.setPALevel(RF24_PA_HIGH);
  radio.setDataRate(RF24_1MBPS); // or try RF24_250KBPS for longer range
  radio.openWritingPipe(address);
  radio.stopListening();
  Serial.println("Transmitter ready");
}

void loop() {
  // Read joystick
  data.yaw = analogRead(joy1_x);
  data.throttle = analogRead(joy1_y);
  data.roll = analogRead(joy2_x);
  data.pitch = analogRead(joy2_y);

  // Read encoder rotation
  // int clkState = digitalRead(clk);
  // if (clkState != lastClkState && clkState == LOW) {
  //   if (digitalRead(dt) != clkState) {
  //     counter++;
  //   } else {
  //     counter--;
  //   }
  // }
  // lastClkState = clkState;
  // data.encoderVal = counter;

  // Read encoder button with debounce
  // bool reading = digitalRead(sw);
  // if (reading != lastButtonState && (millis() - lastDebounce) > 50) {
  //   lastDebounce = millis();
  //   if (reading == LOW) {
  //     data.encoderBtn = !data.encoderBtn;
  //   }
  // }
  // lastButtonState = reading;

  // Send via NRF24L01
  radio.write(&data, sizeof(DataPacket));
  Serial.println("Data sent!");

  // Display on LCD
  // lcd.setCursor(0, 0);
  // lcd.print("J1: ");
  // lcd.print(data.joy1_x);
  // lcd.print(",");
  // lcd.print(data.joy1_y);

  // lcd.setCursor(0, 1);
  // lcd.print("J2: ");
  // lcd.print(data.joy2_x);
  // lcd.print(",");
  // lcd.print(data.joy2_y);


  // Serial.println(data.pitch);
  Serial.println(data.throttle);
  // Serial.println(data.roll);
  Serial.println(data.yaw);
  // delay(100);
  // lcd.clear();
  delay(5);   // ~200 Hz update rate

}
