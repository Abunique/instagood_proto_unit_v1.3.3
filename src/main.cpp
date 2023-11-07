// 3.5 DISPLAY, GPIO25 as output pin for display DC pin
// APN 2 setting added
// IR sensor 2 added, GPIO25 as input for ir receiver

// V1.3.2
//  turn off display if no stock is available
//  mqtt subscribe qos change from 0 to 1
//  check sim status and gprs setting : if no sim available or gprs not connected
//    --disable simcom loop, mqtt logic & loop
//

#include <Arduino.h>
//#include <Servo.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

#include <SPI.h>
#include <TFT_eSPI.h> // Hardware-specific library
// #include <006_qrcode.h>
// #include <007_qrcode.h>
// #include <008_qrcode.h>
// #include <009_qrcode.h>

 #define BOARD_ID_vj_148 148
 #include <qr-code.h>

#define TINY_GSM_MODEM_SIM800
#define SerialMon Serial
#define SerialAT Serial1
#define TINY_GSM_DEBUG SerialMon
#define GSM_PIN ""
// TTGO T-Call pins
#define MODEM_RST 32
#define MODEM_PWRKEY 13
// #define MODEM_POWER_ON -1
#define MODEM_TX 27
#define MODEM_RX 26
// #define MODEM_RI 35
////////////////////////////////////////

#define COIN_ACCEPTOR_INPUT 39
#define NOTE_ACCEPTOR_INPUT 12
#define COIN_IN_ENABLE_OUTPUT 4
// #define DISPENSE_IR_INPUT2 25
#define DISPENSE_IR_INPUT1 34

#define MOTOR_FORWARD_PIN 14
#define MOTOR_REVERSE_PIN 5

#define USR_RST_SWITCH_PIN 35
#define NFC_BUZZER 2

#define Manual_Pin 36

#define SERVO_PIN 33

#define I2C_SDA 21
#define I2C_SCL 22

#define stock_addr 0

const char apn1[] = "airtelgprs.com";
const char apn2[] = "iot.com";
const char gprsUser[] = "";
const char gprsPass[] = "";
const char simPIN[] = "";

#include <ArduinoJson.h>
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <PN532_I2C.h>
#include <PN532.h>
#include <NfcAdapter.h>

TinyGsm modem(SerialAT);
TinyGsmClient client(modem);
PubSubClient mqtt(client);
PN532_I2C pn532i2c(Wire);
PN532 nfc(pn532i2c);
//LiquidCrystal_I2C lcd(0x3F, 16, 2);
LiquidCrystal_I2C lcd(0x27, 16, 2);
// Servo myservo;
TFT_eSPI tft = TFT_eSPI(); // Invoke custom library

#define TFT_GREY 0x5AEB // New colour

uint32_t lastReconnectAttempt = 0;
long lastMsg = 0;

boolean success;
uint8_t uid[] = { 0, 0, 0, 0, 0, 0, 0 };  // Buffer to store the returned UID
uint8_t uidLength;                        // Length of the UID (4 or 7 bytes depending on ISO14443A card type)
  
byte readCard[7];
char UID[50];
// int a;
int count = 0;

int coin_impulsCount = 0;
int note_impulsCount = 0;
bool dispense_product = false;

bool check_flag = 0;
volatile int state = 2;
volatile int state2 = 2;
bool stock_reset = false;

bool gsm_connected = false;
bool gprs_connected = false;
bool mqtt_connected = false;

bool run_timer = false;
unsigned long previousMillis = 0;
unsigned long mqtt_previousMillis = 0;
const long interval = 6000;

bool coin_mqtt_flag = false;
bool upi_mqtt_flag = false;
bool note_mqtt_flag = false;
bool nfc_mqtt_flag = false;

////////////////////////////////////
String COMMAND;
String DEVICE_ID;
String DATE;
String DISPATCH;
String CAPACITY;
String PAYMENT;
String STOCK;
int idset;
////////////////////////////////////////

void IRAM_ATTR COIN_ACCEPTOR_INT1()
{
  coin_impulsCount = coin_impulsCount + 1;
}

void IRAM_ATTR NOTE_ACCEPTOR_INT1()
{
  note_impulsCount = note_impulsCount + 1;
}

void IRAM_ATTR DISPENSE_IR_INT1()
{
  // check_flag = true;
  state = 0;
}

void IRAM_ATTR USR_RST_INT1()
{
  // check_flag = true;
  stock_reset = true;
}

////////////////////////////////////////////////////////////////

//////////////////////////EEPROM////////////////////////////////////////////////////////////

template <class T>
int EEPROM_writeAnything(int ee, const T &value)
{
  const byte *p = (const byte *)(const void *)&value;
  unsigned int i;
  for (i = 0; i < sizeof(value); i++)
    EEPROM.write(ee++, *p++);
  return i;
}

template <class T>
int EEPROM_readAnything(int ee, T &value)
{
  byte *p = (byte *)(void *)&value;
  unsigned int i;
  for (i = 0; i < sizeof(value); i++)
    *p++ = EEPROM.read(ee++);
  return i;
}

////////////////////////////////////////////////////////////////////////////////////////////////

void mqttCallback(char *topic, byte *message, unsigned int len)
{
  Serial.print("Message arrived on topic: ");
  Serial.print(topic);
  Serial.print("Message: ");
  String messageTemp;

  for (int i = 0; i < len; i++)
  {
    Serial.print((char)message[i]);
    messageTemp += (char)message[i];
  }
  Serial.println();

  StaticJsonBuffer<300> jsonBuffer; // Memory pool
  JsonObject &root = jsonBuffer.parseObject(message);
  if (!root.success())
  { // Check for errors in parsing
    Serial.println("Parsing failed");
    delay(1000);
    return;
  }
  const char *command = root["command"];
  const char *device_id = root["device_id"];
  const char *date = root["date"];
  const char *dispatch = root["dispatch"];
  const char *capacity = root["capacity"];
  const char *payment = root["payment"];
  const char *stock = root["stock"];

  COMMAND = command;
  DEVICE_ID = device_id;
  DATE = date;
  DISPATCH = dispatch;
  CAPACITY = capacity;
  PAYMENT = payment;
  STOCK = stock;
}

unsigned long stock_previousMillis = 0;
bool first_stock_empty = true;

void setup3_5inchDisplay()
{
  tft.init();

  if (mqtt_connected)
  {
    tft.setRotation(3);
    tft.fillScreen(TFT_BLACK);
    tft.drawBitmap(80, 0, qrcode, 320, 480, TFT_WHITE);
  }
  else
  {
    tft.setRotation(3);
    tft.fillScreen(TFT_BLACK);
    tft.setCursor(0, 0, 2);
    tft.setTextColor(TFT_WHITE, TFT_BLACK);
    tft.setTextSize(1);
    tft.println("Server Disconnected!");
  }

  // pinMode(DISPENSE_IR_INPUT2, INPUT_PULLUP);
  //  Serial.println("Pin set to input");
  // delay(1000);
}

boolean mqttConnect()
{
  SerialMon.print("Connecting to ");
  SerialMon.print(broker);
  boolean status = mqtt.connect(cid, mqttUsername, mqttPassword);

  if (status == false)
  {
    SerialMon.println(" fail");
    // ESP.restart();
    mqtt_connected = false;
    setup3_5inchDisplay();
    return false;
  }
  SerialMon.println(" success");
  mqtt_connected = true;
  setup3_5inchDisplay();
  mqtt.subscribe(topic2, 1);
  return mqtt.connected();
}

void stock_empty()
{
  String time = modem.getGSMDateTime(DATE_FULL);
  StaticJsonBuffer<300> JSONbuffer;
  JsonObject &JSONencoder = JSONbuffer.createObject();
  JSONencoder["command"] = "reload";
  JSONencoder["device_id"] = ID;
  JSONencoder["Date"] = time;
  JSONencoder["product_available"] = "0";
  char JSONmessageBuffer[200];
  JSONencoder.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
  Serial.println("Sending message to MQTT topic..");
  Serial.println(JSONmessageBuffer);
  mqtt.publish(topic1, JSONmessageBuffer);
}

void send_error_message()
{
  String time = modem.getGSMDateTime(DATE_FULL);
  StaticJsonBuffer<300> JSONbuffer;
  JsonObject &JSONencoder = JSONbuffer.createObject();
  JSONencoder["command"] = "error";
  JSONencoder["device_id"] = ID;
  JSONencoder["Date"] = time;
  // JSONencoder["product_available"] = "0";
  char JSONmessageBuffer[200];
  JSONencoder.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
  Serial.println("Sending message to MQTT topic..");
  Serial.println(JSONmessageBuffer);
  mqtt.publish(topic1, JSONmessageBuffer);
}

void send_coin_note_UPI_ackMessage(int input)
{
  String time = modem.getGSMDateTime(DATE_FULL);
  StaticJsonBuffer<300> JSONbuffer;
  JsonObject &JSONencoder = JSONbuffer.createObject();

  if (input == 1)
    JSONencoder["command"] = "coinIn";
  if (input == 2)
    JSONencoder["command"] = "noteIn";
  if (input == 3)
    JSONencoder["command"] = "UPI";

  JSONencoder["device_id"] = ID;
  JSONencoder["Date"] = time;
  // JSONencoder["product_available"] = "0";
  char JSONmessageBuffer[200];
  JSONencoder.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
  Serial.println("Sending message to MQTT topic..");
  Serial.println(JSONmessageBuffer);
  mqtt.publish(topic1, JSONmessageBuffer);
}

void send_emptyStock()
{
  unsigned long stock_currentMillis = millis();

  if ((stock_currentMillis - stock_previousMillis >= 300000) || (first_stock_empty == true))
  {
    // save the last time you blinked the LED
    stock_previousMillis = stock_currentMillis;
    first_stock_empty = false;
    stock_empty();
  }
}

void setupModem()
{
#ifdef MODEM_RST
  // Keep reset high
  pinMode(MODEM_RST, OUTPUT);
  digitalWrite(MODEM_RST, HIGH);
#endif

  pinMode(MODEM_PWRKEY, OUTPUT);
  // pinMode(MODEM_POWER_ON, OUTPUT);

  // Turn on the Modem power first
  // digitalWrite(MODEM_POWER_ON, HIGH);

  // Pull down PWRKEY for more than 1 second according to manual requirements
  digitalWrite(MODEM_PWRKEY, HIGH);
  delay(100);
  digitalWrite(MODEM_PWRKEY, LOW);
  delay(1000);
  digitalWrite(MODEM_PWRKEY, HIGH);
}

void NFC_DETAILS()
{
  String time = modem.getGSMDateTime(DATE_FULL);
  StaticJsonBuffer<300> JSONbuffer;
  JsonObject &JSONencoder = JSONbuffer.createObject();
  JSONencoder["command"] = "nfc";
  JSONencoder["device_id"] = ID;
  JSONencoder["Date"] = time;
  JSONencoder["data"] = UID;
  JSONencoder["product_available"] = count;
  char JSONmessageBuffer[200];
  JSONencoder.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
  Serial.println("Sending message to MQTT topic..");
  Serial.println(JSONmessageBuffer);
  mqtt.publish(topic1, JSONmessageBuffer);
}

void NFC_mqtt()
{
  String time = modem.getGSMDateTime(DATE_FULL);
  StaticJsonBuffer<300> JSONbuffer;
  JsonObject &JSONencoder = JSONbuffer.createObject();
  JSONencoder["command"] = "nfc_sack";
  JSONencoder["device_id"] = ID;
  JSONencoder["Date"] = time;
  JSONencoder["dispatch"] = "true";
  JSONencoder["product_available"] = count;
  char JSONmessageBuffer[200];
  JSONencoder.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
  Serial.println("Sending message to MQTT topic..");
  Serial.println(JSONmessageBuffer);
  bool ack = mqtt.publish(topic1, JSONmessageBuffer);

  Serial.println(ack);
  // mqtt.publish(topic1, JSONmessageBuffer);
}

void coinmachine_mqtt()
{
  String time = modem.getGSMDateTime(DATE_FULL);
  StaticJsonBuffer<300> JSONbuffer;
  JsonObject &JSONencoder = JSONbuffer.createObject();
  if (coin_mqtt_flag == true)
    JSONencoder["command"] = "coin";
  if (note_mqtt_flag == true)
    JSONencoder["command"] = "note";

  JSONencoder["device_id"] = ID;
  JSONencoder["Date"] = time;
  JSONencoder["product_available"] = count;
  char JSONmessageBuffer[200];
  JSONencoder.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
  Serial.println("Sending message to MQTT topic..");
  Serial.println(JSONmessageBuffer);

  mqtt.publish(topic1, JSONmessageBuffer);
}

void upi_mqtt()
{
  String time = modem.getGSMDateTime(DATE_FULL);
  StaticJsonBuffer<300> JSONbuffer;
  JsonObject &JSONencoder = JSONbuffer.createObject();
  JSONencoder["command"] = "UPI";
  JSONencoder["device_id"] = ID;
  JSONencoder["Date"] = time;
  JSONencoder["Dispatch"] = "true";
  JSONencoder["product_available"] = count;
  char JSONmessageBuffer[200];
  JSONencoder.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
  Serial.println("Sending message to MQTT topic..");
  Serial.println(JSONmessageBuffer);

  mqtt.publish(topic1, JSONmessageBuffer);
}

void getid()
{
  String time = modem.getGSMDateTime(DATE_FULL);
  StaticJsonBuffer<200> JSONbuffer;
  JsonObject &JSONencoder = JSONbuffer.createObject();
  JSONencoder["command"] = "getid";
  JSONencoder["Date"] = time;
  char JSONmessageBuffer[100];
  JSONencoder.printTo(JSONmessageBuffer, sizeof(JSONmessageBuffer));
  Serial.println("Sending message to MQTT topic..");
  Serial.println(JSONmessageBuffer);
  bool ack = mqtt.publish(topic1, JSONmessageBuffer);
  // mqtt.publish(topic1, JSONmessageBuffer);
  Serial.println(ack);
}

void array_to_string(byte a[], unsigned int len, char buffer[])
{
  for (unsigned int i = 0; i < len; i++)
  {
    byte nib1 = (a[i] >> 4) & 0x0F;
    byte nib2 = (a[i] >> 0) & 0x0F;
    buffer[i * 2 + 0] = nib1 < 0x0A ? '0' + nib1 : 'A' + nib1 - 0x0A;
    buffer[i * 2 + 1] = nib2 < 0x0A ? '0' + nib2 : 'A' + nib2 - 0x0A;
  }
  buffer[len * 2] = '\0';
}
bool Pin_status= LOW;
void deliver()
{
  /*
    put dispatch code here
  */
 
  digitalWrite(COIN_IN_ENABLE_OUTPUT, HIGH);
  delay(100);
  Serial.println("dispatch starts");

  // open shutter
  // myservo.attach(SERVO_PIN);
  // myservo.write(95);
  // delay(1000);
  // myservo.detach();
  dispense_product = true;
  check_flag = false;

  // turn on motor and enable timer
  if ((dispense_product == true) && (check_flag == false) && (run_timer == false))
  {
    digitalWrite(MOTOR_FORWARD_PIN, HIGH);
    digitalWrite(MOTOR_REVERSE_PIN, LOW);

    // enable ir sensor interrupt
    Serial.printf("\n%d %d %d", dispense_product, check_flag, run_timer);

    // run timer for 4 seconds, if check_flag == true turn of motor.
    run_timer = true;
    previousMillis = millis();
attachInterrupt(DISPENSE_IR_INPUT1, DISPENSE_IR_INT1, RISING);
    // delay(100);
  }
}


void setupNFC()
{
  ///////////////////NFC/////////////////
  nfc.begin();
  uint32_t versiondata = nfc.getFirmwareVersion();
  if (!versiondata)
  {
    Serial.println("PN53x card not found!");
    //while(1);//halt
  }
  else
  {
     // Got ok data, print it out!
  Serial.print("Found chip PN5"); Serial.println((versiondata>>24) & 0xFF, HEX); 
  Serial.print("Firmware ver. "); Serial.print((versiondata>>16) & 0xFF, DEC); 
  Serial.print('.'); Serial.println((versiondata>>8) & 0xFF, DEC);
    // Set the max number of retry attempts to read from a card
  // This prevents us from waiting forever for a card, which is
  // the default behaviour of the PN532.
  nfc.setPassiveActivationRetries(0x01);
    // configure board to read RFID tags
  nfc.SAMConfig();
    Serial.println("Waiting for card (ISO14443A Mifare)...");
    Serial.println("");
  }
}


void setup()
{

  pinMode(NFC_BUZZER,OUTPUT);
  digitalWrite(NFC_BUZZER,LOW);

  pinMode(MOTOR_FORWARD_PIN, OUTPUT);
  pinMode(MOTOR_REVERSE_PIN, OUTPUT);

  digitalWrite(MOTOR_FORWARD_PIN, LOW);
  digitalWrite(MOTOR_REVERSE_PIN, LOW);

  pinMode(COIN_ACCEPTOR_INPUT, INPUT);
  pinMode(NOTE_ACCEPTOR_INPUT, INPUT);

  pinMode(DISPENSE_IR_INPUT1, INPUT);

  pinMode(Manual_Pin, INPUT);

  pinMode(USR_RST_SWITCH_PIN, INPUT);

  pinMode(COIN_IN_ENABLE_OUTPUT, OUTPUT);
  digitalWrite(COIN_IN_ENABLE_OUTPUT, HIGH);


  SerialMon.begin(115200);
  // Serial.println(digitalRead(DISPENSE_IR_INPUT2));
  // delay(500);
  Serial.println(BOARD_ID_vj_148);
  Serial.println("Firmware : v1.3.2");
  setupNFC();

  // myservo.attach(SERVO_PIN);
  // myservo.write(0); // shutter close
  // delay(1000);
  // myservo.detach();

  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Instagood");
  lcd.setCursor(0, 1);
  lcd.print("Initializing..");

  delay(1000);

  // setup3_5inchDisplay();

  if (!EEPROM.begin(4))
  {
    Serial.println("failed to initialise EEPROM");
    delay(1000);
  }

  setupModem();

  SerialMon.println("Wait...");
  // Set GSM module baud rate and UART pins
  SerialAT.begin(115200, SERIAL_8N1, MODEM_RX, MODEM_TX);
  delay(6000);

  // Restart takes quite some time
  // To skip it, call init() instead of restart()
  SerialMon.println("Initializing modem...");
  modem.restart();
  // modem.init();

  String modemInfo = modem.getModemInfo();
  SerialMon.print("Modem Info: ");
  SerialMon.println(modemInfo);

  if (modem.getSimStatus() == 1)
  {
    // Unlock your SIM card with a PIN if needed
    if (GSM_PIN && modem.getSimStatus() != 3)
    {
      modem.simUnlock(GSM_PIN);
    }

    gsm_connected = true;

    SerialMon.print("Connecting to APN1: ");
    SerialMon.print(apn1);

    if (!modem.gprsConnect(apn1, gprsUser, gprsPass))
    {
      SerialMon.println("APN1 fail");
      // ESP.restart();
      SerialMon.print("Connecting to APN2: ");
      SerialMon.print(apn2);
      if (!modem.gprsConnect(apn2, gprsUser, gprsPass))
      {
        SerialMon.println("APN2 fail");
      }
      else
      {
        SerialMon.println("APN2 OK");
      }
    }
    else
    {
      SerialMon.println("APN1 OK");
    }

    if (modem.isGprsConnected())
    {
      SerialMon.println("GPRS connected");
      gprs_connected = true;
    }
    // MQTT Broker setup
    mqtt.setServer(broker, 1883);
    mqtt.setCallback(mqttCallback);
    mqttConnect();
  }

  //////////////////////////////////////////////
  // uncomment below code to get machine id and stock
  ///////////////////////////////////

  //  while (!mqtt.connected()) {
  //    SerialMon.println("=== MQTT NOT CONNECTED ===");
  //    uint32_t t = millis();
  //    if (t - lastReconnectAttempt > 10000L) {
  //      lastReconnectAttempt = t;
  //      if (mqttConnect()) {
  //        lastReconnectAttempt = 0;
  //      }
  //    }
  //    delay(1000);
  //  }
  //  getid();
  //
  //  do {
  //    mqtt.loop();
  //    idset = 5;
  //    Serial.println("receive");
  //    if ((COMMAND == "setid"))
  //    {
  //      ID = DEVICE_ID;
  //      count = CAPACITY.toInt();
  //      Serial.println(ID);
  //      Serial.println(count);
  //
  //      idset = 2;
  //    }
  //  } while (idset > 3);
  //  delay(5000);

  ///////////////////////////////////////////////
  //
  // ID = "tng_b05b13ee_f659_4542_bd85_19bf0f63cfff";
  // count = 500;

  digitalWrite(COIN_IN_ENABLE_OUTPUT, LOW);
  delay(100);
  attachInterrupt(COIN_ACCEPTOR_INPUT, COIN_ACCEPTOR_INT1, FALLING);
  attachInterrupt(NOTE_ACCEPTOR_INPUT, NOTE_ACCEPTOR_INT1, FALLING);

  attachInterrupt(USR_RST_SWITCH_PIN, USR_RST_INT1, FALLING);

  Serial.printf("\n%d %d %d", dispense_product, check_flag, run_timer);

  // myservo.attach(SERVO_PIN);
  // myservo.write(57); // shutter close
  // delay(1000);
  // myservo.detach();

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Instagood");
  lcd.setCursor(0, 1);
  lcd.print("In Stock:");
  EEPROM_readAnything(0, count);
  lcd.setCursor(9, 1);
  lcd.print(count);
   Serial.println(count);
   
}

bool stock_reload = true;


void loop()
{
  Pin_status = digitalRead(Manual_Pin);
 while(Pin_status){
digitalWrite(COIN_IN_ENABLE_OUTPUT, HIGH);
Serial.println("Manual interrupt");
Pin_status = digitalRead(Manual_Pin);
if(!Pin_status){
  digitalWrite(COIN_IN_ENABLE_OUTPUT, LOW);
  coin_impulsCount = 0;
  note_impulsCount = 0;
}
delay(1000);
}
  if (dispense_product == false)
  {
    if (gprs_connected == true)
    {
      if (!mqtt.connected())
      {
        // if (millis() - mqtt_previousMillis > 60000L)
        //{
        uint32_t t = millis();
        if (t - lastReconnectAttempt > 60000L)
        {
          SerialMon.println("=== MQTT NOT CONNECTED ===");
          lastReconnectAttempt = t;
          // mqtt_previousMillis = millis();
          if (mqttConnect())
          {
            lastReconnectAttempt = 0;
          }
        }
      }
      else
        mqtt.loop();
    }

    int proximity_value = digitalRead(DISPENSE_IR_INPUT1);

    if (stock_reset == true)
    {
      stock_reset = false;
      count = 500;
      EEPROM_writeAnything(0, count);
      EEPROM.commit();
      delay(100);
      EEPROM_readAnything(stock_addr, count);
      Serial.println(count);
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Instagood");
      lcd.setCursor(0, 1);
      lcd.print("In Stock:");
      lcd.setCursor(9, 1);
      lcd.print(count);
    }
    ///////////////////////////////////////////////////////////////////////////
    if (proximity_value == HIGH)
    {
      digitalWrite(25, LOW);
      if (stock_reload == false)
      {
        if (digitalRead(COIN_IN_ENABLE_OUTPUT) == HIGH)
          digitalWrite(COIN_IN_ENABLE_OUTPUT, LOW);
        // delay(100);
        attachInterrupt(COIN_ACCEPTOR_INPUT, COIN_ACCEPTOR_INT1, FALLING);
        attachInterrupt(NOTE_ACCEPTOR_INPUT, NOTE_ACCEPTOR_INT1, FALLING);
        stock_reload = true;
        first_stock_empty = true;
        EEPROM_readAnything(stock_addr, count);
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print("Instagood");
        lcd.setCursor(0, 1);
        lcd.print("In Stock:");
        lcd.setCursor(9, 1);
        lcd.print(count);
        setup3_5inchDisplay();
        delay(500);
        digitalWrite(25, LOW);
      }

      // int coin_value = digitalRead(coin);
      // Serial.println("in");
      // success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, &uid[0], &uidLength);
      // // Serial.println("out");

      // if (success) // NFC
      // {
      //   Serial.println("buzzer");
      //   for (uint8_t i = 0; i < uidLength; i++) // NFC
      //   {
      //     readCard[i] = uid[i];
      //   }
      //   UID[0] = '\0';
      //   array_to_string(readCard, 4, UID);

      //   delay(5000);
      //   NFC_DETAILS();
      // }

      // // UPI ACK
      success = nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, &uid[0], &uidLength);
  
  if (success) {
    Serial.println("Found a card!");
    Serial.print("UID Length: ");Serial.print(uidLength, DEC);Serial.println(" bytes");
    Serial.print("UID Value: ");

    for (uint8_t i=0; i < uidLength; i++) 
    {
      Serial.print(" 0x");Serial.print(uid[i], HEX); //serial monitor printing
      readCard[i] = uid[i];
    }
    Serial.println("");
    array_to_string(readCard, 4, UID);
    // Wait 1 second before continuing
   delay(5000);
   UID[0] = '\0';
   NFC_DETAILS();
    coin_impulsCount = 0;
    note_impulsCount = 0;
    
    digitalWrite(NFC_BUZZER,HIGH);
     delay(500);
     digitalWrite(NFC_BUZZER,LOW);
  }
 
 Serial.print("\nchecking for coin /note");
      if ((COMMAND == "ack") && (DEVICE_ID == ID) && (PAYMENT == "success") && ((DISPATCH == "true")))
      {
        send_coin_note_UPI_ackMessage(3);
        deliver();
        // delay(1000);
        upi_mqtt_flag = true;
        // delay(1000);
        DISPATCH = "false";
        attachInterrupt(digitalPinToInterrupt(DISPENSE_IR_INPUT1), DISPENSE_IR_INT1, RISING);
      }
      // NFC ACK
      else if ((COMMAND == "nfc_sack") && (DEVICE_ID == ID) && ((DISPATCH == "true")))
      {
        deliver();
        // delay(1000);
        nfc_mqtt_flag = true;
        // delay(1000);
        DISPATCH = "false";
        attachInterrupt(digitalPinToInterrupt(DISPENSE_IR_INPUT1), DISPENSE_IR_INT1, RISING);
      }

      if (coin_impulsCount >= 4)
      {
        detachInterrupt(digitalPinToInterrupt(NOTE_ACCEPTOR_INPUT));
        delay(500);
        coin_impulsCount = 0;

        if ((gprs_connected == true) && (mqtt_connected == true))
        {
          send_coin_note_UPI_ackMessage(1);
          coin_mqtt_flag = true;
        }

        deliver();
        ////delay(5000);

        Serial.printf("\n%d %d %d", dispense_product, check_flag, run_timer);
        attachInterrupt(digitalPinToInterrupt(DISPENSE_IR_INPUT1), DISPENSE_IR_INT1, RISING);
      }

      // if (note_impulsCount > 0)
      //   Serial.println(note_impulsCount);

      if (note_impulsCount >= 4)
      {
        detachInterrupt(digitalPinToInterrupt(COIN_ACCEPTOR_INPUT));
        delay(500);
        note_impulsCount = 0;
        if ((gprs_connected == true) && (mqtt_connected == true))
        {
          send_coin_note_UPI_ackMessage(2);
          note_mqtt_flag = true;
        }
        // disable coin acceptor interrupt
        // Serial.println(note_impulsCount);

        deliver();
        /////delay(5000);

        Serial.printf("\n%d %d %d", dispense_product, check_flag, run_timer);
        attachInterrupt(digitalPinToInterrupt(DISPENSE_IR_INPUT1), DISPENSE_IR_INT1, RISING);
      }
      
    }
    else
    {
      Serial.println("stock empty");
      detachInterrupt(digitalPinToInterrupt(COIN_ACCEPTOR_INPUT));
      detachInterrupt(digitalPinToInterrupt(NOTE_ACCEPTOR_INPUT));
      digitalWrite(COIN_IN_ENABLE_OUTPUT, HIGH);
      stock_reload = false;
      delay(100);
      lcd.clear();
      lcd.setCursor(0, 1);
      lcd.print("Stock Empty");
      if ((gprs_connected == true) && (mqtt_connected == true))
        send_emptyStock();
      digitalWrite(25, HIGH);
      tft.fillScreen(TFT_BLACK);
      // delay(1000);
    }
  }
 
  // run timer for 4 seconds. if check_flag is not set within 4 seconds turn off motor.
  if (run_timer == true)
  {
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= interval)
    {
      detachInterrupt(digitalPinToInterrupt(DISPENSE_IR_INPUT1));
      // detachInterrupt(digitalPinToInterrupt(DISPENSE_IR_INPUT2));
      delay(500);
      previousMillis = currentMillis;

      digitalWrite(MOTOR_FORWARD_PIN, LOW);
      dispense_product = false;
      check_flag = false;
      run_timer = false;
      state = 2;
      // state2 = 2;
      Serial.print("\nDispense stopped by timer");
      // detachInterrupt(digitalPinToInterrupt(DISPENSE_IR_INPUT));
      // myservo.attach(SERVO_PIN);
      // myservo.write(0); // shutter close
      // delay(1000);
      // myservo.detach();
      digitalWrite(COIN_IN_ENABLE_OUTPUT, LOW);
      delay(1000);
      note_impulsCount = 0;
      attachInterrupt(COIN_ACCEPTOR_INPUT, COIN_ACCEPTOR_INT1, FALLING);
      attachInterrupt(NOTE_ACCEPTOR_INPUT, NOTE_ACCEPTOR_INT1, FALLING);

      Serial.printf("\n%d %d %d", dispense_product, check_flag, run_timer);
      // digitalWrite(COIN_IN_ENABLE_OUTPUT, HIGH);
      if ((gprs_connected == true) && (mqtt_connected == true))
        send_error_message();

      coin_mqtt_flag = false;
      note_mqtt_flag = false;
      nfc_mqtt_flag = false;
      upi_mqtt_flag = false;
    }
  }

  // Serial.printf("\ncheck_flag:%d\n",check_flag);
  // turn off motor if ir sensor interrupt is triggered and check_flag i true
  if (state == 0)
  {
    if ((dispense_product == true))
    {
      detachInterrupt(digitalPinToInterrupt(DISPENSE_IR_INPUT1));
      //delay(500);
      Serial.print("\nDispense stopped by IR1");

      delay(1300); // delay(1000)
      digitalWrite(MOTOR_FORWARD_PIN, LOW);

      check_flag = false;
      run_timer = false;
      state = 2;
      // state2 = 2;
      delay(1000); // delay(5000)
      // myservo.attach(SERVO_PIN);
      // myservo.write(0); // shutter close
      // delay(1000);
      // myservo.detach();

      if (count > 0)
        count--;

      Serial.printf("\ncount: %d\n", count);

      EEPROM_writeAnything(stock_addr, count);
      EEPROM.commit();
      delay(100);

      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Instagood");
      lcd.setCursor(0, 1);
      lcd.print("In Stock:");
      lcd.setCursor(9, 1);
      lcd.print(count);

      //      if(count == 0)
      //      count

      // if (!mqtt.connected())
      // {
      //   SerialMon.println("=== MQTT NOT CONNECTED ===");
      //   uint32_t t = millis();
      //   if (t - lastReconnectAttempt > 10000L)
      //   {
      //     lastReconnectAttempt = t;
      //     if (mqttConnect())
      //     {
      //       lastReconnectAttempt = 0;
      //     }
      //   }
      //   delay(1000);
      //   return;
      // }

      // Serial.println(coin_mqtt_flag);
      // Serial.println(note_mqtt_flag);
      // Serial.println(nfc_mqtt_flag);
      // Serial.println(upi_mqtt_flag);

      if ((gprs_connected == true) && (mqtt_connected == true))
      {
        if ((coin_mqtt_flag == true) || (note_mqtt_flag == true))
        {
          // Serial.println("Coin MQTT start");
          coinmachine_mqtt();
          // Serial.println("Coin MQTT End");
          coin_mqtt_flag = false;
          note_mqtt_flag = false;
        }
        else if (nfc_mqtt_flag == true)
        {
          NFC_mqtt();
          nfc_mqtt_flag = false;
        }
        else if (upi_mqtt_flag == true)
        {
          upi_mqtt();
          upi_mqtt_flag = false;
        }
      }
      digitalWrite(COIN_IN_ENABLE_OUTPUT, LOW);
      delay(100);
      note_impulsCount = 0;
      attachInterrupt(COIN_ACCEPTOR_INPUT, COIN_ACCEPTOR_INT1, FALLING);
      // delay(500);
      attachInterrupt(NOTE_ACCEPTOR_INPUT, NOTE_ACCEPTOR_INT1, FALLING);

      dispense_product = false;
      Serial.printf("\n%d %d %d\n", dispense_product, check_flag, run_timer);
    }
  }

  ///////////////////////////////////////////////////////////

  // STOCK RELOAD
  // else if(
  //{
  //
  //}
  //////////////////////////////////////////////////////////////////
  // mqtt.loop();
}
