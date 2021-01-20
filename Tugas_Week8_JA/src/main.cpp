#include <Arduino.h>
#include <WiFi.h>
#include <EEPROM.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <Wire.h>
#include <SPI.h>
#include <DHT.h>

#define DHT_PIN 19 
#define DHT_TYPE DHT11 //sensor suhu & humidity
#define PUBLISH_INTERVAL 10000

#define LED_PIN 18 //Led
#define PWM_CHANNEL 0
#define PWM_FREQ 5000
#define PWM_RESOLUTION 8
#define TOMBOL_AUTO 15 //Tombol auto brightness

#define BH1750_ADDRESS 0x23 //address sensor cahaya (lux)
#define BH1750_DATALEN 2

#define EEPROM_SIZE 96
#define SSIDPASS_ADDRESS 32
#define SSIDPASS_SIZE 64

unsigned long lastPublish = 0;
float tempFloat, humidFloat, luxFloat = 0;
String temperature;
String humidity;
DHT dht(DHT_PIN, DHT_TYPE);

String Stringlux;
unsigned short lux = 0, dutyCycle = 0;
byte buff[2];
void bh1750Request(int address);
int bh1750GetData(int address);

const char *ssid = "I3";
const char *password = "@green789";
String mqttServer = "broker.hivemq.com";
String mqttUser = "";
String mqttPwd = "";
String BuildingId = "Home_JA1";
String pubTopic = String(BuildingId + "/sensor_data");
String subTopic = String(BuildingId + "/SmartLighting");
String mqttPort = "1883";

WiFiClient ESPClient;
PubSubClient ESPMqtt(ESPClient);

bool statusAuto = false, buttonPressed = false, modeWifi = false;

char Otomatis[EEPROM_SIZE], DataSsidPass[EEPROM_SIZE];
String DataReceived, Ssid, Password;

WiFiServer server (80);

String ledState;


void BacaEEPROM();
void readEEPROM(int address, char *data, int size);
void writeEEPROM(int address, const char *data, int size);
void BacaSsidPassword();
void BacaStatusSerial(); 
void CekStatusAuto();
void BacaLux();
void BacaPerubahanTombol(); 
void AutoBrightness();
void connectToNetwork();
void connectToMqtt();
void publishMessage();
void updateDhtData();
void mqttCallback(char *topic, byte *payload, long length);
void do_actions(const char *msg);


void IRAM_ATTR gpioISR() {
  buttonPressed = true;
}

void setup() {
  Wire.begin();
  Serial.begin(9600);
  delay(1000);
  pinMode(BUILTIN_LED, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  attachInterrupt(TOMBOL_AUTO, &gpioISR, FALLING);
  
  ledcSetup(PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcAttachPin(LED_PIN, PWM_CHANNEL);

  dht.begin();

  CekStatusAuto();
  BacaSsidPassword();
   
  //Disconnecting previous WiFi;
  WiFi.disconnect();
  ESPMqtt.setServer(mqttServer.c_str(), mqttPort.toInt());
  ESPMqtt.setCallback(mqttCallback);

 


  ledState ="off";
  connectToNetwork();

  Serial.println(WiFi.macAddress());
  Serial.println(WiFi.localIP());
  
  connectToMqtt();

  // start server
  server.begin();

}

void loop() { 
  BacaLux();
  BacaPerubahanTombol();
  AutoBrightness();
  BacaStatusSerial();

  if (WiFi.status() != WL_CONNECTED) {
    connectToNetwork();
  } 

  if (WiFi.status() == WL_CONNECTED && !ESPMqtt.connected()) {
    connectToMqtt();
  } 
  
  if (millis() - lastPublish > PUBLISH_INTERVAL) {
    updateDhtData();
    // Temp and Humidity
    Serial.print("Temperature = ");
    Serial.print(tempFloat);
    Serial.print(" C ; Humidity = ");
    Serial.print(humidFloat);
    Serial.println(" %");
     // Lux
    Serial.print("Intensitas cahaya= ");
    Serial.print(luxFloat);
    Serial.println(" lux");
    publishMessage();

    lastPublish = millis();

  }

  ESPMqtt.loop(); //agar tdk tiba2 disconnect

  WiFiClient client = server.available();

  if (client) {
    if (client.available()) {
      String request = client.readStringUntil('\n');
      Serial.println(request);

      client.println("HTTP/1.1 200 OK");
      client.println("Conten-type:text/html");
      client.println("Connection:close");
      client.println();

      if (request.indexOf("GET /led/on") >=0) {
        Serial.println("Client ask to turn LED on");
        ledState = "on";
        digitalWrite(BUILTIN_LED, HIGH);
      } else if (request.indexOf("GET /led/off") >=0) {
        Serial.println("Client ask to turn LED off");
        ledState = "off";
        digitalWrite(BUILTIN_LED, LOW);
      }
      
      client.println("<!DOCTYPE html><html><head>");
      client.println("<meta http-equiv=\"refresh\" content=\"30\" ");
      client.println("name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
      client.println("<link rel=\"icon\" href=\"data:,\">");
      
      client.println("<style>");
      client.println("body { text-align:center; font-family:\"Arial\"}");
      client.println("table { border-collapse:collapse; width:40%; margin-left:auto;");
      client.println("margin-right:auto; border-spacing:2px; background-color:white; border:4px solid green; }");
      client.println("th { padding:20px; background-color:#008000; color:white; }");
      client.println("tr { border: 5px solid green; padding: 2px; }");
      client.println("tr:hover { background-color:yellow; }");
      client.println("td {border:4px; padding: 12px; }");
      client.println(".sensor { color:red; font-weight: bold; padding: 1px; }"); 
      client.println(".button { background-color:#4CAF50; border:none; color:white; padding: 16px 40px; }");
      client.println("</style>");

      client.println("</head>");
      client.println("<body>");
      client.println("<h1>ESP32 Web Server reading sensor values</h1>");
      client.println("<h2>DHT11 & BH1750</h2>");
      client.println("<table><tr>");
      client.println("<th>MEASUREMENT</th><th>VALUE</th></tr>");
      client.println("<tr><td>Temp. Celcius</td><td><span class=\"sensor\">");
      client.println(tempFloat);
      client.println(" *C</td></tr>");
      client.println("<tr><td>Temp. fahrenheit</td><td><span class=\"sensor\">");
      client.println(tempFloat * 1.8 + 32);
      client.println(" *F</td></tr>");
      client.println("<tr><td>Humidity</td><td><span class=\"sensor\">");
      client.println(humidFloat);
      client.println(" %</td></tr");
      client.println("<tr><td>Intensitas cahaya</td><td><span class=\"sensor\">");
      client.println(luxFloat);
      client.println(" Lux</td></tr");
      client.println("</table>");
      
      // Tomb0l / Button
      if (ledState == "on") {
        client.println("<p><a href=\"/led/off\"><button class=\"button\">Lampu OFF</button></a></p>");  
      } else {
        client.println("<p><a href=\"/led/on\"><button class=\"button\">Lampu ON</button></a></p>");  
      }
      client.println("</body></html>");
      client.println();
      client.stop();

    }
  } 
}

void BacaSsidPassword() {
  EEPROM.begin(EEPROM_SIZE);
  readEEPROM(SSIDPASS_ADDRESS, DataSsidPass, SSIDPASS_SIZE);
  String DataStringSsidPass = String(DataSsidPass);
  int petikPertama = 0, petikKedua = 0;
  
  petikPertama = DataStringSsidPass.indexOf(";");
  Ssid = DataStringSsidPass.substring(0, petikPertama);
  Serial.println("SSID = " + Ssid);

  petikKedua = DataStringSsidPass.indexOf(";", petikPertama + 1);
  Password = DataStringSsidPass.substring(petikPertama + 1, petikKedua);
  Serial.println("Password = " + Password);

  if (Ssid == 0 && Password == 0) {
    modeWifi = false;
  } else {
    modeWifi = true;
  }
  Serial.println("status Wifi = " + String(modeWifi));

}

void readEEPROM(int address, char *Data, int size) {
  Serial.println("[EEPROM] mulai membaca dari EEPROM");
  for(int i = 0; i < size; i++) {
    Data[i] = EEPROM.read(address + i);
  }
}

void CekStatusAuto() {   //OK
  EEPROM.begin(128);
  delay(100);
  char Otomatis = EEPROM.read(0);
  Serial.println("Read Otomatis : " + char(Otomatis));
  if (Otomatis == '1') { 
    Serial.println("Otomatis = ON");
    statusAuto = 1;
  //  digitalWrite(BUILTIN_LED, HIGH);  
  }
  else {
    Serial.println("Otomatis = OFF");
    statusAuto = 0;
  //  digitalWrite(BUILTIN_LED, LOW);  
  }
}

void BacaStatusSerial() {
  while (Serial.available()) {
    char DataSerial = Serial.read();
    DataReceived += DataSerial;
    
    if (DataSerial == '\n') {
      Serial.println("Data diterima dari user: " + DataReceived);
      writeEEPROM(SSIDPASS_ADDRESS, DataReceived.c_str(), DataReceived.length());
      DataReceived ="";
    }
  }
}

void writeEEPROM(int address, const char *data, int size) {
  Serial.println("[EEPROM] mulai menulis ke EEPROM");
  for (int i = 0; i < size; i++) {
    EEPROM.write(address + i, data[i]);
  }
  EEPROM.commit();
}


void updateDhtData() { 
  tempFloat = dht.readTemperature();
  humidFloat = dht.readHumidity();
}

void BacaLux() { 
  bh1750Request(BH1750_ADDRESS);
  delay(100);
  if (bh1750GetData(BH1750_ADDRESS) == BH1750_DATALEN) {
    lux = (((unsigned short)buff[0] << 8) | (unsigned short)buff[1]) / 1.2;
  }

  if (lux >= 0 && lux <=2500) {
    dutyCycle = 250 - (lux/10);
  } else {
    dutyCycle = 0;
  }
//  Serial.println("  Nilai duty Cycle = " + String(dutyCycle) + " PWM");
  luxFloat = lux;
  delay(100);  
}

void bh1750Request(int address) { 
  Wire.beginTransmission(address);
  Wire.write(0x10);
  Wire.endTransmission();
}

int bh1750GetData(int address) { 
  int i = 0;
  Wire.beginTransmission(address);
  Wire.requestFrom(address, 2);
  while (Wire.available()) {
    buff[i] = Wire.read();
    i++;
  }
  Wire.endTransmission();

  return i;
}

void BacaPerubahanTombol() {
  if (buttonPressed) {
    statusAuto = !statusAuto;
    buttonPressed = false;
    if (statusAuto == 1) {
      Serial.println("Otomatis = ON");
    } else {
      Serial.println("Otomatis = OFF");
    }
  } 

}

void AutoBrightness() { 
  if (statusAuto) {
   ledcWrite(PWM_CHANNEL, dutyCycle);  
  // Serial.println("Otomatis = ON");
  }
  else {
    ledcWrite(PWM_CHANNEL, 0);
  //  Serial.println("Otomatis = OFF");
  }

}


void connectToMqtt() {
  while (!ESPMqtt.connected()) {
    Serial.println("ESP > Connecting to MQTT ....");

    if (ESPMqtt.connect("ESP32Client", mqttUser.c_str(), mqttPwd.c_str())) {
      Serial.println("Connected to Server");
      // subscribe to the topic
      ESPMqtt.subscribe(subTopic.c_str());
    } else {
        Serial.print("Failed with state");
        Serial.print(ESPMqtt.state());
        Serial.print("\r\n");
        delay(2000);
    }
  }
}

void publishMessage() {
  char msgToSend[1024] = {0};
  const size_t capacity = JSON_OBJECT_SIZE(4);
  DynamicJsonDocument doc(capacity);

  temperature = String(tempFloat);
  humidity = String(humidFloat);
  Stringlux = String(luxFloat);

  //doc["eventName"] = "sensorStatus";
  //doc["status"] = "none";
  doc["Pembacaan sensor"] = "sensorStatus";
  doc["temp"] = temperature.c_str();
  doc["humid"] = humidity.c_str();
  doc["lux"] = Stringlux.c_str();

  serializeJson(doc, msgToSend);

//  Serial.println("proses publish");
  ESPMqtt.publish(pubTopic.c_str(), msgToSend);
}

void connectToNetwork() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.print("Establishing connection to ");
    Serial.println(ssid);
  }
  Serial.println("Connected to network");
}

void mqttCallback(char *topic, byte *payload, long length) {
  char msg[256];

//  Serial.println("Messasge arrived [" + subTopic + "] ");
  for (int i = 0; i < length; i++) {
    msg[i] = (char)payload[i];
  }
  
  do_actions(msg);
}




void do_actions(const char *message) {
  const size_t capacity = JSON_OBJECT_SIZE(2) + 50;
  DynamicJsonDocument doc(capacity);

  deserializeJson(doc, message);

  const char *deviceId = doc["deviceId"];
  const char *lampStatus = doc["StatusLampu"];
  Serial.print("Nama device : ");
  Serial.println(deviceId);
  Serial.print("Perintah lampu : ");
  Serial.println(lampStatus);

  if (String(deviceId) == "LampuTeras") {
    if (String(lampStatus) == "ON") {
      Serial.println("TURN ON LAMP");
      digitalWrite(BUILTIN_LED, HIGH);
    } else if (String(lampStatus) == "OFF") {
      Serial.println("TURN OFF LAMP");
      digitalWrite(BUILTIN_LED, LOW);
    } else {
      Serial.println("Perintah tidak diproses karena Perintah ON-OFF lampu tidak dikenal");  
    }  
  } else {
    Serial.println("Perintah tidak diproses karena Nama device tidak cocok ");
  }
}

