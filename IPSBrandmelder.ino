#include <PubSubClient.h>
#include <driver/adc.h>
#include "esp_camera.h"
#include <WiFi.h>

const char* ssid = "";
const char* password = "";
//Ipsymcon Server:
const char* mqtt_server = "10.0.0.93";

String DeviceName = "Brandmelder/Küche";

#define PWDN_GPIO_NUM     -1
#define RESET_GPIO_NUM    2
#define Y2_GPIO_NUM       4
#define Y3_GPIO_NUM       5
#define Y4_GPIO_NUM       18
#define Y5_GPIO_NUM       19
#define Y6_GPIO_NUM       36
#define Y7_GPIO_NUM       39
#define Y8_GPIO_NUM       34
#define Y9_GPIO_NUM       35
#define XCLK_GPIO_NUM     21
#define PCLK_GPIO_NUM     22
#define HREF_GPIO_NUM     23
#define VSYNC_GPIO_NUM    25
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27

void startCameraServer();

//Wifi Qualität
long rssi = 0;
long quality = 0;

int AlarmausgangPin = 15;
int TasterPin = 12;


  WiFiClient espClient;
  PubSubClient client(espClient);
  long lastMsg = 0;
  char msg[50];
  int value = 0;

  long lastMsgTaster = 0;
  long lastMsgAlarm = 0;

//Wakeup Pin: https://randomnerdtutorials.com/esp32-deep-sleep-arduino-ide-wake-up-sources/
//#define BUTTON_PIN_BITMASK 0x8004 // GPIOs 2 and 15
#define BUTTON_PIN_BITMASK 0x9000 // GPIOs 12 and 15
RTC_DATA_ATTR int bootCount = 0;

//ESP Schlafen in TIME_TO_SLEEP Sekunden
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP  10        /* Time ESP32 will go to sleep (in seconds) */
bool IsWakeUpByTimer = false;
long lastSleep = 0;


float val0 = 0;
float val1 = 0;
float val2 = 0;
float val3 = 0;
float val4 = 0;
float val5 = 0;
float val6 = 0;
float val7 = 0;
long lastVolt = 0;

void setup() {

  // Initialisiere ESP32 ADC
  adc1_config_width(ADC_WIDTH_12Bit);
  adc1_config_channel_atten(ADC1_CHANNEL_0,ADC_ATTEN_0db);
  adc1_config_channel_atten(ADC1_CHANNEL_1,ADC_ATTEN_0db);
  adc1_config_channel_atten(ADC1_CHANNEL_2,ADC_ATTEN_0db);
  adc1_config_channel_atten(ADC1_CHANNEL_3,ADC_ATTEN_0db);
  adc1_config_channel_atten(ADC1_CHANNEL_4,ADC_ATTEN_0db);
  adc1_config_channel_atten(ADC1_CHANNEL_5,ADC_ATTEN_0db);
  adc1_config_channel_atten(ADC1_CHANNEL_6,ADC_ATTEN_0db);
  adc1_config_channel_atten(ADC1_CHANNEL_7,ADC_ATTEN_0db);
  
  //Alarm Test Gelb
  pinMode(14, OUTPUT);
  digitalWrite(14, LOW);  
  
  //Kamera Power
  pinMode(16, OUTPUT);
  digitalWrite(16, HIGH);  

  //LED Weis
  pinMode(17, OUTPUT);
  digitalWrite(17, LOW);  

  //LED Rot
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);  

  delay(500);

  //LED Rot
  pinMode(13, OUTPUT);
  digitalWrite(13, LOW);  

  

  //Alarmausgang IO Rot
  pinMode(AlarmausgangPin, INPUT);
  //attachInterrupt(AlarmausgangPin, Alarm, CHANGE);

  //Taster
  pinMode(TasterPin, INPUT);
  //attachInterrupt(TasterPin, Taster, CHANGE);

  Serial.begin(115200);
  Serial.setDebugOutput(false);
  Serial.println();

  
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  //init with high specs to pre-allocate larger buffers
  if(psramFound()){
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Kamera init fehlgeschlagen mit Fehler 0x%x", err);
    return;
  }

  //drop down frame size for higher initial frame rate
  sensor_t * s = esp_camera_sensor_get();
  s->set_framesize(s, FRAMESIZE_QVGA);
  
  
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi Verbunden");

  rssi = WiFi.RSSI();
  Serial.print("RSSI:");
  Serial.println(rssi);

  startCameraServer();

  Serial.print("Kamera bereit! Verwende: 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' Verbunden");
  
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);


  //Increment boot number and print it every reboot
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));

  //Print the wakeup reason for ESP32
  print_wakeup_reason();

  //Print the GPIO used to wake up
  print_GPIO_wake_up();

  //If you were to use ext1, you would use it like
  esp_sleep_enable_ext1_wakeup(BUTTON_PIN_BITMASK,ESP_EXT1_WAKEUP_ANY_HIGH);

  //If you were to use ext0, you would use it like
  //Nur einer möglich!
  //esp_sleep_enable_ext0_wakeup(GPIO_NUM_12,1); //1 = High, 0 = Low
  //esp_sleep_enable_ext0_wakeup(GPIO_NUM_15,1); //1 = High, 0 = Low

  //esp aufwecken
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 aufwecken über Timer alle " + String(TIME_TO_SLEEP) +
  " Sekunden");
  
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Angekommene Meldung [");
  
  String Namen = String(topic);
  Serial.print(Namen);
  
  Serial.print("] ");
  
  String Auswertung = "";
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);  
    Auswertung = Auswertung + (char)payload[i];
  }


  if (Namen == DeviceName + "/Kamera")
  {
    if(Auswertung == "true") {
       digitalWrite(16, HIGH);
       startCameraServer();
       Serial.print("Kamera bereit! Verwende 'http://");
       Serial.print(WiFi.localIP());
       Serial.println("' Verbunden");
      } 
      if(Auswertung == "false") {
       digitalWrite(16, LOW); 
      } 
  }

  if (Namen == DeviceName + "/Licht")
  {
    if(Auswertung == "true") {
      digitalWrite(17, HIGH); 
    } 
    if(Auswertung == "false") {
      digitalWrite(17, LOW); 
    } 
  }
      
  if (Namen == DeviceName + "/Rot")
  {
    if(Auswertung == "true") {
      digitalWrite(13, HIGH); 
    } 
    if(Auswertung == "false") {
      digitalWrite(13, LOW); 
    }     
  }
  
  if (Namen == DeviceName + "/Test")
  {
    if(Auswertung == "true") {
      digitalWrite(14, HIGH); 
    } 
    if(Auswertung == "false") {
      digitalWrite(14, LOW); 
    }     
  }
  
  if (Namen == DeviceName + "/Reset")
  {
   if(Auswertung == "true") {
      ESP.restart();
      Auswertung = "";
    }      
  }

   if (Namen == DeviceName + "/Schlafen")
  {
   if(Auswertung == "true") {
      DeepSleep();
      Auswertung = "";
    }      
  }

  Serial.println(); 
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Verbinden zu MQTT ...");
    // Create a random client ID
    String clientId = DeviceName;
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("Verbunden");
      // Once connected, publish an announcement...

      String DeviceNameConnect = DeviceName + "->Verbunden";
      client.publish("outTopic", (char*) DeviceNameConnect.c_str());

      String DeviceNameLicht = DeviceName + "/Licht";
      client.publish((char*) DeviceNameLicht.c_str(), "false", true);
      String DeviceNameKamera = DeviceName + "/Kamera";
      client.publish((char*) DeviceNameKamera.c_str(), "false", true);
      String DeviceNameRot = DeviceName + "/Rot";
      client.publish((char*) DeviceNameRot.c_str(), "false", true);
      String DeviceNameTest = DeviceName + "/Test";
      client.publish((char*) DeviceNameTest.c_str(), "false", true);
      String DeviceNameReset = DeviceName + "/Reset";
      client.publish((char*) DeviceNameReset.c_str(), "false", true);
      String DeviceNameSchlafen = DeviceName + "/Schlafen";
      client.publish((char*) DeviceNameSchlafen.c_str(), "false", true);
      
      // ... and resubscribe
      //Ankommende Meldungen über diesen Namen akzeptieren

      String DeviceNameSend = DeviceName + "/";
      client.subscribe((char*) DeviceNameSend.c_str());
      client.subscribe((char*) DeviceNameLicht.c_str());
      client.subscribe((char*) DeviceNameKamera.c_str());
      client.subscribe((char*) DeviceNameRot.c_str());
      client.subscribe((char*) DeviceNameTest.c_str());
      client.subscribe((char*) DeviceNameReset.c_str());
      client.subscribe((char*) DeviceNameSchlafen.c_str());
      
    } else {
      Serial.print("Fehlgeschlagen, rc=");
      Serial.print(client.state());
    }
  }
}

void Alarm() {
  Serial.println("Alarm");

  char charBufAlarm[10];
     dtostrf(1, 1, 0, charBufAlarm);
      ++value;
      snprintf (msg, 50, charBufAlarm, value);
      Serial.print("Alarm Meldung: ");
      Serial.println(msg);
      String DeviceNameAlarm = DeviceName + "/Alarm";
      client.publish((char*) DeviceNameAlarm.c_str(), msg, true);
}

void Taster() {  
      Serial.println("Taster gedrückt");
      
       // Taster schicken 
     char charBufTaster[10];
     dtostrf(1, 1, 0, charBufTaster);
      ++value;
      snprintf (msg, 50, charBufTaster, value);
      Serial.print("Taster Meldung: ");
      Serial.println(msg);
      String DeviceNameTaster = DeviceName + "/Taster";
      client.publish((char*) DeviceNameTaster.c_str(), msg, true);
}


/*
Method to print the reason by which ESP32
has been awaken from sleep
*/
void print_wakeup_reason(){
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : 
    Serial.println("Wakeup caused by timer");
    IsWakeUpByTimer = true;
    break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n",wakeup_reason); break;
  }
}

/*
Method to print the GPIO that triggered the wakeup
*/
void print_GPIO_wake_up(){
  int GPIO_reason = esp_sleep_get_ext1_wakeup_status();
  Serial.print("GPIO that triggered the wake up: GPIO ");
  Serial.println((log(GPIO_reason))/log(2), 0);
}

void DeepSleep() {
  Serial.println("");
  Serial.println("ESP32 Schlafen gehen");
  Serial.flush(); 
  esp_deep_sleep_start();
  Serial.println("Fehler ESP32 schläft nicht!");
}

void ReadTasterAndAlarm() {
  if (digitalRead(TasterPin)== HIGH) {
   long nowTaster = millis();
     if (nowTaster - lastMsgTaster > 2000) {
     lastMsgTaster = nowTaster;
     Taster();
    }
   }

   if (digitalRead(AlarmausgangPin)== HIGH) {
    long nowAlarm = millis();
     if (nowAlarm - lastMsgAlarm > 2000) {
     lastMsgAlarm = nowAlarm;
     Alarm();
    }
   }
}


void WIFICheckOrRestart() {

   if (!client.connected()) {
    reconnect();
   }
   client.loop();
  
  int wifi_retry = 0;
  while(WiFi.status() != WL_CONNECTED && wifi_retry < 5 ) {
      wifi_retry++;
      Serial.println("WiFi not connected. Try to reconnect");
      WiFi.disconnect();
      WiFi.mode(WIFI_OFF);
      WiFi.mode(WIFI_STA);
      WiFi.begin(ssid, password);
      delay(100);
  }
  if(wifi_retry >= 5) {
      Serial.println("\nReboot");
      ESP.restart();
  }
}

void ReadVoltage() {
    val0 = adc1_get_voltage(ADC1_CHANNEL_0);
    val1 = adc1_get_voltage(ADC1_CHANNEL_1);
    val2 = adc1_get_voltage(ADC1_CHANNEL_2);
    val3 = adc1_get_voltage(ADC1_CHANNEL_3);
    val4 = adc1_get_voltage(ADC1_CHANNEL_4);
    val4 = val4 / 1000;
    val5 = adc1_get_voltage(ADC1_CHANNEL_5);
    val5 = val5 / 1000;
    val6 = adc1_get_voltage(ADC1_CHANNEL_6);
    val7 = adc1_get_voltage(ADC1_CHANNEL_7);
    Serial.print("Spannung: ");
    Serial.println(val5,1);
}

void SetStatusToMQTT() {
      // rssi Wifi Qualität 
      rssi = WiFi.RSSI();

      //dBm to Quality:
      if(rssi <= -100)
        quality = 0;
      else if(rssi >= -50)
        quality = 100;
      else
        quality = 2 * (rssi + 100);
      
      char charBufQualitaet[20];
      dtostrf(quality,2, 2, charBufQualitaet);    
      snprintf (msg, 50, charBufQualitaet, value);
      String DeviceNameQualitaet = DeviceName + "/WifiQualität";
      client.publish((char*) DeviceNameQualitaet.c_str(), msg, true);
    
      // Taster 0 setzen 
      char charBufTaster[10];
      dtostrf(0,1, 0, charBufTaster);    
      snprintf (msg, 50, charBufTaster, value);
       String DeviceNameTaster = DeviceName + "/Taster";
      client.publish((char*) DeviceNameTaster.c_str(), msg, true);

      // Alarm 0 setzen 
      char charBufAlarm[10];
      dtostrf(0,1, 0, charBufAlarm); 
      snprintf (msg, 50, charBufAlarm, value);
      String DeviceNameAlarm = DeviceName + "/Alarm";
      client.publish((char*) DeviceNameAlarm.c_str(), msg, true);

      // Ip Adresse senden 
      char charBufIp[30];
      String IpAdresse = WiFi.localIP().toString();
      IpAdresse.toCharArray(charBufIp, 30);
      snprintf (msg, 50, charBufIp, value);
      String DeviceNameIpAdresse = DeviceName + "/IpAdresse";
      client.publish((char*) DeviceNameIpAdresse.c_str(), msg, true);

      // Ip Adresse Stream senden 
      char charBufIpStream[50];
      String IpAdresseStream = WiFi.localIP().toString() + ":81/stream";
      IpAdresseStream.toCharArray(charBufIpStream, 50);
      snprintf (msg, 50, charBufIpStream, value);
      String DeviceNameIpAdresseStream = DeviceName + "/IpAdresseStream";
      client.publish((char*) DeviceNameIpAdresseStream.c_str(), msg, true);


       // Ip Adresse Bild senden 
      char charBufIpBild[50];
      String IpAdresseBild = WiFi.localIP().toString() + "/capture";
      IpAdresseBild.toCharArray(charBufIpBild, 50);
      snprintf (msg, 50, charBufIpBild, value);
      String DeviceNameIpAdresseBild = DeviceName + "/IpAdresseBild";
      client.publish((char*) DeviceNameIpAdresseBild.c_str(), msg, true);


      // MAC Adresse senden 
      char charBufMac[50];
      String macAddrString = String(WiFi.macAddress());
      macAddrString.toCharArray(charBufMac, 50);
      snprintf (msg, 50, charBufMac, value);
      String DeviceNameMac = DeviceName + "/MacAdresse";
      client.publish((char*) DeviceNameMac.c_str(), msg, true);


      // Spannung senden 
      char charBufVolt[50];
      String VoltString = String(val5);
      VoltString.toCharArray(charBufVolt, 50);
      snprintf (msg, 50, charBufVolt, value);
      String DeviceNameVolt = DeviceName + "/Spannung";
      client.publish((char*) DeviceNameVolt.c_str(), msg, true);


      // Reset 0 setzen 
      char charBufReset[30];
      String ResetFalse = "false";
      ResetFalse.toCharArray(charBufReset, 30);
      snprintf (msg, 50, charBufReset, value);
      String DeviceNameReset = DeviceName + "/Reset";
      client.publish((char*) DeviceNameReset.c_str(), msg, true);


      // Schlafen 0 setzen 
      char charBufSchlafen[30];
      String SchlafenFalse = "false";
      SchlafenFalse.toCharArray(charBufSchlafen, 30);
      snprintf (msg, 50, charBufSchlafen, value);
      String DeviceNameSchlafen = DeviceName + "/Schlafen";
      client.publish((char*) DeviceNameSchlafen.c_str(), msg, true);


       // Schlafen Status setzen 
      char charBufSchlafenStatus[30];
      String SchlafenStatus = String(IsWakeUpByTimer);
      SchlafenStatus.toCharArray(charBufSchlafenStatus, 30);
      snprintf (msg, 50, charBufSchlafenStatus, value);
      String DeviceNameSchlafenStatus = DeviceName + "/SchlafenStatus";
      client.publish((char*) DeviceNameSchlafenStatus.c_str(), msg, true);

      // Kamera 1 setzen 
      char charBufKamera[30];
      String KameraTrue = "true";
      KameraTrue.toCharArray(charBufKamera, 30);
      snprintf (msg, 50, charBufKamera, value);
      String DeviceNameKamera = DeviceName + "/Kamera";
      client.publish((char*) DeviceNameKamera.c_str(), msg, true);
}


void loop() {

  WIFICheckOrRestart();
  ReadTasterAndAlarm();
   
  long now = millis();

  if (now - lastVolt > 2000) {
    ReadVoltage();
    lastVolt = now;
  }
  
  if (now - lastMsg > 8000) {
    SetStatusToMQTT();
    ++value;
    lastMsg = now;
  }

  if (now - lastSleep > 16000) {
   if (IsWakeUpByTimer == true) {
    DeepSleep();
    lastSleep = now;
    } 
  }

}
