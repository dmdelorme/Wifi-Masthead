/******************************************************
  DavisWindmeter 6410 esp32 feather
  Analog Input: GPIO34 (Wind dir) .
  Digital input GPIO21  (Windspeed)
  Digital output GPIO13 (Anchor light)
  David Delorme 08/12/18
  This is setup for my network ie 192.168.1.7 is my
  Openplotter rpi
  some more testing
*****************************************************/
#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "credentials.h"
#include <HTTPClient.h>
#include "ArduinoJson.h"

// WiFi network name and password:
int lastlight;
const char * networkName = "XXXXXXXX";
const char * networkPswd = "XXXXXXXXX";
const char * AP_SSID = "Windsensor";
//IP address to send UDP data to:
// either use the ip address of the server or
// a network broadcast address
const char * udpAddress = "192.168.1.7";
const int udpPort = 10110;
#define ANENOMETER
const byte WDIR_PIN = 34;// A2/GP34 ADC#1 so it will work with wifi
const byte WSPD_PIN = 21; // pin of ws
const float COR_WD = 0; // adjust windvane to boat centerline
#define ANCHORLIGHT
const byte ALT_PIN = 13; // pin of anchor light and bat led
volatile unsigned int rotation_count; //windspeed counter
volatile uint16_t lastperiod;
//Are we currently connected?
boolean connected = false;
//The udp library class
WiFiUDP udp;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

void getlight();
uint8_t checksum(const char *buf);
void IRAM_ATTR isr_anenometer_count();
void connectToWiFi(const char * ssid, const char * pwd);
void WiFiEvent(WiFiEvent_t event);
void send_nmea(const char *buf);
void read_anenometer();
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);
double ReadVoltage(byte pin);
void setup()
{
  // Setup needed for OTA
  Serial.begin(115200);
  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }
  // Port defaults to 3232                            // ArduinoOTA.setPort(3232);
  // Hostname defaults to esp3232-[MAC]               // ArduinoOTA.setHostname("myesp32");
  // No authentication by default                     // ArduinoOTA.setPassword("admin");
  // Password can be set with it's md5 value as well  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";
    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type);
  });
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR)         Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR)   Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR)     Serial.println("End Failed");
  });
  ArduinoOTA.begin();
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());


#ifdef ANENOMETER
  pinMode(WDIR_PIN, INPUT);
  pinMode(WSPD_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(WSPD_PIN), isr_anenometer_count, FALLING);
#endif

#ifdef ANCHORLIGHT
  pinMode(ALT_PIN, OUTPUT);
#endif
  //Connect to the WiFi network
  connectToWiFi(networkName, networkPswd);
}

void loop()
{
  ArduinoOTA.handle();
  yield();
  if (connected) {
    read_anenometer();
    getlight();
  }

  //Serial.println(analogRead(WDIR_PIN));
  delay(1);
}

void getlight()
{
  uint16_t time = millis();
  static uint16_t last_time;
  uint16_t dt = time - last_time;
  IPAddress server(192, 168, 1, 7);

  //String lights;
  //const char path = "{\"updates\":[{ \"source\": \"OPwifi.masthead\",\"values\":[{\"path\": \"navigation.lights\",\"value\":\"";
  if (dt >= 1000) { // output every 1s
    last_time += 1000;
    if ((WiFi.status() == WL_CONNECTED)) { //Check the current connection status
      //WiFiClient client;
      HTTPClient http;
      http.begin("http://192.168.1.7:3000/signalk/v1/api/vessels/self/sensors/anchorLight"); //Specify the URL
      int httpCode = http.GET();                                        //Make the request
      char lightstate[10];
      if (httpCode > 0) { //Check for the returning code
        String payload = http.getString();
        StaticJsonBuffer<500> jsonBuffer;
        JsonObject& root = jsonBuffer.parseObject(payload);
        http.end(); //Free the resources
        int light = root["value"];

        if (light == lastlight) {
          return;
        }
        if (light == 1) {
          digitalWrite (ALT_PIN, HIGH);  // turn on the LED
          lastlight = light;
          //lights = "anchored";
          strcpy(lightstate, "anchored");
        } else if  (light == 0) {
          digitalWrite (ALT_PIN, LOW);  // turn off the LED
          lastlight = light;
          //lights = "Off";
          strcpy(lightstate, "Off");
        } else {
          Serial.println("Fail");
        }
        //http.end(); //Free the resources
        //return;
      } else {
        Serial.println("get http failed");
      }
      //Serial.println(lights);

      char buf3 [128];

      snprintf(buf3, sizeof buf3, "{\"updates\":[{ \"$source\": \"OPwifi.masthead\",\"values\":[{\"path\": \"navigation.lights\",\"value\":\"%s\" }]}]}\n", lightstate);

      udp.beginPacket(udpAddress, 55561);
      udp.printf(buf3);
      Serial.println(buf3);
      udp.endPacket();
      //      Serial.println("Connected to server successful!");

    } else {
      Serial.println("Error not connected");
    }

  }
}

uint8_t checksum(const char *buf)
{
  uint8_t cksum = 0;
  for (uint8_t i = 0; i < strlen(buf); i++)
    cksum ^= buf[i];
  return cksum;
}
// stuff done durring windspeed Interrupts
void IRAM_ATTR isr_anenometer_count()
{
  static uint16_t lastt;
  int t = millis();
  uint16_t period = t - lastt;
  if (period < 15) // debounce, at least for less than 130 knots of wind
    return;
  lastt = t;
  portENTER_CRITICAL_ISR(&mux);
  lastperiod += period;
  rotation_count++;
  portEXIT_CRITICAL_ISR(&mux);
}
void connectToWiFi(const char * ssid, const char * pwd) {
  Serial.println("Connecting to WiFi network: " + String(ssid));

  WiFi.disconnect(true);
  //register event handler
  WiFi.onEvent(WiFiEvent);

  //Initiate connection
  WiFi.begin(ssid, pwd);
  WiFi.setHostname("esp32");

  Serial.println("Waiting for WIFI connection...");
}
//wifi event handler
void WiFiEvent(WiFiEvent_t event) {
  switch (event) {
    case SYSTEM_EVENT_STA_GOT_IP:
      //When connected set
      Serial.print("WiFi connected! IP address: ");
      Serial.println(WiFi.localIP());
      connected = true;
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:
      Serial.println("WiFi lost connection");
      connected = false;
      break;
  }
}
void send_nmea(const char *buf)
{
  char buf2[32] = {};
  snprintf(buf2, sizeof buf2, "$%s*%02x\r\n", buf, checksum(buf));
  if (connected) {
    udp.beginPacket(udpAddress, udpPort);
    udp.printf(buf2);
    udp.endPacket();
  }

}
void read_anenometer()
{
  float dir = 0;
  static float lpdir = 0;
  const float lp = .1;
  static float maxdir;
  static float mindir;

  //dir = analogRead(WDIR_PIN); //mapping of voltage off
  dir = ReadVoltage(WDIR_PIN); //closer mapping
  dir = mapfloat(dir, 0, 4095, 0, 356); //chop off the 5deg we cant read
  dir += COR_WD;
  if (lpdir - dir > 180)
    dir += 360;
  else if (dir - lpdir > 180)
    dir -= 360;
  //Serial.println(dir);
  //Serial.println(lpdir);
  lpdir = lp * dir + (1 - lp) * lpdir;
  //Serial.println(lpdir);
  if (lpdir >= 360)
    lpdir -= 360;
  else if (lpdir < 0)
    lpdir += 360;
  //Serial.println(dir);
  //Serial.println(lpdir);
  uint16_t time = millis();
  static uint16_t last_time;
  uint16_t dt = time - last_time;
  //delay(1000);
  if (dt >= 100) { // output every 100ms
    last_time += 100;

    uint16_t period = lastperiod;
    uint16_t count = rotation_count;

    portENTER_CRITICAL(&mux);
    rotation_count = 0;
    lastperiod = 0;
    portEXIT_CRITICAL(&mux);

    static uint16_t nowindcount;
    static float knots = 0;
    const int nowindtimeout = 30;
    if (count) {
      if (nowindcount != nowindtimeout)
        knots = .868976 * 2.25 * 1000 * count / period;
      nowindcount = 0;
    } else {
      if (nowindcount < nowindtimeout)
        nowindcount++;
      else
        knots = 0;
    }
    char buf[32];
    //Serial.println(lpdir);
    snprintf(buf, sizeof buf, "ARMWV,%d.%02d,R,%d.%02d,N,A", (int)lpdir, (uint16_t)(lpdir * 100.0) % 100U, (int)knots, (int)(knots * 100) % 100);
    send_nmea(buf);
    //Serial.println(knots);
  }
}
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
double ReadVoltage(byte pin) {
  double reading = analogRead(pin); // Reference voltage is 3v3 so maximum reading is 3v3 = 4095 in range 0 to 4095
  if (reading < 1 || reading > 4095) return 0;
  // return  * pow(reading,3) + 0.000000016557283 * pow(reading,2) + 0.000854596860691 * reading + 0.065440348345433;
  return (-0.000000000000016 * pow(reading, 4) + 0.000000000118171 * pow(reading, 3) - 0.000000301211691 * pow(reading, 2) + 0.001109019271794 * reading + 0.034143524634089) * 1280;
} // Added an improved polynomial, use either, comment out as required
