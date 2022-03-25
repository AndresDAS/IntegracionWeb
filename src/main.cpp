#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <stdlib.h>
#include "data.h"
#include "Settings.h"
#include "UbidotsEsp32Mqtt.h"
#include <UbiConstants.h>
#include <UbidotsEsp32Mqtt.h>
#include <UbiTypes.h>
#include "UbidotsEsp32Mqtt.h"
#include <cstring>
#include <Adafruit_Sensor.h>
#include <DHT.h>
#include <DHT_U.h>
#include <TFT_eSPI.h>
#include <SPI.h>

#define DHTPIN 27
#define DHTTYPE DHT11
#define BUTTON_LEFT 0        // btn activo en bajo
#define LONG_PRESS_TIME 3000 // 3000

const uint8_t LED1 = 32; // Pin used to write data based on 1's and 0's coming from Ubidots
const uint8_t LED2 = 33; // Pin used to write data based on 1's and 0's coming from Ubidots

const char *UBIDOTS_TOKEN = "BBFF-co4IS9ILXqSDZogZGTAC3DMUnyAt9s"; // Put here your Ubidots TOKEN
const char *DEVICE_LABEL = "esp32";                                // Replace with the device label to subscribe to
const char *VARIABLE_LABEL = "SW1";                                // Replace with your variable label to subscribe to
const char *VARIABLE_LABEL2 = "SW2";
const char *VARIABLE_LABEL3 = "temp";
const char *VARIABLE_LABEL4 = "hum"; // Put here your Variable label to which data  will be published

char msg1[] = "/v2.0/devices/esp32/sw1/lv";
char msg2[] = "/v2.0/devices/esp32/sw2/lv";

const int PUBLISH_FREQUENCY = 5000; // Update rate in milliseconds

int estadoled1 = 0, estadoled2 = 0;
int cmp1 = 1, cmp2 = 1;
int pc1x = 40, pc2x = 100, pcy = 205, r = 20;

unsigned long timer;
uint8_t analogPin = 34; // Pin used to read data from GPIO34 ADC_CH6.

TFT_eSPI tft = TFT_eSPI();

DHT dht(DHTPIN, DHTTYPE);

WebServer server(80);

Settings settings;
int lastState = LOW; // para el btn
int currentState;    // the current reading from the input pin
unsigned long pressedTime = 0;
unsigned long releasedTime = 0;

Ubidots ubidots(UBIDOTS_TOKEN);

void load404();
void loadIndex();
void loadFunctionsJS();
void restartESP();
void saveSettings();
bool is_STA_mode();
void AP_mode_onRst();
void STA_mode_onRst();
void detect_long_press();
void callback(char *topic, byte *payload, unsigned int length);

// Rutina para iniciar en modo AP (Access Point) "Servidor"
void startAP()
{
  WiFi.disconnect();
  delay(19);
  Serial.println("Starting WiFi Access Point (AP)");
  WiFi.softAP("Andres_AP", "facil123");
  IPAddress IP = WiFi.softAPIP();
  Serial.print("AP IP address: ");
  Serial.println(IP);
}

// Rutina para iniciar en modo STA (Station) "Cliente"
void start_STA_client()
{
  WiFi.softAPdisconnect(true);
  WiFi.disconnect();
  delay(100);
  Serial.println("Starting WiFi Station Mode");
  WiFi.begin((const char *)settings.ssid.c_str(), (const char *)settings.password.c_str());
  WiFi.mode(WIFI_STA);

  int conta = 0;
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(5000);
    // Serial.print(".");
    if (conta == 100) // Si después de 100 intentos no se conecta, vuelve a modo AP
      AP_mode_onRst();
    conta++;
    Serial.println("attempt # " + (String)conta);
  }

  WiFi.setAutoReconnect(true);
  Serial.println(F("WiFi connected"));
  Serial.println(F("IP address: "));
  Serial.println(WiFi.localIP());
  pressedTime = millis();
  // Rutinas de Ubidots
  ubidots.setCallback(callback);
  ubidots.setup();
  ubidots.reconnect();
  ubidots.subscribeLastValue(DEVICE_LABEL, VARIABLE_LABEL); // Insert the dataSource and Variable's Labels
  ubidots.subscribeLastValue(DEVICE_LABEL, VARIABLE_LABEL2);

  timer = millis();
}

void setup()
{

  Serial.begin(115200);
  delay(2000);

  EEPROM.begin(4096);                 // Se inicializa la EEPROM con su tamaño max 4KB
  pinMode(BUTTON_LEFT, INPUT_PULLUP); // btn activo en bajo

  // Inicializacion de TFT
  tft.init();
  tft.setTextColor(TFT_WHITE); // Color de letra
  tft.fillScreen(TFT_BLACK);   // Color de pantalla

  // inicializacion pines LED
  pinMode(LED1, OUTPUT);
  pinMode(LED2, OUTPUT);

  // Inicializacion DHT
  Serial.println(F("DHTxx test!"));
  dht.begin();

  // settings.reset();
  settings.load(); // se carga SSID y PWD guardados en EEPROM
  settings.info(); // ... y se visualizan

  Serial.println("");
  Serial.println("starting...");

  if (is_STA_mode())
  {
    start_STA_client();
  }
  else // Modo Access Point & WebServer
  {
    startAP();

    /* ========== Modo Web Server ========== */

    /* HTML sites */
    server.onNotFound(load404);

    server.on("/", loadIndex);
    server.on("/index.html", loadIndex);
    server.on("/functions.js", loadFunctionsJS);

    /* JSON */
    server.on("/settingsSave.json", saveSettings);
    server.on("/restartESP.json", restartESP);

    server.begin();
    Serial.println("HTTP server started");
  }
}

void loop()
{
  if (is_STA_mode()) // Rutina para modo Station (cliente Ubidots)
  {
    delay(500);
    float h = dht.readHumidity();
    float t = dht.readTemperature();

    if (isnan(h) || isnan(t))
    {
      tft.fillScreen(TFT_RED);
      tft.drawString("Failed to read from", 10, 20, 2);
      tft.drawString("DHT sensor!", 10, 40, 2);
      return;
    }

    tft.fillScreen(TFT_BLACK);
    tft.drawString("Humedad (%): ", 10, 10, 2);
    tft.drawString(String(h), 0, 40, 7);
    tft.drawString("Temperatura (°C): ", 10, 100, 2);
    tft.drawString(String(t), 0, 130, 7);

    // Encendido y Apagado de LEDs
    if (estadoled1 == 1 && estadoled2 == 1)
    {
      tft.fillCircle(pc1x, pcy, r, TFT_RED);
      tft.fillCircle(pc2x, pcy, r, TFT_GREEN);
      digitalWrite(LED1, HIGH);
      digitalWrite(LED2, HIGH);
    }
    if (estadoled1 == 1 && estadoled2 == 0)
    {
      tft.fillCircle(pc1x, pcy, r, TFT_RED);
      tft.drawCircle(pc2x, pcy, r, TFT_GREEN);
      digitalWrite(LED1, HIGH);
      digitalWrite(LED2, LOW);
    }
    if (estadoled1 == 0 && estadoled2 == 1)
    {
      tft.drawCircle(pc1x, pcy, r, TFT_RED);
      tft.fillCircle(pc2x, pcy, r, TFT_GREEN);
      digitalWrite(LED1, LOW);
      digitalWrite(LED2, HIGH);
    }
    if (estadoled1 == 0 && estadoled2 == 0)
    {
      tft.drawCircle(pc1x, pcy, r, TFT_RED);
      tft.drawCircle(pc2x, pcy, r, TFT_GREEN);
      digitalWrite(LED1, LOW);
      digitalWrite(LED2, LOW);
    }
    if (!ubidots.connected())
    {
      ubidots.reconnect();
    }
    if (abs(millis() - timer) > PUBLISH_FREQUENCY) // triggers the routine every 5 seconds
    {
      float value = (float)random(300);
      ubidots.add(VARIABLE_LABEL, value); // Insert your variable Labels and the value to be sent
      ubidots.publish(DEVICE_LABEL);
      timer = millis();
    }
    ubidots.loop();
  }
  else // rutina para AP + WebServer
    server.handleClient();

  delay(10);
  detect_long_press();
}

// funciones para responder al cliente desde el webserver:
// load404(), loadIndex(), loadFunctionsJS(), restartESP(), saveSettings()

void load404()
{
  server.send(200, "text/html", data_get404());
}

void loadIndex()
{
  server.send(200, "text/html", data_getIndexHTML());
}

void loadFunctionsJS()
{
  server.send(200, "text/javascript", data_getFunctionsJS());
}

void restartESP()
{
  server.send(200, "text/json", "true");
  ESP.restart();
}

void saveSettings()
{
  if (server.hasArg("ssid"))
    settings.ssid = server.arg("ssid");
  if (server.hasArg("password"))
    settings.password = server.arg("password");

  settings.save();
  server.send(200, "text/json", "true");
  STA_mode_onRst();
}

// Rutina para verificar si ya se guardó SSID y PWD del cliente
// is_STA_mode retorna true si ya se guardaron
bool is_STA_mode()
{
  if (EEPROM.read(flagAdr))
    return true;
  else
    return false;
}

void AP_mode_onRst()
{
  EEPROM.write(flagAdr, 0);
  EEPROM.commit();
  delay(100);
  ESP.restart();
}

void STA_mode_onRst()
{
  EEPROM.write(flagAdr, 1);
  EEPROM.commit();
  delay(100);
  ESP.restart();
}

void detect_long_press()
{
  // read the state of the switch/button:
  currentState = digitalRead(BUTTON_LEFT);

  if (lastState == HIGH && currentState == LOW) // button is pressed
    pressedTime = millis();
  else if (lastState == LOW && currentState == HIGH)
  { // button is released
    releasedTime = millis();

    // Serial.println("releasedtime" + (String)releasedTime);
    // Serial.println("pressedtime" + (String)pressedTime);

    long pressDuration = releasedTime - pressedTime;

    if (pressDuration > LONG_PRESS_TIME)
    {
      Serial.println("(Hard reset) returning to AP mode");
      delay(500);
      AP_mode_onRst();
    }
  }

  // save the the last state
  lastState = currentState;
}

void callback(char *topic, byte *payload, unsigned int length)
{
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  // Cambios de estados
  cmp1 = strcmp(topic, msg1);
  cmp2 = strcmp(topic, msg2);

  if (cmp1 == 0)
  {
    if ((char)payload[0] == '0')
    {
      estadoled1 = 0;
      Serial.println("LED 1 off");
    }
    if ((char)payload[0] == '1')
    {
      estadoled1 = 1;
      Serial.println("LED 1 on");
    }
  }

  if (cmp2 == 0)
  {
    if ((char)payload[0] == '0')
    {
      estadoled2 = 0;
      Serial.println("LED 2 off");
    }
    if ((char)payload[0] == '1')
    {
      estadoled2 = 1;
      Serial.println("LED 2 on");
    }
  }
}