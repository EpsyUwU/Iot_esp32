#include <WiFi.h>               // Libreria para conectarce a la  red WIFI
#include <PubSubClient.h>        
#include <DHT.h>
#include <DHT_U.h>
#include <Adafruit_BMP085_U.h>
#include <ArduinoJson.h>

#define rainAnalog 35
#define rainDigital 34
#define DHTPIN 27
#define DHTTYPE DHT11
#define DHTPIN2 26
#define DHTTYPE2 DHT11
#define MQ135_PIN 32

const int motorPin1 = 2; 
const int motorPin2 = 4; 

const char* ssid = "MABE_INTERNET";
const char* password = "adminadmin";
const char* mqttServer = "192.168.7.163";
const int mqttPort = 1883;
const char* mqttUser = "esteban";
const char* mqttPassword = "esteban";

WiFiClient espClient;
PubSubClient client(espClient);

DHT dht(DHTPIN, DHTTYPE);
DHT dht2(DHTPIN2, DHTTYPE2);

Adafruit_BMP085_Unified bmp = Adafruit_BMP085_Unified(10085);

void setup() {
  pinMode(motorPin1, OUTPUT);
  pinMode(motorPin2, OUTPUT);
  
  Serial.begin(115200);
  dht.begin();
  dht2.begin();

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.println("Connecting to WiFi..");
  }
  Serial.println("Connected to the WiFi network");

  client.setServer(mqttServer, mqttPort);
  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");
    if (client.connect("ESP32Client", mqttUser, mqttPassword )) {
      Serial.println("connected");
    } else {
      Serial.print("failed with state ");
      Serial.print(client.state());
      delay(2000);
    }
  }
}

void loop() {
  int mq135_value = analogRead(MQ135_PIN);
  int rainAnalogVal = analogRead(rainAnalog);
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  float h2 = dht2.readHumidity();
  float t2 = dht2.readTemperature();

  if (!bmp.begin()) {
    Serial.println("No se pudo encontrar el sensor BMP180");
  }

  if (isnan(h) || isnan(t)) {
    Serial.println("Error al leer el sensor DHT11");
    return;
  }

  if (isnan(h2) || isnan(t2)) {
    Serial.println("Error al leer el sensor DHT11 2");
    return;
  }

  float p;
  bmp.getPressure(&p);

  const size_t bufferSize = JSON_OBJECT_SIZE(6);

    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);

  Serial.print("Estado de la bomba: ");
  if (rainAnalogVal < 4095){
    Serial.println("Bomba activa");
  }else{
    Serial.println("bomba apagada");
  }

  Serial.print("Calidad aire: ");
  Serial.println(mq135_value);
  Serial.print(" ppm\t");

  Serial.print("Humedad: ");
  Serial.print(h);
  Serial.print(" %\t");

  Serial.print("Temperatura: ");
  Serial.print(t);
  Serial.print(" *C\t");

  Serial.print("Humedad 2: ");
  Serial.print(h2);
  Serial.print(" %\t");

  Serial.print("Temperatura 2: ");
  Serial.print(t2);
  Serial.print(" *C\t");

  Serial.print("Presion: ");
  Serial.print(p);
  Serial.print(" Pa\t");

  StaticJsonDocument<bufferSize> jsonDoc;   
  jsonDoc["agua"] = rainAnalogVal;
  jsonDoc["calidad_aire"] = mq135_value;
  jsonDoc["humedad"] = h; 
  jsonDoc["humedad2"] = h2;
  jsonDoc["temperatura"] = t;
  jsonDoc["presion"] = p;

  char payload[200];
  serializeJson(jsonDoc, payload);
  client.publish("esp32/input", payload);

  delay(2000);
}