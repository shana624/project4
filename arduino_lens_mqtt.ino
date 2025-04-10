#include <SoftwareSerial.h>
#include <WiFiS3.h>
#include <PubSubClient.h>
#include <HUSKYLENS.h>

HUSKYLENS huskylens;
SoftwareSerial huskySerial(2, 3);  // RX, TX
WiFiClient wifiClient;
PubSubClient client(wifiClient);

const char* ssid = "SF_MAIN_2G";
const char* password = "";
const char* mqttServer = "192.168.0.188";
const int mqttPort = 1883;
const char* topic_status = "/arduino/lens/status";
const char* topic_debug = "/arduino/lens/debug";

int previousID = 0;
unsigned long idStartTime = 0;
bool messageSent = false;
const unsigned long requiredTime = 500;

void sendDebug(String msg) {
  Serial.println(msg);
  client.publish(topic_debug, msg.c_str());
}

void connectWiFi() {
  WiFi.begin(ssid);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  sendDebug("WiFi connected");
}

void connectMQTT() {
  client.setServer(mqttServer, mqttPort);
  while (!client.connected()) {
    if (client.connect("ArduinoClient_lens")) {
      sendDebug("Connected to MQTT broker");
    } else {
      delay(500);
    }
  }
}

bool initHuskylens() {
  for (int i = 0; i < 5; i++) {
    if (huskylens.begin(huskySerial)) {
      sendDebug("Huskylens Ready!!");
      return true;
    }
    sendDebug("Huskylens init failed, retrying...");
    delay(500);
  }
  return false;
}

void setup() {
  Serial.begin(115200);
  huskySerial.begin(9600);

  if (!initHuskylens()) {
    sendDebug("Failed to initialize Huskylens after retries.");
    while (1);
  }

  connectWiFi();
  connectMQTT();
}

void loop() {
  if (!huskylens.request()) {
    sendDebug("fail to request data from lens");
    delay(100);
    return;
  }

  if (!huskylens.isLearned()) {
    sendDebug("NOTHING LEARNED");
    return;
  }

  if (!huskylens.available()) {
    sendDebug("No block or arrow appeared");
    previousID = 0;
    messageSent = false;
    return;
  }

  while (huskylens.available()) {
    HUSKYLENSResult result = huskylens.read();
    checkAndPublish(result.ID);
  }

  client.loop();
}

void checkAndPublish(int currentID) {
  if (currentID == 1 || currentID == 2 || currentID == 3) {
    String idMsg = "ID: " + String(currentID);
    sendDebug(idMsg);

    if (previousID != currentID) {
      previousID = currentID;
      idStartTime = millis();
      messageSent = false;
    }

    if (!messageSent && (millis() - idStartTime >= requiredTime)) {
      String message = "c" + String(currentID);
      client.publish(topic_status, message.c_str());
      sendDebug("Published: " + message);
      messageSent = true;
    }
  } else {
    previousID = 0;
    messageSent = false;
  }
}
