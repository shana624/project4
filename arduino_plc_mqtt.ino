#include <WiFiS3.h>         // Arduino R4 WiFi 전용 라이브러리
#include <PubSubClient.h>    // MQTT 라이브러리
#include <SoftwareSerial.h>

// WiFi 네트워크 설정
const char* ssid = "SF_MAIN_2G";
const char* password = "";

// MQTT 브로커 설정
const char* mqtt_server = "192.168.0.188";
const int mqtt_port = 1883;
const char* mqtt_topic_cmd = "/arduino/plc/cmd";
const char* mqtt_topic_status = "/arduino/plc/status";
const char* mqtt_topic_debug = "/arduino/plc/debug";

// WiFi 및 MQTT 클라이언트 객체 생성
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// PLC 통신을 위한 SoftwareSerial 설정 (RX: 2, TX: 3)
SoftwareSerial plcSerial(2, 3);

#define EXPECTED_BYTES 2 // 2개의 WORD * 2바이트 = 4바이트

// 디버깅 출력 + MQTT 발행 함수
void sendDebug(String msg) {
  Serial.println(msg);
  mqttClient.publish(mqtt_topic_debug, msg.c_str());
}

String readPLCFixedMessage() {
  String message = "";
  while (plcSerial.available() < EXPECTED_BYTES) {
    // 데이터 대기
  }

  for (int i = 0; i < EXPECTED_BYTES; i++) {
    message += char(plcSerial.read());
  }

  return message;
}

// MQTT 메시지 수신 콜백 함수
void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  
  for (unsigned int i = 0; i < length; i++) {
    message += (char)payload[i];
  }

  sendDebug("[MQTT 수신] Topic: " + String(topic) + ", Message: " + message);

  if (message == "c1" || message == "c2" || message == "c3" || message == "go" || message == "s1" || message == "s2") {
    sendCommandToPLC(message);
  }
}

// PLC로 명령어 전송 함수
void sendCommandToPLC(String cmd) {
  sendDebug("[PLC 전송] " + cmd);
  plcSerial.println(cmd);
}

// WiFi 연결 함수
void setupWiFi() {
  sendDebug("WiFi 연결 중: " + String(ssid));
  
  WiFi.begin(ssid);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
  }

  sendDebug("WiFi 연결 완료! IP: " + WiFi.localIP().toString());
}

// MQTT 재연결 함수
void reconnectMQTT() {
  while (!mqttClient.connected()) {
    sendDebug("MQTT 브로커 연결 시도...");
    if (mqttClient.connect("ArduinoR4Client_plc")) {
      sendDebug("MQTT 연결 성공!");
      mqttClient.subscribe(mqtt_topic_cmd);
    } else {
      sendDebug("MQTT 연결 실패, 상태 코드 = " + String(mqttClient.state()));
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);
  plcSerial.begin(9600);

  setupWiFi();
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(callback);
}

void loop() {
  if (!mqttClient.connected()) {
    reconnectMQTT();
  }
  mqttClient.loop();

  if (plcSerial.available() >= EXPECTED_BYTES) {
    String plcMessage = readPLCFixedMessage();
    if (plcMessage.length() > 0) {
      sendDebug("[PLC 수신] " + plcMessage);
      mqttClient.publish(mqtt_topic_status, plcMessage.c_str());
    }
  }
}
