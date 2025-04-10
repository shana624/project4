#include <Servo.h>
#include <WiFiS3.h>
#include <PubSubClient.h>

// 서보 모터 설정
Servo microServo;
const int servoPin = 9;

// Wi-Fi 정보
const char* ssid = "SF_MAIN_2G";
const char* password = "";

// MQTT 브로커 정보
const char* mqtt_server = "192.168.0.188";
const int mqtt_port = 1883;
const char* mqtt_topic_cmd = "/arduino/door/cmd";
const char* mqtt_topic_status = "/arduino/door/status";
const char* mqtt_topic_debug = "/arduino/door/debug";

WiFiClient espClient;
PubSubClient client(espClient);

void setup() {
  Serial.begin(115200);
  microServo.attach(servoPin);
  microServo.write(10);
  delay(500);

  connectWiFi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
  connectMQTT();
}

void loop() {
  if (!client.connected()) {
    connectMQTT();
  }
  client.loop();
}

void connectWiFi() {
  sendDebug("Wi-Fi 연결 시도");
  WiFi.begin(ssid);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 30) {
    delay(1000);
    sendDebug("Wi-Fi 연결중...");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    sendDebug("Wi-Fi 연결 성공 : " + WiFi.localIP().toString());
  } else {
    sendDebug("Wi-Fi 연결 실패");
  }
}

void connectMQTT() {
  while (!client.connected()) {
    sendDebug("MQTT 연결 시도중...");
    if (client.connect("ArduinoClient_door")) {
      sendDebug("MQTT 연결 성공");
      client.subscribe(mqtt_topic_cmd);
    } else {
      sendDebug("MQTT 연결 실패.. 5초 후 재시도");
      delay(5000);
    }
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  String message = "";
  for (int i = 0; i < length; i++) {
    message += (char)payload[i];
  }
  message.trim();

  sendDebug("수신된 명령 : " + message);

  if (message == "open") {
    openDoor();
    sendStatus("open_done");
  } else if (message == "close") {
    closeDoor();
    sendStatus("close_done");
  }
}

void sendStatus(String status) {
  client.publish(mqtt_topic_status, status.c_str());
  sendDebug("상태 전송 : " + status);
}

void openDoor() {
  sendDebug("문 열기 시작");
  for (int angle = 10; angle <= 100; angle++) {
    microServo.write(angle);
    delay(30);
  }
  sendDebug("문 열기 완료");
}

void closeDoor() {
  sendDebug("문 닫기 시작");
  for (int angle = 100; angle >= 10; angle--) {
    microServo.write(angle);
    delay(30);
  }
  sendDebug("문 닫기 완료");
}

void sendDebug(String msg) {
  Serial.println(msg);
  client.publish(mqtt_topic_debug, msg.c_str());
}
