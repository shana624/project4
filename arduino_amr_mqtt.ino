#include <Adafruit_PWMServoDriver.h>
#include <Wire.h>
#include <WiFiS3.h>
#include <PubSubClient.h>  // MQTT 라이브러리

// ===================== 하드웨어 및 핀 설정 =====================

// PCA9685 보드 주소 (서보 드라이버)
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

// 서보 모터 핀 할당
#define servoFront 0
#define servoBack  1

// 서보 모터의 PWM 값 범위 (각도에 따른 값 조정)
#define servoMIN 150
#define servoMID 375
#define servoMAX 600

// 스위치(버튼) 핀 정의 (기본 HIGH, 누르면 LOW)
#define AMR_OUTPUT_1 2
#define AMR_OUTPUT_2 3
#define AMR_OUTPUT_3 4
#define AMR_OUTPUT_4 5
#define AMR_OUTPUT_5 6
#define AMR_OUTPUT_6 7

// 디지털 출력으로 제어할 입력 핀 (실제 모듈 제어용)
#define AMR_INPUT_1 8
#define AMR_INPUT_2 9
#define AMR_INPUT_3 10
#define AMR_INPUT_4 11

// 컨베이어 제어 핀
#define CONVEYOR_PIN 13

// ===================== WiFi 및 MQTT 설정 =====================

// Wi-Fi 네트워크 정보
const char* ssid = "SF_MAIN_2G";
const char* password = "";

// MQTT 브로커 정보 및 토픽 설정
const char* mqtt_server = "192.168.0.188";
const int mqtt_port = 1883;
const char* mqtt_topic_cmd = "/arduino/amr/cmd";     // 명령 수신용 토픽
const char* mqtt_topic_status = "/arduino/amr/status"; // 상태 발행용 토픽
const char* mqtt_topic_debug = "/arduino/amr/debug";   // 디버그 메시지용 토픽

// WiFi 및 MQTT 클라이언트 객체 생성
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// ===================== 디버그 메시지 발행 함수 =====================

// 디버그 메시지를 Serial 모니터와 MQTT로 동시에 출력
void debugPrint(String message) {
  Serial.println(message);
  mqttClient.publish(mqtt_topic_debug, message.c_str());
}

// ===================== 명령어 매핑 테이블 =====================
// 수신한 MQTT 명령어에 따라 제어할 핀 번호 매핑 (필요에 따라 추가 가능)
struct MqttCommand {
  const char* command;  // 인식할 명령어
  int pin;              // 제어할 핀 번호 (AMR_INPUT_x)
};

// 예시로 "start", "opened", "package" 등의 명령을 매핑합니다.
const MqttCommand commandTable[] = {
  {"start",   AMR_INPUT_1},  // "start" 명령 → AMR_INPUT_1 제어
  {"open_done",  AMR_INPUT_2},  // "opened" 명령 → AMR_INPUT_2 제어
  {"package", AMR_INPUT_3}   // "package" 명령 → AMR_INPUT_3 제어 (서보 동작 후)
};

// ===================== 디바운스 및 타이머 변수 =====================

// 버튼 디바운스 처리 변수
bool stableButtonState[6] = {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}; // 안정된 상태 저장
bool lastRawReading[6]    = {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}; // 마지막 읽은 값
unsigned long lastDebounceTime[6] = {0, 0, 0, 0, 0, 0};
const unsigned long debounceDelay = 50;  // 50ms

// AMR 입력 핀 제어용 타이머 변수 (핀 자동 리셋)
unsigned long inputTimer[4] = {0};
bool inputActive[4] = {false};

// ===================== WiFi 및 MQTT 연결 함수 =====================

// WiFi 연결 함수
void connectWiFi() {
  debugPrint("WiFi 네트워크에 연결 중: " + String(ssid));
  
  WiFi.begin(ssid);

  // WiFi 연결 상태를 대기
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    debugPrint(".");
  }
  debugPrint("WiFi 연결 완료!");
  debugPrint("IP 주소: " + WiFi.localIP().toString());
}

// MQTT 브로커에 연결하는 함수
void connectMQTT() {
  // MQTT 연결이 되어 있지 않으면 반복하여 연결 시도
  while (!mqttClient.connected()) {
    debugPrint("MQTT 브로커에 연결 중...");
    // 고유 클라이언트 ID 사용 (여기서는 "ArduinoR4Client")
    if (mqttClient.connect("ArduinoR4Client")) {
      debugPrint("MQTT 연결 완료!");
      // 명령 수신을 위해 구독 설정
      mqttClient.subscribe(mqtt_topic_cmd);
      // 연결 상태를 발행으로 알림
      mqttClient.publish(mqtt_topic_status, "MQTT 연결 완료");
    } else {
      debugPrint("MQTT 연결 실패, 상태 코드=" + String(mqttClient.state()) + " 5초 후 재시도합니다.");
      delay(5000);
    }
  }
}

// ===================== MQTT 메시지 수신 콜백 함수 =====================

// 이 함수는 구독한 토픽(mqtt_topic_cmd)으로부터 메시지가 도착하면 호출됩니다.
void callback(char* topic, byte* payload, unsigned int length) {
  // payload를 String으로 변환
  String command = "";
  for (unsigned int i = 0; i < length; i++) {
    command += (char)payload[i];
  }
  
  debugPrint("MQTT로부터 명령 수신: " + command);

  // 명령에 따른 동작 처리
  if (command == "package") {
    pwm.setPWM(servoFront, 0, servoMID);  // 서보를 중간 위치로 이동
    debugPrint("servoFront를 중간 위치로 이동 (package 명령)");
    delay(500); // 서보 이동 대기
    digitalWrite(AMR_INPUT_3, HIGH);
    inputActive[AMR_INPUT_3 - 8] = true;
    inputTimer[AMR_INPUT_3 - 8] = millis();
    mqttClient.publish(mqtt_topic_status, "AMR_INPUT_3 HIGH 설정 완료");
  } else {
    // 매핑 테이블을 확인하여 등록된 명령어에 대해 처리
    for (size_t i = 0; i < sizeof(commandTable) / sizeof(commandTable[0]); i++) {
      if (command == commandTable[i].command && command != String("package")) {
        int pin = commandTable[i].pin;
        digitalWrite(pin, HIGH);
        inputActive[pin - 8] = true;
        inputTimer[pin - 8] = millis();
        debugPrint("수신 명령: " + command + " → AMR_INPUT_" + String(pin - 7));
        mqttClient.publish(mqtt_topic_status, ("수신 명령: " + command).c_str());
        break;
      }
    }
  }
}

// ===================== 버튼 디바운스 및 동작 처리 함수 =====================

// 각 버튼의 상태를 디바운스 처리하여 변화 시 동작을 수행하는 함수
void checkButton(int index, int pin) {
  bool raw = digitalRead(pin);
  
  if (raw != lastRawReading[index]) {
    lastDebounceTime[index] = millis();
  }
  lastRawReading[index] = raw;
  
  if ((millis() - lastDebounceTime[index]) > debounceDelay) {
    if (raw != stableButtonState[index]) {
      stableButtonState[index] = raw;
      
      if (index == 0) {
        debugPrint("차단기 이벤트");
        mqttClient.publish(mqtt_topic_status, "door");
      } else {
        if (raw == LOW) {
          switch (index) {
            case 1:
              pwm.setPWM(servoFront, 0, servoMID);
              debugPrint("ABB앞에 도착");
              mqttClient.publish(mqtt_topic_status, "ready");
              break;
            case 2:
              pwm.setPWM(servoFront, 0, 330);
              digitalWrite(CONVEYOR_PIN, HIGH);
              debugPrint("ABB 공정위치 도착");
              mqttClient.publish(mqtt_topic_status, "arrived");
              delay(10);
              break;
            case 3:
              pwm.setPWM(servoFront, 0, servoMIN);
              digitalWrite(CONVEYOR_PIN, LOW);
              debugPrint("후진완료 서보,컨베이어 정지");
              mqttClient.publish(mqtt_topic_status, "");
              break;
            case 4:
              pwm.setPWM(servoBack, 0, 440);
              debugPrint("배송장소 도착");
              mqttClient.publish(mqtt_topic_status, "drop");
              // 기존 isServoActive 대신 conveyorControl 함수 호출
              conveyorControl(true);
              break;
            case 5:
              debugPrint("도킹스테이션 복귀");
              mqttClient.publish(mqtt_topic_status, "home");
              break;
          }
        }
      }
    }
  }
}

// ===================== AMR 입력 핀 자동 해제 함수 =====================

void checkInputPins() {
  unsigned long current = millis();
  for (int i = 0; i < 4; i++) {
    if (inputActive[i] && (current - inputTimer[i] >= 500)) {
      digitalWrite(8 + i, LOW);
      inputActive[i] = false;
      debugPrint("AMR_INPUT_" + String(i + 1) + " 자동 해제");
    }
  }
}

// ===================== 컨베이어 동작 제어 함수 =====================
// 인자로 true 전달 시 5초간 컨베이어 동작 후 서보 복귀, 컨베이어 정지,
// 4번 핀을 잠깐 활성화한 후 MQTT 메시지 발행
void conveyorControl(bool activate) {
  if (activate) {
    debugPrint("컨베이어 5초 동작 시작");
    digitalWrite(CONVEYOR_PIN, HIGH);
    delay(7000);  // 7초 동작
    digitalWrite(CONVEYOR_PIN, LOW);
    debugPrint("컨베이어 정지 및 서보 복귀 시작");
    
    // 서보 복귀
    pwm.setPWM(servoBack, 0, servoMIN);
    delay(500); // 서보 복귀 대기

    // 4번 핀 잠깐 활성화 (펄스)
    digitalWrite(AMR_INPUT_4, HIGH);
    delay(100);
    digitalWrite(AMR_INPUT_4, LOW);

    debugPrint("컨베이어 시퀀스 완료, 'home' 상태 MQTT 발행");
    mqttClient.publish(mqtt_topic_status, "home");
  }
}

// ===================== setup() 함수 =====================

void setup() {
  Serial.begin(115200);

  // PCA9685 초기화 (서보 드라이버)
  pwm.begin();
  pwm.setPWMFreq(60);

  // 버튼 입력 핀을 INPUT_PULLUP 모드로 설정
  pinMode(AMR_OUTPUT_1, INPUT_PULLUP);
  pinMode(AMR_OUTPUT_2, INPUT_PULLUP);
  pinMode(AMR_OUTPUT_3, INPUT_PULLUP);
  pinMode(AMR_OUTPUT_4, INPUT_PULLUP);
  pinMode(AMR_OUTPUT_5, INPUT_PULLUP);
  pinMode(AMR_OUTPUT_6, INPUT_PULLUP);

  // AMR 제어용 출력 핀 설정 및 초기 LOW 상태
  pinMode(AMR_INPUT_1, OUTPUT);
  pinMode(AMR_INPUT_2, OUTPUT);
  pinMode(AMR_INPUT_3, OUTPUT);
  pinMode(AMR_INPUT_4, OUTPUT);
  digitalWrite(AMR_INPUT_1, LOW);
  digitalWrite(AMR_INPUT_2, LOW);
  digitalWrite(AMR_INPUT_3, LOW);
  digitalWrite(AMR_INPUT_4, LOW);

  // 컨베이어 핀 설정 및 초기 LOW 상태
  pinMode(CONVEYOR_PIN, OUTPUT);
  digitalWrite(CONVEYOR_PIN, LOW);

  // PCA9685 I2C 연결 테스트
  debugPrint("PCA9685 초기화 중...");
  Wire.beginTransmission(0x40);
  if (Wire.endTransmission() == 0) {
    debugPrint("PCA9685 연결 확인됨!");
  }

  // 초기 서보 위치 설정
  pwm.setPWM(servoFront, 0, servoMIN);
  pwm.setPWM(servoBack, 0, servoMIN);

  // WiFi 연결 및 MQTT 브로커 연결 설정
  connectWiFi();
  mqttClient.setServer(mqtt_server, mqtt_port);
  mqttClient.setCallback(callback);
  connectMQTT();

  debugPrint("명령 대기중...");
}

// ===================== loop() 함수 =====================

void loop() {
  if (!mqttClient.connected()) {
    connectMQTT();
  }
  mqttClient.loop();

  // 각 버튼의 상태 확인 (디바운스 처리 포함)
  checkButton(0, AMR_OUTPUT_1);
  checkButton(1, AMR_OUTPUT_2);
  checkButton(2, AMR_OUTPUT_3);
  checkButton(3, AMR_OUTPUT_4);
  checkButton(4, AMR_OUTPUT_5);
  checkButton(5, AMR_OUTPUT_6);
  
  // AMR 입력 핀 자동 해제 처리 (500ms 후 LOW로 복귀)
  checkInputPins();
}
