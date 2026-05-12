/*
  MOMFIT SmartBand

  기준:
  - 심박 측정은 SparkFun Example5_HeartRate의 PBA 알고리즘과 beatAvg 계산 방식을 따른다.
  - 1초마다 프론트로 평균 심박수(beatAvg)를 전송한다.
  - 손가락 미착용 또는 평균값 준비 전에는 0을 전송한다.
  - 프론트에서 COMMAND_UUID로 "1"을 보내면 1초 동안 진동한다.
*/

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

// 핀 설정
#define VIBRATION_PIN 25

// BLE UUID
#define SERVICE_UUID        "19b10000-e8f2-537e-4f6c-d104768a1214"
#define HEART_RATE_UUID     "19b10001-e8f2-537e-4f6c-d104768a1214"
#define COMMAND_UUID        "19b10002-e8f2-537e-4f6c-d104768a1214"

// BLE 객체
BLEServer* pServer = nullptr;
BLECharacteristic* pHeartRateCharacteristic = nullptr;
BLECharacteristic* pCommandCharacteristic = nullptr;
volatile bool deviceConnected = false;

// MAX30105 센서
MAX30105 particleSensor;

// Sample5_HeartRate 기준 심박 평균 변수
const byte RATE_SIZE = 4;     // 최근 4개 BPM 평균
byte rates[RATE_SIZE];        // byte 배열 사용
byte rateSpot = 0;
long lastBeat = 0;

float beatsPerMinute = 0;
int beatAvg = 0;

// 전송 주기: 1초마다 프론트로 값 전송
unsigned long previousSendMillis = 0;
const unsigned long SEND_INTERVAL = 1000;

// 손가락 감지 기준
const long NO_FINGER_THRESHOLD = 50000;

// 진동 제어
bool isVibrating = false;
unsigned long vibrationStartMillis = 0;
const unsigned long VIBRATION_DURATION = 1000;

void vibrationOn() {
  digitalWrite(VIBRATION_PIN, HIGH);
}

void vibrationOff() {
  digitalWrite(VIBRATION_PIN, LOW);
}

void startVibration() {
  vibrationOn();
  vibrationStartMillis = millis();
  isVibrating = true;
  Serial.println("진동 시작: 1초 동안 동작");
}

void resetHeartRateValues() {
  beatsPerMinute = 0;
  beatAvg = 0;
  lastBeat = 0;
  rateSpot = 0;

  for (byte i = 0; i < RATE_SIZE; i++) {
    rates[i] = 0;
  }
}

void notifyHeartRate(int bpm) {
  if (!deviceConnected || pHeartRateCharacteristic == nullptr) return;

  String bpmString = String(bpm);
  pHeartRateCharacteristic->setValue(bpmString.c_str());
  pHeartRateCharacteristic->notify();

  Serial.print("프론트로 전송한 BPM: ");
  Serial.println(bpmString);
}

class MomfitServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    deviceConnected = true;
    Serial.println("프론트 연결 성공");
  }

  void onDisconnect(BLEServer* pServer) override {
    deviceConnected = false;
    Serial.println("프론트 연결 끊김. 다시 연결 대기 시작");
    pServer->getAdvertising()->start();
    Serial.println("프론트 연결 대기 중...");
  }
};

class CommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) override {
    String value = pCharacteristic->getValue();
    if (value.length() == 0) return;

    Serial.print("프론트에서 받은 진동 명령: ");
    Serial.println(value);

    if (value == "1") {
      startVibration();
    } else if (value == "0") {
      vibrationOff();
      isVibrating = false;
      Serial.println("진동 강제 종료");
    }
  }
};

void setupBLE() {
  BLEDevice::init("MOMFIT_Wearable");

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MomfitServerCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);

  pHeartRateCharacteristic = pService->createCharacteristic(
    HEART_RATE_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY
  );
  pHeartRateCharacteristic->addDescriptor(new BLE2902());
  pHeartRateCharacteristic->setValue("0");

  pCommandCharacteristic = pService->createCharacteristic(
    COMMAND_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  pCommandCharacteristic->setCallbacks(new CommandCallbacks());

  pService->start();

  BLEAdvertising* pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMinPreferred(0x12);

  BLEDevice::startAdvertising();
  Serial.println("BLE 서버 시작 완료");
  Serial.println("프론트 연결 대기 중...");
}

void setupSensor() {
  Serial.println("MAX30105 센서 초기화 중...");

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30105를 찾을 수 없습니다. 배선/전원을 확인하세요.");
    while (1);
  }

  Serial.println("센서 초기화 완료. 손가락을 센서에 안정적으로 올려주세요.");

  // 기본 설정
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeGreen(0);

  resetHeartRateValues();
}

void setup() {
  Serial.begin(115200);
  Serial.println("MOMFIT SmartBand 시작");

  pinMode(VIBRATION_PIN, OUTPUT);
  vibrationOff();

  setupSensor();
  setupBLE();
}

void loop() {
  long irValue = particleSensor.getIR();

  // Sample5_HeartRate 예제의 심박 계산 방식 사용
  if (checkForBeat(irValue) == true) {
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60 / (delta / 1000.0);

    if (beatsPerMinute < 255 && beatsPerMinute > 20) {
      rates[rateSpot++] = (byte)beatsPerMinute;
      rateSpot %= RATE_SIZE;

      beatAvg = 0;
      for (byte x = 0; x < RATE_SIZE; x++) {
        beatAvg += rates[x];
      }
      beatAvg /= RATE_SIZE;
    }
  }

  // 시리얼 모니터 확인용 로그
  Serial.print("IR=");
  Serial.print(irValue);
  Serial.print(", BPM=");
  Serial.print(beatsPerMinute);
  Serial.print(", Avg BPM=");
  Serial.print(beatAvg);

  if (irValue < NO_FINGER_THRESHOLD) {
    Serial.print(" No finger?");
  }
  Serial.println();

  // 1초마다 프론트로 평균 BPM 전송
  unsigned long currentMillis = millis();
  if (currentMillis - previousSendMillis >= SEND_INTERVAL) {
    previousSendMillis = currentMillis;

    if (irValue < NO_FINGER_THRESHOLD) {
      resetHeartRateValues();
      notifyHeartRate(0);
    } else {
      notifyHeartRate(beatAvg);
    }
  }

  // 진동 1초 후 자동 종료
  if (isVibrating && millis() - vibrationStartMillis >= VIBRATION_DURATION) {
    vibrationOff();
    isVibrating = false;
    Serial.println("진동 자동 종료");
  }
}
