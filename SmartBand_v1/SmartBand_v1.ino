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
BLEServer* pServer = NULL;
BLECharacteristic* pHeartRateCharacteristic = NULL;
BLECharacteristic* pCommandCharacteristic = NULL;
volatile bool deviceConnected = false;

// 심박 전송 타이머
unsigned long previousMillis = 0;
const unsigned long interval = 1000;

// 심박 센서 변수
MAX30105 particleSensor;
const byte RATE_SIZE = 4;
int rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute = 0;
int beatAvg = 0;
byte validRateCount = 0;
bool fingerDetected = false;

const long FINGER_OFF_THRESHOLD = 30000;
const long FINGER_ON_THRESHOLD = 70000;

// BPM 필터 기준
const int MIN_BPM = 45;
const int MAX_BPM = 160;
const int MAX_BPM_JUMP = 30;
const long MIN_DELTA = 350;
const long MAX_DELTA = 1500;

// 진동 모터 제어 변수
bool isVibrating = false;
unsigned long vibrationStartMillis = 0;
const unsigned long vibrationDuration = 1000;

const bool VIBRATION_ACTIVE_LOW = false;

void vibrationOn() {
  if (VIBRATION_ACTIVE_LOW) {
    digitalWrite(VIBRATION_PIN, LOW);
  } else {
    digitalWrite(VIBRATION_PIN, HIGH);
  }
}

void vibrationOff() {
  if (VIBRATION_ACTIVE_LOW) {
    digitalWrite(VIBRATION_PIN, HIGH);
  } else {
    digitalWrite(VIBRATION_PIN, LOW);
  }
}

void startVibration() {
  vibrationOn();
  vibrationStartMillis = millis();
  isVibrating = true;
  Serial.println("진동 시작");
}

void resetHeartRateState() {
  beatAvg = 0;
  beatsPerMinute = 0;
  validRateCount = 0;
  rateSpot = 0;
  lastBeat = 0;

  for (byte i = 0; i < RATE_SIZE; i++) {
    rates[i] = 0;
  }
}

class MomfitCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) override {
    deviceConnected = true;
    Serial.println("프론트엔드와 BLE 연결 성공");
  }

  void onDisconnect(BLEServer* pServer) override {
    deviceConnected = false;
    Serial.println("BLE 연결 끊어짐. 다시 광고 시작");
    pServer->getAdvertising()->start();
  }
};

class CommandCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) override {
    String value = pCharacteristic->getValue();

    if (value.length() == 0) return;

    Serial.print("받은 명령: ");
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

void setup() {
  Serial.begin(115200);

  pinMode(VIBRATION_PIN, OUTPUT);
  vibrationOff();

  delay(200);

  Serial.println("센서 초기화 중...");
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("센서 연결 실패! 배선을 확인하세요.");
    while (1);
  }

  particleSensor.setup(20, 4, 2, 100, 411, 4096);
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeGreen(0);

  BLEDevice::init("MOMFIT_Wearable");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MomfitCallbacks());

  BLEService* pService = pServer->createService(SERVICE_UUID);

  pHeartRateCharacteristic = pService->createCharacteristic(
    HEART_RATE_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pHeartRateCharacteristic->addDescriptor(new BLE2902());

  pCommandCharacteristic = pService->createCharacteristic(
    COMMAND_UUID,
    BLECharacteristic::PROPERTY_WRITE
  );
  pCommandCharacteristic->setCallbacks(new CommandCallbacks());

  pService->start();
  pServer->getAdvertising()->start();

  Serial.println("BLE 서버 시작 완료. 앱에서 연결해주세요!");
}

void loop() {
  long irValue = particleSensor.getIR();

  // 1. 손가락 감지 / 이탈 상태 처리
  if (irValue < FINGER_OFF_THRESHOLD) {
    if (fingerDetected) {
      Serial.println("손가락 이탈 - 측정 상태 초기화");
      resetHeartRateState();
      fingerDetected = false;
    }
  } else if (irValue > FINGER_ON_THRESHOLD) {
    if (!fingerDetected) {
      Serial.println("손가락 감지 - 새 측정 시작");
      resetHeartRateState();
      fingerDetected = true;
    }
  }

  // 2. 심박 감지
  if (fingerDetected && checkForBeat(irValue) == true) {
    unsigned long now = millis();

    if (lastBeat == 0) {
      lastBeat = now;
      Serial.println("첫 박동 감지 - 워밍업");
    } else {
      long delta = now - lastBeat;
      lastBeat = now;

      // 1차 필터: 박동 간격 이상값 제거
      if (delta < MIN_DELTA || delta > MAX_DELTA) {
        Serial.print("delta 이상값 제외: ");
        Serial.println(delta);
        return;
      }

      beatsPerMinute = 60.0 / (delta / 1000.0);

      // 2차 필터: BPM 범위 이상값 제거
      if (beatsPerMinute < MIN_BPM || beatsPerMinute > MAX_BPM) {
        Serial.print("BPM 범위 이상값 제외: ");
        Serial.println(beatsPerMinute);
        return;
      }

      // 3차 필터: 평균 대비 너무 튀는 값 제거
      if (beatAvg > 0 && abs((int)beatsPerMinute - beatAvg) > MAX_BPM_JUMP) {
        Serial.print("평균 대비 튐 값 제외: ");
        Serial.println(beatsPerMinute);
        return;
      }

      rates[rateSpot++] = (int)beatsPerMinute;
      rateSpot %= RATE_SIZE;

      if (validRateCount < RATE_SIZE) {
        validRateCount++;
      }

      if (validRateCount == RATE_SIZE) {
        int sum = 0;
        for (byte x = 0; x < RATE_SIZE; x++) {
          sum += rates[x];
        }
        beatAvg = sum / RATE_SIZE;
      } else {
        beatAvg = 0;
      }

      Serial.print("현재 BPM: ");
      Serial.print((int)beatsPerMinute);
      Serial.print(" / 평균 BPM: ");
      Serial.println(beatAvg);
    }
  }

  // 3. 1초마다 프론트로 전송
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    const unsigned long beatTimeout = 5000;

    if (fingerDetected && lastBeat != 0 && (millis() - lastBeat > beatTimeout)) {
      Serial.println("박동 타임아웃! 찌꺼기 데이터 강제 초기화 (재측정 시작)");
      resetHeartRateState();
    }

    if (deviceConnected) {
      String bpmString = String(beatAvg);
      pHeartRateCharacteristic->setValue(bpmString.c_str());
      pHeartRateCharacteristic->notify();

      Serial.print("앱으로 전송한 BPM: ");
      Serial.println(bpmString);
    }
  }

  // 4. 진동 자동 종료
  if (isVibrating && millis() - vibrationStartMillis >= vibrationDuration) {
    vibrationOff();
    isVibrating = false;
    Serial.println("진동 자동 종료");
  }
}