#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

// 프론트엔드와 맞출 블루투스 서비스/특성 UUID
#define SERVICE_UUID        "19b10000-e8f2-537e-4f6c-d104768a1214"
#define CHARACTERISTIC_UUID "19b10001-e8f2-537e-4f6c-d104768a1214"

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
volatile bool deviceConnected = false;

// 1초 타이머용 변수
unsigned long previousMillis = 0;
const long interval = 1000; 

// --- MAX30105 심박 센서 변수 ---
MAX30105 particleSensor;
const byte RATE_SIZE = 4; // 평균을 낼 측정값의 개수
int rates[RATE_SIZE];    // 심박수 저장 배열
byte rateSpot = 0;
long lastBeat = 0;        // 마지막 맥박 시간
float beatsPerMinute = 0;
int beatAvg = 0;              // 최종 평균 심박수
byte validRateCount = 0;  // 초기 평균 계산용

// 블루투스 연결 상태 감지 콜백
class MomfitCallbacks: public BLEServerCallbacks {
  // 프론트가 BLE 연결 성공했을 때 호출
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("프론트엔드(웹)와 블루투스 연결 성공");
    };
    // 연결 끊겼을 때 호출
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("블루투스 연결 끊어짐. 다시 연결을 기다립니다...");
      pServer->getAdvertising()->start();
    }
};

// 초기화 함수
void setup() {
  Serial.begin(115200);

  // 심박 센서 초기화
  Serial.println("센서 초기화 중...");
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) { 
    Serial.println("MAX30105 센서를 찾을 수 없습니다. 배선을 확인해주세요.");
    while (1); 
  }
  
  particleSensor.setup(); 
  particleSensor.setPulseAmplitudeRed(0x0A); // 센서 작동을 표시하기 위해 Red LED 켜기
  particleSensor.setPulseAmplitudeGreen(0);  // Green LED 끄기

  // 블루투스(BLE) 초기화
  BLEDevice::init("MOMFIT_Wearable");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MomfitCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_NOTIFY
                    );
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();
  pServer->getAdvertising()->start();
  
  Serial.println("블루투스 신호 송출 시작. 프론트엔드에서 기기를 찾아보세요.");
}

// 실시간 측정 + 전송 메인 루프
void loop() {
  // -----------------------------------------------------------
  // 센서 데이터 실시간 읽기 (Non-blocking: 지연 없이 계속 실행)
  // -----------------------------------------------------------
  long irValue = particleSensor.getIR(); 
  
  // 맥박 감지
  if (checkForBeat(irValue) == true) {
    // bpm 계산 
    long delta = millis() - lastBeat; 
    lastBeat = millis(); 

    beatsPerMinute = 60 / (delta / 1000.0); 

    // 측정값이 정상 범위(40~255)일 때만 배열에 저장
    if (beatsPerMinute < 255 && beatsPerMinute > 40) { 
      rates[rateSpot++] = (int)beatsPerMinute;
      rateSpot %= RATE_SIZE;

      if (validRateCount < RATE_SIZE) {
        validRateCount++;
      }

      beatAvg = 0;
      for (byte x = 0; x < validRateCount; x++) {
      beatAvg += rates[x];
      }
      beatAvg /= validRateCount;
    }
  }

  // -----------------------------------------------------------
  // 1초마다 BLE로 프론트엔드에 전송
  // -----------------------------------------------------------
  unsigned long currentMillis = millis();
  
  // 1초마다 BLE 전송
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // 손가락이 센서에서 떨어졌을 때의 예외 처리 (IR 값이 낮으면 0으로 초기화)
    if (irValue < 50000) { 
      beatAvg = 0;
      validRateCount = 0;
    }

    if (deviceConnected) {
      String bpmString = String(beatAvg);
      // notify 전송  
      pCharacteristic->setValue(bpmString.c_str());
      pCharacteristic->notify(); 
      
      Serial.print("블루투스로 전송한 평균 심박수: ");
      Serial.println(bpmString);
    }
  }
}