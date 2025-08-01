#include <Wire.h>
#include <SPI.h>
#include <sparkFun_BMI270_Arduino_Library.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <DFRobot_OzoneSensor.h>
#include <Adafruit_MLX90393.h>
#include <Adafruit_MCP9600.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <PMS.h>
#include <HardwareSerial.h>
#include <SD.h>

// I2C 주소 정의
const uint8_t BMI270_ADDR    = 0x68; //실제주소 확인필요
const uint8_t BME688_ADDR    = 0x76;
const uint8_t MLX90393_ADDR  = 0x0C;
const uint8_t MCP9600_ADDR   = 0x60;
const uint8_t OZONE_ADDR     = 0x70;
const uint8_t GDK101_ADDR    = 0x18;
const int SD_CS_PIN = 5;

// DS18B20 데이터핀
#define DS18B20_PIN 4

// PMS3003 시리얼 핀
#define PMS_RX_PIN 16
#define PMS_TX_PIN 17

// GDK101 I2C 통신 사용

#define INTERNAL_TEMP_LIMIT_LOW -45.0 // 예시 임계치(℃), 필요시 조정 (영하 45도 이하)
unsigned long lastSensorRestart = 0;
const unsigned long restartInterval = 1000; // 1초마다 재시도

const unsigned long fastInterval = 100;   // 내부 온도, 외부 온습도, 열전대: 10Hz
const unsigned long envInterval  = 200;   // 외부 온습도, 열전대: 5Hz
const unsigned long slowInterval = 1000;  // 나머지 센서: 1Hz
unsigned long lastFastUpdate = 0;
unsigned long lastEnvUpdate = 0;
unsigned long lastSlowUpdate = 0;

bool sensorsActive = true; // 센서 측정 활성화 플래그

// GDK101 I2C 통신 함수
void Gamma_Mod_Read(int cmd) {
  Wire.beginTransmission(GDK101_ADDR);
  Wire.write(cmd);
  Wire.endTransmission();
  delay(10);
  Wire.requestFrom(GDK101_ADDR, 2); // 항상 2바이트 반환
  for(byte i = 0; i < 2 && Wire.available(); i++) gdkBuffer[i] = Wire.read();
}

// 센서 객체 선언
Adafruit_BME680 bme;
DFRobot_OzoneSensor ozone;
Adafruit_MLX90393 mlx;
Adafruit_MCP9600 mcp;
OneWire oneWire(DS18B20_PIN);
DallasTemperature sensors(&oneWire);
HardwareSerial pmsSerial(1); // UART1
PMS pms(pmsSerial);
// GDK101 I2C 통신용 변수
byte gdkBuffer[2] = {0, 0};
BMI270 bmi270;

// 센서 데이터 구조체
struct SensorData {
  float ds18b20;
  float bme_temp;
  float bme_hum;
  float bme_press;
  float mcp_temp;
  float mlx_x, mlx_y, mlx_z;
  float bmi_x, bmi_y, bmi_z;
  uint16_t pms_1_0, pms_2_5, pms_10;
  float gdk101;
  float ozone;
};

void setup() {
  Serial.begin(115200);
  Wire.begin();
  pmsSerial.begin(9600, SERIAL_8N1, PMS_RX_PIN, PMS_TX_PIN);

  if (!SD.begin(SD_CS_PIN)) {
    Serial.println("SD 카드 초기화 실패");
  }

  ModuleInit();
}

void loop() {
  unsigned long now = millis();
  static SensorData data;

  // 1. 내부 온도(DS18B20) - 10Hz, 중단 없음
  if (now - lastFastUpdate >= fastInterval) {
    lastFastUpdate = now;
    sensors.requestTemperatures();
    float temp = sensors.getTempCByIndex(0);
    if (temp == DEVICE_DISCONNECTED_C || isnan(temp)) {
      Serial.println("[경고] 내부 온도 센서 오류! 재시도 중...");
      sensors.begin();
      data.ds18b20 = -999;
    } else {
      data.ds18b20 = temp;
    }
  }

  // 2. 외부 온습도기압(BME688), 열전대(MCP9600) - 5Hz, 중단 없음
  if (now - lastEnvUpdate >= envInterval) {
    lastEnvUpdate = now;
    if (!bme.performReading()) {
      data.bme_temp = data.bme_hum = data.bme_press = -999;
      bme.begin(BME688_ADDR);
    } else {
      data.bme_temp  = bme.temperature;
      data.bme_hum   = bme.humidity;
      data.bme_press = bme.pressure / 100.0;
    }
    float mcpTemp = mcp.readThermocouple();
    if (isnan(mcpTemp)) {
      data.mcp_temp = -999;
      mcp.begin(MCP9600_ADDR);
    } else {
      data.mcp_temp = mcpTemp;
    }
  }

  // 3. 나머지 센서 - 1Hz, 오류 시 재시도 및 오류값 기록
  if (now - lastSlowUpdate >= slowInterval) {
    lastSlowUpdate = now;

    // 자기장(MLX90393)
    float x, y, z;
    if (mlx.readData(&x, &y, &z)) {
      data.mlx_x = x; 
      data.mlx_y = y; 
      data.mlx_z = z;
    } else {
      data.mlx_x = data.mlx_y = data.mlx_z = -999;
    }

    // 가속도(BMI270)
    bmi270.getSensorData(); // 센서 데이터 업데이트
    data.bmi_x = bmi270.data.accelX;
    data.bmi_y = bmi270.data.accelY;
    data.bmi_z = bmi270.data.accelZ;

    // 미세먼지(PMS3003)
    PMS::DATA pmsData;
    if (pms.read(pmsData)) {
      data.pms_1_0 = pmsData.PM_AE_UG_1_0;
      data.pms_2_5 = pmsData.PM_AE_UG_2_5;
      data.pms_10  = pmsData.PM_AE_UG_10_0;
    } else {
      data.pms_1_0 = data.pms_2_5 = data.pms_10 = 0;
    }

    // 공기질(GDK101) - I2C 통신
    Gamma_Mod_Read(0xB3); // 1분 평균 읽기
    data.gdk101 = gdkBuffer[0] + gdkBuffer[1] / 100.0; // uSv/h 단위

    // 오존(DFRobot_OzoneSensor)
    float ozoneVal = ozone.readOzoneData();
    if (isnan(ozoneVal)) {
      data.ozone = -999;
      ozone.begin();
    } else {
      data.ozone = ozoneVal;
    }
  }

  // 이상 상황 기록/알림 (내/외부 온도 차이 급격 변화)
  if (abs(data.ds18b20 - data.bme_temp) > 20.0) {
    Serial.println("[경고] 내부/외부 온도 차이 급격! 즉시 로그 기록");
    logToSD(data);
  }

  printSensorData(data);
  logToSD(data);

  Serial.println("--------------------------------------------------");
  delay(10); // 최소 loop 주기
}

void printSensorData(const SensorData& d) {
  Serial.print("[DS18B20] 내부온도: "); Serial.println(d.ds18b20);
  Serial.print("[BME688] 외부온도: "); Serial.print(d.bme_temp);
  Serial.print(" ℃, 습도: "); Serial.print(d.bme_hum);
  Serial.print(" %, 기압: "); Serial.println(d.bme_press);
  Serial.print("[MCP9600] 열전대온도: "); Serial.println(d.mcp_temp);
  Serial.print("[MLX90393] X: "); Serial.print(d.mlx_x);
  Serial.print(" uT, Y: "); Serial.print(d.mlx_y);
  Serial.print(" uT, Z: "); Serial.println(d.mlx_z);
  Serial.print("[BMI270] 가속도 X: "); Serial.print(d.bmi_x);
  Serial.print(", Y: "); Serial.print(d.bmi_y);
  Serial.print(", Z: "); Serial.println(d.bmi_z);
  Serial.print("[PMS3003] PM1.0: "); Serial.print(d.pms_1_0);
  Serial.print(" ug/m3, PM2.5: "); Serial.print(d.pms_2_5);
  Serial.print(" ug/m3, PM10: "); Serial.println(d.pms_10);
  Serial.print("[GDK101] 방사선: "); Serial.print(d.gdk101); Serial.println(" uSv/h");
  Serial.print("[Ozone] 오존: "); Serial.println(d.ozone);
}

void logToSD(const SensorData& d) {
  const char* fileName = "data.csv";
  bool newFile = !SD.exists(fileName);
  File myFile = SD.open(fileName, FILE_WRITE);

  if (myFile) {
    if (newFile) {
      myFile.println(F("Millis,DS18B20,BME_Temp,BME_Hum,BME_Press,MCP_Temp,MLX_X,MLX_Y,MLX_Z,BMI270_X,BMI270_Y,BMI270_Z,PMS1.0,PMS2.5,PMS10,GDK101,Ozone"));
    }
    myFile.print(millis()); myFile.print(",");
    myFile.print(d.ds18b20); myFile.print(",");
    myFile.print(d.bme_temp); myFile.print(",");
    myFile.print(d.bme_hum); myFile.print(",");
    myFile.print(d.bme_press); myFile.print(",");
    myFile.print(d.mcp_temp); myFile.print(",");
    myFile.print(d.mlx_x); myFile.print(",");
    myFile.print(d.mlx_y); myFile.print(",");
    myFile.print(d.mlx_z); myFile.print(",");
    myFile.print(d.bmi_x); myFile.print(",");
    myFile.print(d.bmi_y); myFile.print(",");
    myFile.print(d.bmi_z); myFile.print(",");
    myFile.print(d.pms_1_0); myFile.print(",");
    myFile.print(d.pms_2_5); myFile.print(",");
    myFile.print(d.pms_10); myFile.print(",");
    myFile.print(d.gdk101); myFile.print(",");
    myFile.print(d.ozone);
    myFile.println();
    myFile.close();
  } else {
    Serial.println("⚠ Error: Failed to open log file.");
  }
}

void ModuleInit() {
  if (!bme.begin(BME688_ADDR)) Serial.println("BME688 초기화 실패");
  if (!ozone.begin(OZONE_ADDR)) Serial.println("Ozone 센서 초기화 실패");
  if (!mlx.begin_I2C(MLX90393_ADDR)) Serial.println("MLX90393 초기화 실패");
  if (!mcp.begin(MCP9600_ADDR)) Serial.println("MCP9600 초기화 실패");
  sensors.begin();
  if (bmi270.beginI2C(BMI270_ADDR) != BMI2_OK) Serial.println("BMI270 초기화 실패");
  
  // PMS 센서 활성화
  pms.activeMode();
  
  // GDK101 센서 초기화
  Gamma_Mod_Read(0xB4); // 펌웨어 확인
  Serial.print("GDK101 Firmware V: "); Serial.print(gdkBuffer[0]); Serial.print("."); Serial.println(gdkBuffer[1]);
  Gamma_Mod_Read(0xA0); // 리셋
  
  Serial.println("모듈 초기화 완료");
}