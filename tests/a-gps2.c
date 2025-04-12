#include <HardwareSerial.h>
#include <TinyGPS++.h>
#include <HTTPClient.h>
#include <WiFi.h>
#include <EEPROM.h>

// Configurações UART
HardwareSerial SerialGPS(2);  // UART2 para NEO-6M (RX: GPIO16, TX: GPIO17)
HardwareSerial SerialSIM(1);  // UART1 para SIM800L (RX: GPIO4, TX: GPIO2)

// Objeto GPS
TinyGPSPlus gps;

// Dados A-GPS (exemplo: servidor público)
const char* agpsServer = "http://example.com/agps/data.ubx";  // Substitua pelo seu servidor

// EEPROM
#define EEPROM_SIZE 4096
#define AGPS_DATA_ADDR 0

void setup() {
  Serial.begin(115200);
  SerialGPS.begin(9600);  // NEO-6M
  SerialSIM.begin(9600);  // SIM800L

  // Inicia EEPROM
  EEPROM.begin(EEPROM_SIZE);

  // 1. Tenta carregar dados A-GPS da EEPROM
  if (loadAGPSFromEEPROM()) {
    Serial.println("Dados A-GPS carregados da EEPROM!");
  } else {
    // 2. Se falhar, baixa novos dados via rede celular
    if (connectToNetwork()) {
      downloadAGPSData();
    }
  }

  // 3. Configura o GPS para alta performance
  configureGPS();
}

void loop() {
  // Monitora dados do GPS
  while (SerialGPS.available() > 0) {
    if (gps.encode(SerialGPS.read())) {
      if (gps.location.isValid()) {
        Serial.print("Lat: "); Serial.print(gps.location.lat(), 6);
        Serial.print(", Lng: "); Serial.println(gps.location.lng(), 6);
      }
    }
  }
}

// --------------------------------------------
// Funções Auxiliares
// --------------------------------------------

bool connectToNetwork() {
  SerialSIM.println("AT+CREG?");  // Verifica registro na rede
  delay(1000);
  SerialSIM.println("AT+CGATT=1");  // Anexa ao GPRS
  delay(2000);
  return true;  // Simplificado (verifique resposta real)
}

bool downloadAGPSData() {
  HTTPClient http;
  http.begin(agpsServer);
  int httpCode = http.GET();

  if (httpCode == HTTP_CODE_OK) {
    uint8_t* agpsData = http.getStreamPtr();
    size_t agpsSize = http.getSize();

    // Envia dados para o NEO-6M
    SerialGPS.write(agpsData, agpsSize);

    // Salva na EEPROM para uso futuro
    EEPROM.put(AGPS_DATA_ADDR, agpsSize);
    EEPROM.writeBytes(AGPS_DATA_ADDR + sizeof(size_t), agpsData, agpsSize);
    EEPROM.commit();

    Serial.println("Dados A-GPS baixados e injetados!");
    return true;
  }
  return false;
}

bool loadAGPSFromEEPROM() {
  size_t agpsSize;
  EEPROM.get(AGPS_DATA_ADDR, agpsSize);

  if (agpsSize > 0 && agpsSize < EEPROM_SIZE) {
    uint8_t agpsData[agpsSize];
    EEPROM.readBytes(AGPS_DATA_ADDR + sizeof(size_t), agpsData, agpsSize);
    SerialGPS.write(agpsData, agpsSize);
    return true;
  }
  return false;
}

void configureGPS() {
  // Configura NEO-6M para 10Hz e mensagens UBX
  const uint8_t ubxConfig[] = {
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0x64, 0x00, 0x01, 0x00, 0x01, 0x00, 0x7A, 0x12  // UBX-CFG-RATE (10Hz)
  };
  SerialGPS.write(ubxConfig, sizeof(ubxConfig));
  delay(500);
}