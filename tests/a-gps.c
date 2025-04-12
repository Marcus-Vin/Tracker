#include <HardwareSerial.h>
#include <TinyGPS++.h>

// Configuração das UARTs
HardwareSerial SerialGPS(2);  // UART2 para NEO-6M
HardwareSerial SerialSIM(1);  // UART1 para SIM800L
TinyGPSPlus gps;

// Dados A-GPS (simplificados)
const char* agpsData = "PGCMD,33,1*6D\r\n";  // Exemplo: ativa assistência via rede

void setup() {
  Serial.begin(115200);
  SerialGPS.begin(9600, SERIAL_8N1, 16, 17);  // NEO-6M
  SerialSIM.begin(9600, SERIAL_8N1, 4, 2);    // SIM800L

  // Inicializa SIM800L e obtém dados A-GPS
  setupSIM800L();
  sendAGPSData();
}

void loop() {
  // Processa dados do GPS
  while (SerialGPS.available() > 0) {
    if (gps.encode(SerialGPS.read())) {
      if (gps.location.isValid()) {
        Serial.print("Latitude: "); Serial.println(gps.location.lat(), 6);
        Serial.print("Longitude: "); Serial.println(gps.location.lng(), 6);
      }
    }
  }
}

void setupSIM800L() {
  SerialSIM.println("AT");  // Testa comunicação
  delay(1000);
  SerialSIM.println("AT+CGATT=1");  // Anexa à rede GPRS
  delay(2000);
}

void sendAGPSData() {
  SerialGPS.println(agpsData);  // Envia dados A-GPS para o NEO-6M
  delay(500);
}