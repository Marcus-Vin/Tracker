#include <HardwareSerial.h>
#include <TinyGPS++.h>

#ifndef GETGPS_INO  // Verifica se GETGPS_H não foi definido
#define GETGPS_INO  

typedef struct {  
    float latitude;
    float longitude;
} locData;

class GPSHandler {
  private:
    TinyGPSPlus& gps;  // Referência ao objeto GPS
    HardwareSerial& gpsSerial;  // Referência ao objeto serial
    unsigned long lastGPSUpdateTime;  // Armazena o tempo da última atualização GPS
    unsigned long timeout;  // Intervalo entre as leituras de GPS

  public:
    // Construtor com injeção de dependência
    GPSHandler(TinyGPSPlus& gpsInstance, HardwareSerial& serialInstance, unsigned long timeoutValue = 1000) 
      : gps(gpsInstance), gpsSerial(serialInstance), timeout(timeoutValue), lastGPSUpdateTime(0) {}

    locData getGPS() {
      bool newData = false;

      // Verifica se o intervalo de tempo passou
      if (millis() - lastGPSUpdateTime >= timeout) {
        lastGPSUpdateTime = millis();  // Atualiza o tempo da última leitura

        // Processa os dados disponíveis na serial
        while (gpsSerial.available()) {
          if (gps.encode(gpsSerial.read())) {
            newData = true;  // Dados válidos foram lidos
          }
        }

        if (newData) {
          if (gps.location.isValid()) {
            locData location = {gps.location.lat(), gps.location.lng()};
            return location;
          } else {
            Serial.println("INVALID GPS data");
          }
        } else {
          Serial.println("Waiting for GPS data...");
        }
      }

      // Se não houver dados válidos por um longo período, mostra mensagem
      if (millis() > 5000 && gps.charsProcessed() < 10) {
        Serial.println("No GPS data received: check wiring");
      }

      // Retorna valores padrão caso não tenha dados
      return {0.0, 0.0};
    }
};

// HardwareSerial mySerial(1);  // Configura a Serial 1 para o GPS
// TinyGPSPlus gps;  // Instância do objeto GPS
// GPSHandler gpsHandler(gps, mySerial, 1000);  // Instância do handler de GPS

// Setup do sistema (inicialização)
// void setup() {
//   // Inicializa a comunicação serial com o computador
//   Serial.begin(115200);
  
//   // Inicializa a comunicação serial com o GPS (usando a Serial 1 na ESP32)
//   mySerial.begin(9600, SERIAL_8N1, 16, 17);  // RX no pino 16 e TX no pino 17
  
//   Serial.println("GPS Test Started");
// }

// // Loop principal (executado repetidamente)
// void loop() {
//   locData location = gpsHandler.getGPS();

//   // Mostra as coordenadas no Serial Monitor
//   if (location.latitude != 0.0 && location.longitude != 0.0) {
//     Serial.print("Latitude: ");
//     Serial.print(location.latitude, 6);
//     Serial.print(" | Longitude: ");
//     Serial.println(location.longitude, 6);
//   }

//   // Aqui o código pode continuar executando outras tarefas
//   // como monitorar entradas, realizar processamento, etc.
// }
#endif