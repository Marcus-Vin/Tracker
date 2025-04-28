#include <HardwareSerial.h>
#include <TinyGPS++.h>


#ifndef GETGPS_INO  // Verifica se GETGPS_H não foi definido
#define GETGPS_INO  

typedef struct {  
    float latitude;
    float longitude;
} locData;



locData getGPS(TinyGPSPlus gps, HardwareSerial gpsSerial)
{
  bool newData = false;
  
  // Por um segundo analisamos os dados GPS e reportamos alguns valores chave
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (gpsSerial.available())
    {
      // gpsSerial.read(); // descomente para mostrar os dados crus
      if (gps.encode(gpsSerial.read())) // Atribui true para newData caso novos dados sejam recebidos
        newData = true;
    }
  }

  if (newData)
  {
    if (gps.location.isValid()) {
        // Serial.println("Lat: %lf , Long: %lf", gps.location.lat(), gps.location.lng()); dando erro
        locData location = {gps.location.lat(), gps.location.lng()}; 
        return location;
    } else {
      Serial.print("INVALID");
      
    }
  }
  
  // Se não houver dados válidos por um longo período, mostra mensagem
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println("No GPS data received: check wiring");
  }
}
#endif