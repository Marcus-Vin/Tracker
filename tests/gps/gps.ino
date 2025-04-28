// Definindo os pinos do UART2 do ESP32
#define GPS_RX 16  // RX2 (GPIO16)
#define GPS_TX 17  // TX2 (GPIO17)
#define GPS_Serial_Baud 9600

#include <HardwareSerial.h>
#include <TinyGPS.h>

TinyGPS gps;  // Note: Using TinyGPS, not TinyGPS++
HardwareSerial gpsSerial(2);  // UART2 do ESP32

void setup()
{
  Serial.begin(115200);
  gpsSerial.begin(GPS_Serial_Baud, SERIAL_8N1, GPS_RX, GPS_TX);
  Serial.println("GPS inicializado. Aguardando dados...");
}

void loop()
{
  bool newData = false;
  
  // For one second we parse GPS data
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    while (gpsSerial.available())
    {
      char c = gpsSerial.read();
      // Serial.write(c);  // Descomente para mostrar os dados crus
      if (gps.encode(c))  // Atribui true para newData caso novos dados sejam recebidos
        newData = true;
    }
  }
  
  if (newData)
  {
    float flat, flon;
    unsigned long age;
    gps.f_get_position(&flat, &flon, &age);
    
    Serial.print("LAT=");
    Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
    Serial.print(" LON=");
    Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
    Serial.print(" SAT=");
    Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
    Serial.print(" PREC=");
    Serial.print(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
    Serial.println();
    
    if (age > 5000)
      Serial.println("Warning: Data may be outdated!");
    
    if (gps.satellites() == 0)
      Serial.println("Waiting for satellite fix...");
  }
  else
  {
    Serial.println("No GPS data received. Check connections.");
  }
}