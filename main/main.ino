// para usar esse codigo tem que ser no arduino IDE e tem que ter baixado as bibliotecas MPU6050 by eletronic cats e a biblioteca de compatibilidade do esp32 com o arduino IDE, além disso, o tipo de esp selecionado foi o "esp32 dev module"

#include "I2Cdev.h"
#define I2CDEV_IMPLEMENTATION I2CDEV_ARDUINO_WIRE
#include <Wire.h>
#include "MPU6050_6Axis_MotionApps20.h"
#include "mandarMensagem.ino"
// #include "mesh.ino"
#include "getGPS.ino"

const char* ssid = ""; //nome da rede
const char* password = ""; //senha da rede

TinyGPSPlus gps;
HardwareSerial gpsSerial(2);  // UART2 do ESP32
GPSHandler gpsHandler(gps, gpsSerial, 1000);  // Instância do handler de GPS

// Uncomment these when you add the hardware
// #define BUZZER_PIN 5    // Pin for connecting a buzzer or alarm
// #define LED_PIN 13      // Status LED pin

// variáveis do filtro
float pitchFiltrado = 0;
float rollFiltrado = 0;
float filtroPorcentagem = 0.1; 

#define angQuedaPotencialMin 40.0f      // Ângulo mínimo para começar a detectar queda
#define acelQuedaPotencialMin 1.2f      // Aceleração mínima para começar a detectar queda
#define tempoMinQuedaConfirm 2500       // Tempo mín de queda para quedaConfirmada = true
#define tempoResetQueda 2500        // Tempo que leva pra resetar o quedaConfirmada para false
#define anguloEstavel 10.0f     // ângulo max de variação normal
#define angPorLeitura 3.5f      // ângulo max de variação de ang entre leituras do MPU
#define acelAdapt 0.005f        // quanto a aceleração vai depender de novas leituras pro filtro
#define acelNormalMax 0.8f      // aceleração máxima normal

#define I2C_SDA 21
#define I2C_SCL 22
bool quedaDetectada = false;
bool quedaConfirmada = false;
unsigned long tempoInicioQueda = 0;
unsigned long tempoConfirmQueda = 0;
bool queda = false;

float acelAtualFiltrada = 0.2f;
bool sistemaCalibrado = false;

namespace nsMPU {
    MPU6050 mpu;
}


// Definindo os pinos do UART2 do ESP32
#define GPS_RX 16  // RX2 (GPIO16)
#define GPS_TX 17  // TX2 (GPIO17)
#define GPS_Serial_Baud 9600

bool mpuInit()
{
    Serial.println("DEBUG: INICIALIZANDO MPU");

    // Iniciando I2C
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin(I2C_SDA, I2C_SCL); // Using ESP32 specific pins
        Wire.setClock(400000);
        Serial.println("DEBUG: Wire initialized with SDA=21, SCL=22");
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
        Serial.println("DEBUG: FastWire initialized");
    #endif

    // Iniciando MPU
    Serial.println("DEBUG: Inicia MPU...");
    nsMPU::mpu.initialize();

    // verifica a conexão do MPU
    Serial.println("DEBUG: Verificando conexão do MPU...");
    bool connected = nsMPU::mpu.testConnection();
    if (!connected) {
        Serial.println("ERROR: Conexão falhou! verifique os fios!");
        return false;
    }
    Serial.println("DEBUG: MPU conectado");

    // Iniciando DMP
    Serial.println("DEBUG: Iniciando DMP...");
    uint8_t devStatus = nsMPU::mpu.dmpInitialize();

    // Calibrando offsets
    Serial.println("DEBUG: Calibrando offsets...");
    mpuCalibrateOffsets();

    // checagem do DMP
    if (devStatus == 0) {
        Serial.println("DEBUG: DMP iniciou corretamente");

        // Reset FIFO before enabling DMP
        nsMPU::mpu.resetFIFO();
        delay(100);

        Serial.println("DEBUG: Habilitando DMP...");
        nsMPU::mpu.setDMPEnabled(true);
        Serial.println("DEBUG: DMP Habilitado");

        // Reset FIFO again after enabling DMP
        nsMPU::mpu.resetFIFO();

        return true;
    } else {
        Serial.print("ERROR: DMP initialization failed (code ");
        Serial.print(devStatus);
        Serial.println(")");
        return false;
    }
}


float angNormaliza(float angMedido, float angMedidoMax, float angObjetivo) {

    bool ehNegativo = angMedido < 0;

    float angAbsoluto = abs(angMedido);
    float angMedidoMaxAbsoluto = abs(angMedidoMax);
    float angObjetivoAbsoluto = abs(angObjetivo);


    float angNormalizado = (angAbsoluto / angMedidoMaxAbsoluto) * angObjetivoAbsoluto;

    if (ehNegativo) {
        angNormalizado = -angNormalizado;
    }

    return angNormalizado;

}

bool mpuGetYawPitchRoll(float& y, float& p, float& r, float& acelAtual)
{
    uint8_t FIFOBuffer[64];
    Quaternion q;
    VectorFloat gravity;
    VectorInt16 accel;
    VectorInt16 accelReal;

    uint16_t fifoCount = nsMPU::mpu.getFIFOCount();

    if (fifoCount >= 1024) {
        // Reseta FIFO
        nsMPU::mpu.resetFIFO();
        return false;
    }

    if (fifoCount < 42) {
        return false;
    }

    if (nsMPU::mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
        nsMPU::mpu.dmpGetQuaternion(&q, FIFOBuffer);
        nsMPU::mpu.dmpGetGravity(&gravity, &q);

        float gx = gravity.x;
        float gy = gravity.y;
        float gz = gravity.z;

        r = atan2(gy, gz) * 180.0 / M_PI;
        p = atan2(-gx, sqrt(gy * gy + gz * gz)) * 180.0 / M_PI;
        y = atan2(2.0f * (q.w * q.z + q.x * q.y), 1.0f - 2.0f * (q.y * q.y + q.z * q.z)) * 180.0f / M_PI;

        p = angNormaliza(p, -83.0f, -90.0f);
        r = angNormaliza(r, 83.0f, 90.0f);

        nsMPU::mpu.dmpGetAccel(&accel, FIFOBuffer);
        nsMPU::mpu.dmpGetLinearAccel(&accelReal, &accel, &gravity);

        acelAtual = sqrt(accelReal.x*accelReal.x + accelReal.y*accelReal.y + accelReal.z*accelReal.z) / 16384.0f;

        return true;
    }

    return false;
}

void mpuCalibrateOffsets() {

  int16_t ax, ay, az, gx, gy, gz;
  int32_t axSoma, aySoma, azSoma;
  int32_t gxSoma, gySoma, gzSoma;

  int16_t gx_offset = 0;
  int16_t gy_offset = 0;
  int16_t gz_offset = 0;
  int16_t ax_offset = 0;
  int16_t ay_offset = 0;
  int16_t az_offset = 0;

  // Calibra gyro
  for (int calibra = 0; calibra < 5; calibra++) {
    gxSoma = 0; gySoma = 0; gzSoma = 0;

    // Pega os offsets dos eixos do gyro
    nsMPU::mpu.setXGyroOffset(gx_offset);
    nsMPU::mpu.setYGyroOffset(gy_offset);
    nsMPU::mpu.setZGyroOffset(gz_offset);
    delay(100);

    // Faz leituras
    for (int i = 0; i < 100; i++) {
      nsMPU::mpu.getRotation(&gx, &gy, &gz);
      gxSoma += gx; gySoma += gy; gzSoma += gz;
      delay(5);
    }

    // Atualiza offsets
    gx_offset -= gxSoma / 100;
    gy_offset -= gySoma / 100;
    gz_offset -= gzSoma / 100;

  }

  // Calibra acelerômetro
  for (int calibra = 0; calibra < 5; calibra++) {
    axSoma = 0; aySoma = 0; azSoma = 0;

    // mesma coisa que gyro
    nsMPU::mpu.setXAccelOffset(ax_offset);
    nsMPU::mpu.setYAccelOffset(ay_offset);
    nsMPU::mpu.setZAccelOffset(az_offset);
    delay(100);

    for (int i = 0; i < 100; i++) {
      nsMPU::mpu.getAcceleration(&ax, &ay, &az);
      axSoma += ax; aySoma += ay; azSoma += az;
      delay(5);
    }

    int16_t ax_avg = axSoma / 100;
    int16_t ay_avg = aySoma / 100;
    int16_t az_avg = azSoma / 100;

    // offsets suaves
    ax_offset -= ax_avg / 8; 
    ay_offset -= ay_avg / 8;
    az_offset += (16384 - az_avg) / 8;

  }

  // Aplica offsets
  nsMPU::mpu.setXGyroOffset(gx_offset);
  nsMPU::mpu.setYGyroOffset(gy_offset);
  nsMPU::mpu.setZGyroOffset(gz_offset);
  nsMPU::mpu.setXAccelOffset(ax_offset);
  nsMPU::mpu.setYAccelOffset(ay_offset);
  nsMPU::mpu.setZAccelOffset(az_offset);

  delay(200);
}

bool detectaQueda(float pitch, float roll, float acelAtual) {
    static float acelMax = 0;
    static float angMax = 0;
    static float angAnterior = 0;

    float angAtual = max(abs(pitch), abs(roll));

    float taxaMudancaAng = angAtual - angAnterior;
    angAnterior = angAtual;

    acelMax = max(acelMax, acelAtual);
    angMax = max(angMax, angAtual);

    if (!quedaDetectada && angAtual < 20.0f) {
        if (acelAtual < acelNormalMax) {
            acelAtualFiltrada = acelAtualFiltrada * (1.0f - acelAdapt) + acelAtual * acelAdapt;
        }
    }


    float acelBrusca = acelAtual - acelAtualFiltrada;

    static unsigned long ultimoStatus = 0;
    if (millis() -  ultimoStatus >= 1000) {
        Serial.print(" | Ang: "); Serial.print(angAtual);
        Serial.print(" | Acel Sensor : "); Serial.print(acelAtual, 4);
        Serial.print(" | Acel Filtro: "); Serial.print(acelAtualFiltrada, 4);
        Serial.print(" | Acel Brusca: "); Serial.print(acelBrusca, 4);
        Serial.print(" | Taxa Ang: "); Serial.println(taxaMudancaAng);
        ultimoStatus = millis();
    }

    if (!quedaDetectada) {
        if (angAtual > 60.0f || (angAtual > angQuedaPotencialMin && (acelBrusca > acelQuedaPotencialMin || taxaMudancaAng > angPorLeitura))) {

            quedaDetectada = true;
            tempoInicioQueda = millis();

            return false;
        }
    } else if (!quedaConfirmada && quedaDetectada) {
        if (angAtual < anguloEstavel && (millis() - tempoInicioQueda > 2000)) {
            quedaDetectada = false;
            acelMax = 0;
            angMax = 0;

            return false;
        }

        if (millis() - tempoInicioQueda > tempoMinQuedaConfirm) {
            quedaConfirmada = true;
            tempoConfirmQueda = millis();
            Serial.println("QUEDA CONFIRMADA!");

            return true;
        }
    } else if (millis() - tempoConfirmQueda > tempoResetQueda || angAtual < anguloEstavel) {
        quedaDetectada = false;
        quedaConfirmada = false;
        acelMax = 0;
        angMax = 0;
    }

    return quedaConfirmada;
}

void setup() {

    gpsSerial.begin(GPS_Serial_Baud, SERIAL_8N1, GPS_RX, GPS_TX);

    Serial.begin(115200);

  //conetar ao wifi
    WiFi.mode(WIFI_STA);
    WiFi.begin(ssid, password);
    client.setCACert(TELEGRAM_CERTIFICATE_ROOT);

    delay(1000);

    if (!mpuInit()) {
        Serial.println("Failed to initialize MPU. Halting.");
        while (1) {
            delay(100);
        }
    }

    // Tempo de inicialização
    for (int i = 5; i > 0; i--) {
        delay(1000);
    }
}

void loop() {
    static unsigned long ultimoPrint = 0;
    static unsigned long semDataContagem = 0;
    static unsigned long loops = 0;
    static float ultimoPitch = 0;
    static float lastRoll = 0;

    float y = 0, p = 0, r = 0, acelAtual = 0;

    loops++;

    bool mpuYPR = mpuGetYawPitchRoll(y, p, r, acelAtual);

    if (mpuYPR) {
        if (abs(p - ultimoPitch) > 30) {
            p = ultimoPitch;
        }

        if (abs(r - lastRoll) > 30) {
            r = lastRoll;
        }

        ultimoPitch = p;
        lastRoll = r;

        // filtro passa baixo
        pitchFiltrado = (p * filtroPorcentagem) + (pitchFiltrado * (1.0 - filtroPorcentagem));
        rollFiltrado = (r * filtroPorcentagem) + (rollFiltrado * (1.0 - filtroPorcentagem));

        queda = detectaQueda(pitchFiltrado, rollFiltrado, acelAtual);

        if (queda) {
            Serial.print('_');
            locData gpsData = gpsHandler.getGPS();
              if (gpsData.latitude != 0.0 && gpsData.longitude != 0.0) {
                Serial.print("Latitude: ");
                Serial.print(gpsData.latitude, 6);
                Serial.print(" | Longitude: ");
                Serial.println(gpsData.longitude, 6);
                sendMessageClient(gpsData.latitude, gpsData.longitude, queda);
              }

            // Actions when fall is confirmed
            // Uncomment this section when you add a buzzer
            // digitalWrite(BUZZER_PIN, HIGH);
            // delay(1000);
            // digitalWrite(BUZZER_PIN, LOW);
            // Additional code could be added here to send an alert,
            // activate an alarm, send SMS, etc.
        }

        if (millis() - ultimoPrint >= 1000 || queda) {
            Serial.print("Y: ");
            Serial.print(y);
            Serial.print(" | P: ");
            Serial.print(pitchFiltrado);
            Serial.print(" | R: ");
            Serial.print(rollFiltrado);
            Serial.print(" | Accel: ");
            Serial.print(acelAtual);
            Serial.print("g");

            if (quedaDetectada && !quedaConfirmada) {
                Serial.print(" | POSSIVEL QUEDA");
                Serial.println();
            } else if (quedaConfirmada) {
                Serial.print(" | QUEDA CONFIRMADA");
                Serial.println();
            }

            Serial.print(" ");
            ultimoPrint = millis();
        }
    }

    delay(10);
}
  
