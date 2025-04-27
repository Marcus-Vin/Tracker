// para usar esse codigo tem que ser no arduino IDE e tem que ter baixado as bibliotecas MPU6050 by eletronic cats e a biblioteca de compatibilidade do esp32 com o arduino IDE, além disso, o tipo de esp selecionado foi o "esp32 dev module"

#include "I2Cdev.h"
#define I2CDEV_IMPLEMENTATION I2CDEV_ARDUINO_WIRE
#include <Wire.h>
#include "MPU6050_6Axis_MotionApps20.h"

// entradas de I2C do acelerômetro no esp
#define I2C_SDA 21
#define I2C_SCL 22
// Uncomment these when you add the hardware
// #define BUZZER_PIN 5    // Pin for connecting a buzzer or alarm
// #define LED_PIN 13      // Status LED pin

// variáveis utilizadas no filtro
float filteredPitch = 0;
float filteredRoll = 0;
float filterFactor = 0.1; // Adjust between 0.01 (very smooth) and 0.5 (less filtering)

#define angQuedaPotencialMin 40.0f      // Ângulo mínimo para começar a detectar queda
#define acelQuedaPotencialMin 1.2f      // Aceleração mínima para começar a detectar queda Base acceleration threshold (will be dynamically adjusted)
#define tempoMinQuedaConfirm 2500        // Tempo mínimo em condição de queda para começar a considerar queda como true
#define tempoResetQueda 2500            // Tempo que leva para resetar o queda true para queda false depois de voltar às condições de uma "não queda"
#define anguloEstavel 10.0f             // ângulo máximo que a bicicleta pode variar de forma estável
#define angPorLeitura 3.5f              // Quantidade máxima que o ângulo pode variar por ciclo sem detectar queda
#define acelAdapt 0.005f                // Quanto leva para se adaptar a novas acelerações (menor = adapta mais devagar, maior = adapta mais rápido)
#define acelNormalMax 0.8f              // Aceleração máxima que pode ser considerada normal, acima disso é considerado potencial queda

// valores para detecção de queda
bool quedaDetectada = false;
bool quedaConfirmada = false;
unsigned long tempoInicioQueda = 0;
unsigned long tempoConfirmQueda = 0;

float acelAtualFiltrada = 0.2f;     // guarda o valor atual de aceleração (vai ser alterada ao longo do tempo baseado com o "acelAdapt")
bool sistemaCalibrado = false;  // Se a calibração terminou ou não


// cria objeto mpu e atribui namespace
namespace nsMPU {
    MPU6050 mpu;
}

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

// Scales the measured angle to get a full 90 degrees
float scaleAngle(float angMedido, float angMedidoMax, float angObjetivo) {
    // Determina se é angulo positivo ou negativo
    bool ehNegativo = (angMedido < 0);

    float angAbsoluto = abs(angMedido);
    float angMedidoMaxAbsoluto = abs(angMedidoMax);
    float angObjetivoAbsoluto = abs(angObjetivo);

    // So deixa proporcional se o ângulo for maior que 5 graus pois medidas muito pequenas a diferença não interfere
    if (angAbsoluto > 5.0f) {
        // Deixando angulo proporcional
        float scaledAngle = (angAbsoluto / angMedidoMaxAbsoluto) * angObjetivoAbsoluto;

        // Angulos Muito grandes ficam limitados a um valor máximo
        if (angAbsoluto > angMedidoMaxAbsoluto * 0.95f) {
            scaledAngle = angObjetivoAbsoluto;
        }

        // Coloca sinal de volta caso precise
        return ehNegativo ? -scaledAngle : scaledAngle;
    }

    // For small angles, return as is
    return angMedido;
}

bool mpuGetYawPitchRoll(float& y, float& p, float& r, float& acelAtual)
{
    uint8_t FIFOBuffer[64];
    Quaternion q;
    VectorFloat gravity;
    VectorInt16 accel;
    VectorInt16 accelReal;

    // Get the FIFO count
    uint16_t fifoCount = nsMPU::mpu.getFIFOCount();

    // Check for overflow (common problem)
    if (fifoCount >= 1024) {
        // Reset FIFO
        Serial.println("FIFO overflow! Resetting...");
        nsMPU::mpu.resetFIFO();
        return false;
    }

    // Espera FIFO ter valores suficientes
    if (fifoCount < 42) {
        return false;  // Sem data suficiente
    }

    // Verifica se pacote FIFO está disponível
    if (nsMPU::mpu.dmpGetCurrentFIFOPacket(FIFOBuffer)) {
        // Get quaternion and gravity
        nsMPU::mpu.dmpGetQuaternion(&q, FIFOBuffer);
        nsMPU::mpu.dmpGetGravity(&gravity, &q);

        // Calcula ângulos da gravidade
        float gx = gravity.x;
        float gy = gravity.y;
        float gz = gravity.z;

        // Calcula pitch e roll
        r = atan2(gy, gz) * 180.0 / M_PI;
        p = atan2(-gx, sqrt(gy * gy + gz * gz)) * 180.0 / M_PI;

        // usa quaternio pra conseguir yaw
        y = atan2(2.0f * (q.w * q.z + q.x * q.y), 1.0f - 2.0f * (q.y * q.y + q.z * q.z)) * 180.0f / M_PI;

        // Deixa os ângulos de pitch e roll proporcionais a 90 graus, 83 é um valor arbitrário baseado em testes prévios
        p = scaleAngle(p, -83.0f, -90.0f);
        r = scaleAngle(r, 83.0f, 90.0f);

        // Pega valores da aceleração
        nsMPU::mpu.dmpGetAccel(&accel, FIFOBuffer);
        nsMPU::mpu.dmpGetLinearAccel(&accelReal, &accel, &gravity);

        // Calculate total acceleration magnitude (in g)
        acelAtual = sqrt(accelReal.x*accelReal.x + accelReal.y*accelReal.y + accelReal.z*accelReal.z) / 16384.0f; // 16384 LSB/g for ±2g range

        return true;
    }

    return false;
}

void mpuCalibrateOffsets() {
  int16_t ax, ay, az, gx, gy, gz;
  int32_t ax_sum, ay_sum, az_sum;
  int32_t gx_sum, gy_sum, gz_sum;
  int qtdLeituras = 100; // Quantidade de leituras do MPU a cada calibração
  int qtdInteracoes = 5; // Quantas vezes a calibração do MPU será feita

  // Offsets iniciais
  int16_t gx_offset = 0;
  int16_t gy_offset = 0;
  int16_t gz_offset = 0;
  int16_t ax_offset = 0;
  int16_t ay_offset = 0;
  int16_t az_offset = 0;

  Serial.println("Calibrando MPU - MANTENHA PARADO EM SUPERFíCIE LISA");

  // Calibra gyro
  Serial.println("Calibranto gyroscope...");
  for (int interacao = 0; interacao < qtdInteracoes; interacao++) {
    gx_sum = 0; gy_sum = 0; gz_sum = 0;

    // Pega os offsets dos eixos do gyro
    nsMPU::mpu.setXGyroOffset(gx_offset);
    nsMPU::mpu.setYGyroOffset(gy_offset);
    nsMPU::mpu.setZGyroOffset(gz_offset);
    delay(100);

    // Faz leituras
    for (int i = 0; i < qtdLeituras; i++) {
      nsMPU::mpu.getRotation(&gx, &gy, &gz);
      gx_sum += gx; gy_sum += gy; gz_sum += gz;
      delay(5);
    }

    // Atualiza offsets por média aritmética a cada calibração
    gx_offset -= gx_sum / qtdLeituras;
    gy_offset -= gy_sum / qtdLeituras;
    gz_offset -= gz_sum / qtdLeituras;

    Serial.print("Gyro calibração ");
    Serial.print(interacao + 1);
    Serial.print(": X=");
    Serial.print(gx_sum / qtdLeituras);
    Serial.print(" Y=");
    Serial.print(gy_sum / qtdLeituras);
    Serial.print(" Z=");
    Serial.println(gz_sum / qtdLeituras);
  }

  // Calibra acelerômetro
  Serial.println("Calibrando acelerômetro...");
  for (int interacao = 0; interacao < qtdInteracoes; interacao++) {
    ax_sum = 0; ay_sum = 0; az_sum = 0;

    // Pega offsets dos eixos do acelerômetro
    nsMPU::mpu.setXAccelOffset(ax_offset);
    nsMPU::mpu.setYAccelOffset(ay_offset);
    nsMPU::mpu.setZAccelOffset(az_offset);
    delay(100);

    // Faz leituras
    for (int i = 0; i < qtdLeituras; i++) {
      nsMPU::mpu.getAcceleration(&ax, &ay, &az);
      ax_sum += ax; ay_sum += ay; az_sum += az;
      delay(5);
    }

    // Calcula médias
    int16_t ax_avg = ax_sum / qtdLeituras;
    int16_t ay_avg = ay_sum / qtdLeituras;
    int16_t az_avg = az_sum / qtdLeituras;

    // atualiza offsets a cada calibração
    ax_offset -= ax_avg / 8;  // Divide por para suavizar as correções do acelerômetro para
    ay_offset -= ay_avg / 8;  // se aproximar do valor ideal sem oscilações muito grandes

    // por causa da gravidade, temos que somar 16384
    az_offset += (16384 - az_avg) / 8;  // Divisão por 8 tem mesmo motivo que os outros offsets

    Serial.print("Acelerômetro Calibração ");
    Serial.print(interacao + 1);
    Serial.print(": X=");
    Serial.print(ax_avg);
    Serial.print(" Y=");
    Serial.print(ay_avg);
    Serial.print(" Z=");
    Serial.println(az_avg);
  }

  // Apply final offsets
  nsMPU::mpu.setXGyroOffset(gx_offset);
  nsMPU::mpu.setYGyroOffset(gy_offset);
  nsMPU::mpu.setZGyroOffset(gz_offset);
  nsMPU::mpu.setXAccelOffset(ax_offset);
  nsMPU::mpu.setYAccelOffset(ay_offset);
  nsMPU::mpu.setZAccelOffset(az_offset);

  Serial.println("Calibration complete!");
  Serial.print("Gyro offsets: X=");
  Serial.print(gx_offset);
  Serial.print(" Y=");
  Serial.print(gy_offset);
  Serial.print(" Z=");
  Serial.println(gz_offset);

  Serial.print("Accel offsets: X=");
  Serial.print(ax_offset);
  Serial.print(" Y=");
  Serial.print(ay_offset);
  Serial.print(" Z=");
  Serial.println(az_offset);

  delay(200);
}

bool detectFall(float pitch, float roll, float acelAtual) {
    static float acelMax = 0;
    static float angMax = 0;
    static float angAnterior = 0;

    // considera ângulo atual o maior ângulo entre pitch e roll
    float angAtual = max(abs(pitch), abs(roll));

    // calcula a taxa de mudança do ângulo a cada medição
    float taxaMudancaAng = angAtual - angAnterior;
    angAnterior = angAtual;

    // atualiza os valores máximos para ângulo e aceleração durante queda
    acelMax = max(acelMax, acelAtual);
    angMax = max(angMax, angAtual);

    // checagem se a bicicleta está em uma condição "sem risco de estar em possível queda"
    if (!quedaDetectada && angAtual < 20.0f) {
        // Atualiza a aceleração atual filtrada aos poucos a cada leitura
        // (se acelAdapt = 0.005, então somente 0.5% da aceleração atual será
        // levada em consideração para alterar a aceleração filtrada, e todo
        // o resto será retirada da aceleração atual filtrada anterior)
        if (acelAtual < acelNormalMax) {
            acelAtualFiltrada = acelAtualFiltrada * (1.0f - acelAdapt) + acelAtual * acelAdapt;
        }
    }

    // calcula a diferença entre a aceleração atual filtrada e aceleração que acabou de
    // ser registrada no sensor para detectar se teve uma mudança brusca na aceleração e,
    // assim, detectar possível queda
    float acelBrusca = acelAtual - acelAtualFiltrada;

    // printa valores de aceleração atingidos
    static unsigned long ultimoStatus = 0;
    if (millis() -  ultimoStatus >= 1000) {
        Serial.print(" | Ang: "); Serial.print(angAtual);
        Serial.print(" | Acel Sensor : "); Serial.print(acelAtual, 4);
        Serial.print(" | Acel Filtro: "); Serial.print(acelAtualFiltrada, 4);
        Serial.print(" | Acel Brusca: "); Serial.print(acelBrusca, 4);
        Serial.print(" | Taxa Ang: "); Serial.println(taxaMudancaAng);
        ultimoStatus = millis();
    }

    // First stage fall detection (potential fall)
    if (!quedaDetectada) {
        // Detecta queda se:
        // 1. Ângulo muito inclinado (> 60°)
        // 2. Ângulo moderado (> 40° = angQuedaPotencial) e mudança brusca na aceleração
        //    (> 1.2 = acelQuedaPotencialMin) ou no ângulo (> 3.5 = angPorLeitura) entre
        //    duas leituras seguidas do acelerômetro
        if (angAtual > 60.0f ||
            (angAtual > angQuedaPotencialMin &&
             (acelBrusca > acelQuedaPotencialMin || taxaMudancaAng > angPorLeitura))) {

            quedaDetectada = true;
            tempoInicioQueda = millis();
            Serial.println("POSSÍVEL QUEDA DETECTADA!");
            Serial.print("Ang: "); Serial.print(angAtual);
            Serial.print(" | Acel Sensor: "); Serial.print(acelAtual);
            Serial.print(" | Acel Brusca: "); Serial.print(acelBrusca);
            Serial.print(" | Taxa Ang: "); Serial.print(taxaMudancaAng);

            // Visual indication - uncomment when you add an LED
            // digitalWrite(LED_PIN, HIGH);

            return false; // Not confirmed yet
        }
    }
    // Confirmação da queda
    else if (!quedaConfirmada && quedaDetectada) {
        // Se retornar a posição estável rápido, não considera queda
        if (angAtual < anguloEstavel && (millis() - tempoInicioQueda > 2000)) {
            // Coloca valores como "não queda"
            quedaDetectada = false;
            acelMax = 0;
            angMax = 0;
            Serial.println("Alarme falso - voltou a uma posição estável");

            // Turn off visual indication - uncomment when you add an LED
            // digitalWrite(LED_PIN, LOW);

            return false;
        }

        // se não retornar a posição rápido (< 2500 ms = tempoMinQuedaConfirm), considera como queda confirmada
        if (millis() - tempoInicioQueda > tempoMinQuedaConfirm) {
            quedaConfirmada = true;
            tempoConfirmQueda = millis();
            Serial.println("QUEDA CONFIRMADA!");
            Serial.print("Ang Max: "); Serial.print(angMax);
            Serial.print(" | Acel Max: "); Serial.println(acelMax);

            // Determina Gravidade da queda
            String quedaSituacao;
            if (angMax > 70.0f && acelMax > 2.0f) {
                quedaSituacao = "Grave";
            } else if (angMax > 50.0f || acelMax > 1.5f) {
                quedaSituacao = "Moderada";
            } else {
                quedaSituacao = "Baixa";
            }
            Serial.print("Situação da queda: ");
            Serial.println(quedaSituacao);

            return true;
        }
    }
    // Reseta valores de queda se passarem (> 2500 ms) desde confirmação de queda
    // e sensor voltou para posição estável
    else if (millis() - tempoConfirmQueda > tempoResetQueda || angAtual < anguloEstavel) {
        quedaDetectada = false;
        quedaConfirmada = false;
        acelMax = 0;
        angMax = 0;
        Serial.println("Resetando valores de queda");

        // Turn off visual indication - uncomment when you add an LED
        // digitalWrite(LED_PIN, LOW);
    }

    return quedaConfirmada;
}

void setup() {
    // Inicializando serial
    Serial.begin(115200);
    delay(1000);

    // Initialize pins - uncomment when you add hardware
    // pinMode(LED_PIN, OUTPUT);
    // pinMode(BUZZER_PIN, OUTPUT);

    // Initial LED blink to show system is starting - uncomment when you add an LED
    // digitalWrite(LED_PIN, HIGH);
    // delay(100);
    // digitalWrite(LED_PIN, LOW);

    Serial.println("\n\n");
    Serial.println("=======================");
    Serial.println("Bicycle Fall Detection System");
    Serial.println("=======================");

    // Inicializando MPU
    if (!mpuInit()) {
        Serial.println("Failed to initialize MPU. Halting.");
        // Error pattern - uncomment when you add an LED
        // while (1) {
        //     digitalWrite(LED_PIN, HIGH);
        //     delay(100);
        //     digitalWrite(LED_PIN, LOW);
        //     delay(100);
        // }
        while (1) {
            delay(100); // Just wait if no LED
        }
    }

    // Tempo de inicialização
    Serial.println("Finalizando... Por favor Espere.");
    for (int i = 5; i > 0; i--) {
        Serial.print(i);
        Serial.println(" Contagem regressiva...");
        delay(1000);
    }

    // System ready indication - uncomment when you add an LED
    // digitalWrite(LED_PIN, HIGH);
    // delay(500);
    // digitalWrite(LED_PIN, LOW);

    Serial.println("Pronto. detecção de queda ativa.");
}

void loop() {
    static unsigned long ultimoPrint = 0;
    static unsigned long semDataContagem = 0;
    static unsigned long loopsContagem = 0;
    static float ultimoPitch = 0;
    static float lastRoll = 0;

    float y = 0, p = 0, r = 0, acelAtual = 0;

    loopsContagem++;

    // pega valores de Yaw Pitch and Roll
    bool gotData = mpuGetYawPitchRoll(y, p, r, acelAtual);

    if (gotData) {
        // Usa valor do Último Pitch que não teve mudança brusca
        if (abs(p - ultimoPitch) > 30) {
            // Likely a transient spike, use the last good value
            p = ultimoPitch;
        }

        // Usa valor do Último Roll que não teve mudança brusca
        if (abs(r - lastRoll) > 30) {
            r = lastRoll;
        }

        // Guarda valores de último pitch para próxima comparação
        ultimoPitch = p;
        lastRoll = r;

        // Filtro que evita que surtos aleatórios na detecção façam grande alteração
        // nas próximas medições do sensor
        filteredPitch = (p * filterFactor) + (filteredPitch * (1.0 - filterFactor));
        filteredRoll = (r * filterFactor) + (filteredRoll * (1.0 - filterFactor));

        // Usa função de detectar quedas com valores filtrados para melhor precisão
        bool fall = detectFall(filteredPitch, filteredRoll, acelAtual);

        // Ação caso quedaConfirmada = true
        if (fall) {
            // Actions when fall is confirmed
            // Uncomment this section when you add a buzzer
            // digitalWrite(BUZZER_PIN, HIGH);
            // delay(1000);
            // digitalWrite(BUZZER_PIN, LOW);

            // Additional code could be added here to send an alert,
            // activate an alarm, send SMS, etc.
        }

        // Printa os valores a cada 1000 ms ou se quedaConfirmada = true
        if (millis() - ultimoPrint >= 1000 || fall) {
            Serial.print("Y: ");
            Serial.print(y);
            Serial.print(" | P: ");
            Serial.print(filteredPitch);
            Serial.print(" | R: ");
            Serial.print(filteredRoll);
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
    } else {
        semDataContagem++;

        // Only print "no data" message occasionally
        if (millis() - ultimoPrint >= 2000) {
            Serial.print("No data available (");
            Serial.print((float)semDataContagem / loopsContagem * 100.0);
            Serial.println("% of readings failed)");
            ultimoPrint = millis();
        }
    }

    delay(10);  // 10ms delay gives approximately 100Hz update rate
}
  
