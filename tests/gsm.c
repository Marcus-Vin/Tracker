#include <HardwareSerial.h>

// Configuração da UART para o SIM800L
HardwareSerial SerialSIM(1);  // UART1 (RX: GPIO4, TX: GPIO2)

// Dados do SMS
const String PHONE_NUMBER = "+5583981243468";  // Número de destino (com código do país)
const String SMS_TEXT = "Olá, esta é uma mensagem de teste do TRACKER!";

void setup() {
  Serial.begin(115200);          // Serial para debug
  SerialSIM.begin(9600);         // Inicia comunicação com SIM800L
  delay(2000);                   // Aguarda inicialização do módulo

  Serial.println("Iniciando teste do SIM800L...");

  // Verifica se o módulo está respondendo
  sendATCommand("AT", "OK", 2000);

  // Configurações iniciais
  sendATCommand("AT+CMGF=1", "OK", 1000);     // Modo texto para SMS
  sendATCommand("AT+CNMI=1,2,0,0,0", "OK", 1000);  // Configura recebimento de SMS

  // Verifica sinal de rede
  checkNetwork();

  // Envia SMS
  sendSMS(PHONE_NUMBER, SMS_TEXT);
}

void loop() {
  // Nada aqui (o teste é executado apenas uma vez no setup)
}

// --------------------------------------------
// Funções Auxiliares
// --------------------------------------------

// Envia comando AT e verifica resposta
String sendATCommand(String command, String expectedResponse, uint32_t timeout) {
  Serial.print("Enviando: ");
  Serial.println(command);

  SerialSIM.println(command);
  String response = "";
  uint32_t startTime = millis();

  while (millis() - startTime < timeout) {
    if (SerialSIM.available()) {
      char c = SerialSIM.read();
      response += c;
      Serial.write(c);  // Mostra a resposta no monitor serial
    }
  }

  if (response.indexOf(expectedResponse) == -1) {
    Serial.println("Falha no comando: " + command);
    return "ERROR";
  }

  return response;
}

// Verifica sinal de rede e registro
void checkNetwork() {
  Serial.println("Verificando rede...");

  // Espera pelo sinal
  while (sendATCommand("AT+CSQ", "OK", 2000).indexOf("+CSQ:") == -1) {
    delay(1000);
  }

  // Verifica registro na rede
  while (sendATCommand("AT+CREG?", "+CREG: 0,1", 2000).indexOf("+CREG: 0,1") == -1) {
    delay(1000);
    Serial.println("Aguardando registro na rede...");
  }

  Serial.println("Rede registrada!");
}

// Envia SMS
void sendSMS(String number, String text) {
  Serial.println("Preparando para enviar SMS...");

  sendATCommand("AT+CMGS=\"" + number + "\"", ">", 5000);  // Configura número
  SerialSIM.print(text);                                   // Envia texto
  SerialSIM.write(26);                                     // CTRL+Z para finalizar

  Serial.println("SMS enviado! Aguarde confirmação...");

  // Aguarda confirmação (pode demorar alguns segundos)
  delay(10000);
  while (SerialSIM.available()) {
    Serial.write(SerialSIM.read());
  }
}