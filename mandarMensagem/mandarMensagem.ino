#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>

#define BOTtoken "a colocar"
#define CHAT_id "a definir"

WiFiClientSecure client;
UniversalTelegramBot bot(BOTtoken, client);

const char* ssid = ""; //nome da rede
const char* password = ""; //senha da rede

void sendMessageClient(String localizacao){
  int contadorMensagem = 0;

    while (contadorMensagem == 0){
    String mensagem = "🚨 ALERTA DE EMERGÊNCIA 🚨\n";
    mensagem += "O usuário sofreu um acidente!\n\n";
    mensagem += "📍 Localização aproximada:\n";
    mensagem += localizacao + "\n\n";

    if (bot.sendMessage(CHAT_ID, mensagem, "")) {
      Serial.println("Alerta de acidente enviado com sucesso!");
      Serial.println("Localização: " + localizacao);
      contadorMensagem = 1;
    } else {
      Serial.println("Falha ao enviar alerta!");
    }
  }
  
}

void setup() {
  Serial.begin(115200);

  //conetar ao wifi
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  client.setCACert(TELEGRAM_CERTIFICATE_ROOT);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("conectando ao wifi..");
  }
  // Print ESP32 Local IP Address
  Serial.println(WiFi.localIP());
}

void loop() {
  // put your main code here, to run repeatedly:
}
