#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>

#define BOT_TOKEN ""
#define CHAT_ID ""

int contadorMensagem = 1;
WiFiClientSecure client;
UniversalTelegramBot bot(BOT_TOKEN, client);

const char* ssid = ""; //nome da rede
const char* password = ""; //senha da rede

void sendMessageClient(String localizacao){

    while (contadorMensagem){
    String mensagem = "🚨 ALERTA DE EMERGÊNCIA 🚨\n";
    mensagem += "O usuário sofreu um acidente!\n\n";
    mensagem += "📍 Localização aproximada:\n";
    mensagem += localizacao + "\n\n";

    if (bot.sendMessage(CHAT_ID, mensagem, "")) {
      Serial.println("Alerta de acidente enviado com sucesso!");
      Serial.println("Localização: " + localizacao);
      contadorMensagem = 0;
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
    delay(10000);
    Serial.println("conectando ao wifi..");
  }
  // Print ESP32 Local IP Address
  Serial.println(WiFi.localIP());

}

void loop() {
  sendMessageClient("latitude: x, longitude: y");
}
