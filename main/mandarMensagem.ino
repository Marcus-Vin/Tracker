#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>

#ifndef SENDMSG_INO
#define SENDMSG_INO

#define BOT_TOKEN "7640736361:AAGRWd36iBlhZHl4EbgCm_566Bn8EPaV9Do" //token do bot
#define CHAT_ID "6103199211" //id do user

bool enviarMensagem = 1;
WiFiClientSecure client;
UniversalTelegramBot bot(BOT_TOKEN, client);


//função para envio da mensagem através do bot do telegram
void sendMessageClient(double latitude, double longitude, bool queda){
  if (queda){ //caso haja queda, a repetição abaixo é acionada
    String localizacao = "Latitude: " + String(latitude) + "Longitude: " + String(longitude);
    enviarMensagem = 1;

    while (enviarMensagem){ 
    String mensagem = "🚨 ALERTA DE EMERGÊNCIA 🚨\n";
    mensagem += "O usuário sofreu um acidente!\n\n";
    mensagem += "📍 Localização aproximada:\n";
    mensagem += localizacao + "\n\n";

    if (bot.sendMessage(CHAT_ID, mensagem, "")){ //envio da mensagem
      Serial.println("Alerta de acidente enviado com sucesso!");
      Serial.println("Localização: " + localizacao);
      enviarMensagem = 0; //quebra do loop para a mensagem ser mandada apenas uma vez
    } else{
      Serial.println("Falha ao enviar alerta!");
      }
    }
  }
}
#endif
// void setup() {
//   Serial.begin(115200);

//   //conetar ao wifi
//   WiFi.mode(WIFI_STA);
//   WiFi.begin(ssid, password);
//   client.setCACert(TELEGRAM_CERTIFICATE_ROOT);

//   while (WiFi.status() != WL_CONNECTED) {
//     delay(10000);
//     Serial.println("conectando ao wifi..");
//   }

//   Serial.println(WiFi.localIP());
// }
