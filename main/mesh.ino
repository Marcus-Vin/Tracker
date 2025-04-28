// #include <esp_now.h>
// #include <WiFi.h>

// typedef struct struct_message {
//     char type;      
//     char id[5];     
//     char command[5];
//     uint8_t mac[6];
//     int msgID;      
//     int hopCount = 0;
//     char palavra[10];
// } struct_message;

// struct_message myData;
// int lastReceivedMsgID = -1;
// String myID = "2";  // Change this value
// int msgCount = 0;

// #define MAX_PEERS 10  
// uint8_t knownPeers[MAX_PEERS][6];  
// int peerCount = 0;  

// unsigned long lastBroadcastTime = 0;
// const unsigned long broadcastInterval = 10000; 
// const int MAX_HOPS = MAX_PEERS-1;

// // void setup() {
// //     Serial.begin(115200);
// //     pinMode(LED_BUILTIN, OUTPUT);
// //     digitalWrite(LED_BUILTIN, LOW);
// //     memset(knownPeers, 0, sizeof(knownPeers));

// //     WiFi.mode(WIFI_STA);  
// //     delay(100);

// //     WiFi.disconnect();  
// //     delay(100);


// //     if (esp_now_init() == ESP_OK)
// //     {
// //     Serial.println("ESP-NOW Init Success");
// //     esp_now_register_recv_cb(OnDataRecv);
// //     esp_now_register_send_cb(OnDataSent);
// //     }

// // else {
// //     Serial.println("ESP-NOW Init Failed");
// //     ESP.restart();
// //     return;
// // }


// // }


// // void loop() {
// //     if (Serial.available() > 0) {
// //     String command = Serial.readString();
// //     command.trim();
// //     Serial.println("Command from Serial: " + command);
// //     myData.type = 'C';
// //     strncpy(myData.id, command.c_str(), sizeof(myData.id));
// //     strncpy(myData.command, command.c_str(), sizeof(myData.command));
// //     myData.msgID = msgCount++;
// //     sendCommand(myData);
    
// // }

// unsigned long currentTime = millis();
// if (currentTime - lastBroadcastTime >= broadcastInterval) {
//     startDiscoveryBroadcast();
//     lastBroadcastTime = currentTime;

// }

// void sendCommand(struct_message commandData) {
//     Serial.println("Command data is: " + String(commandData.id));
//     Serial.println("My ID: " + String(myID));
//     if (String(commandData.id) != myID) {
//         for (int i = 0; i < peerCount; i++) {
//             esp_err_t result = esp_now_send(knownPeers[i], (uint8_t *)&commandData, sizeof(commandData));
//             char formattedMac[20];  // Store the formatted MAC address
//             formatMacAddress(knownPeers[i], formattedMac, sizeof(formattedMac));
//             if (result == ESP_OK) {
//                 Serial.println("Successfully sent command " + String(commandData.command) + " to MAC: " + String(formattedMac));
//             } else {
//                 Serial.println("ESP-NOW send failed to MAC: " + String(formattedMac));
//             }
//         }
//     } else {
//         Serial.println("Its my ID");
//     }
// }


// void startDiscoveryBroadcast() {
//     uint8_t mac[6];
//     WiFi.macAddress(mac);  
    
//     myData.type = 'D';
//     memcpy(myData.mac, mac, 6);  

//     broadcast(myData);
// }


// bool isKnownPeer(const uint8_t* mac) {
// for (int i = 0; i < peerCount; i++) {
//     if (memcmp(knownPeers[i], mac, 6) == 0) {
//     return true;
//     }
// }
// return false;
// }

// bool isBroadcastMac(const uint8_t* mac) {
//     for (int i = 0; i < 6; i++) {
//         if (mac[i] != 0xFF) {
//             return false;
//         }
//     }
//     return true;
// }


// void addPeer(const uint8_t* mac) {
// if (peerCount < MAX_PEERS) {
//     memcpy(knownPeers[peerCount], mac, 6);

//     esp_now_peer_info_t peerInfo;
//     memcpy(peerInfo.peer_addr, mac, 6);
//     peerInfo.channel = 0;
//     peerInfo.encrypt = false;
//     peerInfo.ifidx = WIFI_IF_STA;

//     esp_err_t result = esp_now_add_peer(&peerInfo);
//     if (result == ESP_OK) {
//         char formattedMac[20];  
//         formatMacAddress(mac, formattedMac, sizeof(formattedMac));
//         Serial.println("Peer added successfully with MAC: " + String(formattedMac));
//         peerCount++;
//     } else {
//         Serial.println("Failed to add peer.");
//     }
// } else {
//     Serial.println("Max peer count reached. Can't add new peer.");
// }
// }
// void OnDataRecv(const esp_now_recv_info_t *recv_info, const uint8_t *incomingData, int len) {
//     if(len != sizeof(struct_message)) {
//         Serial.println("Received data of unexpected size.");
//         return;
//     }

//     struct_message receivedData;
//     memcpy(&receivedData, incomingData, sizeof(receivedData));
//     Serial.println("Received message type: " + String(receivedData.type));

//     if (receivedData.type == 'D') { 
//         char formattedMac[20];  
//         formatMacAddress(receivedData.mac, formattedMac, sizeof(formattedMac));

//         if(isBroadcastMac(receivedData.mac)) {
//             Serial.println("Received MAC is a broadcast MAC. Not adding as peer.");
//         } else {
//             Serial.println("Received Discovery message from MAC: " + String(formattedMac));
//             Serial.println(receivedData.)
//             if(!isKnownPeer(receivedData.mac)) {
//                 addPeer(receivedData.mac);
//             }
//         }
//     } else if (receivedData.type == 'C') {  
//         Serial.println("Received command: " + String(receivedData.command));
//         String com = String(receivedData.command);
//         if (receivedData.msgID == lastReceivedMsgID) {
//             Serial.println("same Data");
//             return;
//         }
//         lastReceivedMsgID = receivedData.msgID;

//         if (com.charAt(0) == myID.charAt(0)) {
//             if (com.substring(1).equals("ON")) {
//                 digitalWrite(LED_BUILTIN, HIGH);
//             } else if (com.substring(1).equals("OFF")) {
//                 digitalWrite(LED_BUILTIN, LOW);
//             }
//         } else {
//             if (receivedData.hopCount < MAX_HOPS) {
//                 receivedData.hopCount++;
//                 sendCommand(receivedData);
//             } else {
//                 Serial.println("Dropping message due to excessive hops.");
//             }
//         }
//     } else {
//         Serial.println("Received unknown message type.");
//     }
// }


// void formatMacAddress(const uint8_t *macAddr, char *buffer, int maxLength) {
//     snprintf(buffer, maxLength, "%02x:%02x:%02x:%02x:%02x:%02x", macAddr[0], macAddr[1], macAddr[2], macAddr[3], macAddr[4], macAddr[5]);
// }


// void broadcast(const struct_message &message) {
//     uint8_t broadcastAddress[] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
//     esp_now_peer_info_t peerInfo = {};
//     memcpy(&peerInfo.peer_addr, broadcastAddress, 6);
//     if (!esp_now_is_peer_exist(broadcastAddress)) {
//         esp_now_add_peer(&peerInfo);
//     }

//     esp_err_t result = esp_now_send(broadcastAddress, (const uint8_t *)&message, sizeof(message));

//     if (result == ESP_OK) {
//         Serial.println("Broadcast message success");
//     } else if (result == ESP_ERR_ESPNOW_NOT_INIT) {
//         Serial.println("ESP-NOW not Init.");
//     } else if (result == ESP_ERR_ESPNOW_ARG) {
//         Serial.println("Invalid Argument");
//     } else if (result == ESP_ERR_ESPNOW_INTERNAL) {
//         Serial.println("Internal Error");
//     } else if (result == ESP_ERR_ESPNOW_NO_MEM) {
//         Serial.println("ESP_ERR_ESPNOW_NO_MEM");
//     } else if (result == ESP_ERR_ESPNOW_NOT_FOUND) {
//         Serial.println("Peer not found.");
//     } else {
//         Serial.println("Unknown error");
//     }
// }

// void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
//     if (status == ESP_NOW_SEND_SUCCESS) {
//         Serial.println("Send success");
//     } else {
//         Serial.println("Send failed");
//     }
// }

