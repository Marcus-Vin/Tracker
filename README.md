# Tracker

Traffic Real-time Accident Crash Kit for Emergency Response (TRACKER) is a embedded system project that can indetify an accident and send a message to a emergency contact. Futhermore, a TRACKER can also communicate with another TRACKER by ESP-NOW protocol.

## Technologies
We built this project using ESP32 using the following modules:
- Ublox NEO-6MV2
- MPU-6050

## About TRACKER communication
The TRACKER uses the GSM module to send a message to a emergency contact of the user when identifies an accident. More than this, it uses a mesh network to communicate with the nearests TRACKERs, making useful when GSM module doesn't have signal to be used. 
