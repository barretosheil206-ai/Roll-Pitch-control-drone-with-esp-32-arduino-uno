//MAC address:
//E-1 = F4:2D:C9:7E:E7:48
//E-2 = 1C:C3:AB:BD:CE:E0

#include <esp_now.h>
#include <WiFi.h>

//Controller variables potentiometers:
const int potPin1 = 33;
const int potPin2 = 32;
const int potPin3 = 35;
const int potPin4 = 34;

//Controller variables:
float set_val;
float Kp;
float Ki;
float Kd;

//MAC address for the reciever:
uint8_t broadcastAddress[] = { 0x1C, 0xC3, 0xAB, 0xBD, 0xCE, 0xE0 };

//Define data structure: (Put all the variables you are going to send/recieve, use different name than the one above.)
typedef struct struct_message {
  float a;
  float b;
  float c;
  float d;
} struct_message;

//Create a structured data object:
struct_message myData;

esp_now_peer_info_t peerInfo;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  analogReadResolution(12);  // ESP32 default is 12-bit (0-4095)

  //Put esp into station mode:
  WiFi.mode(WIFI_MODE_STA);

  //Initialise esp-now:
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initialising esp_now");
  }

  // Register send callback:
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
}

void loop() {
  // put your main code here, to run repeatedly:

  //Format structured data:
  //Find pot values:
  set_val = mapFloat(analogRead(potPin1), 0., 4095., -15., 15.);
  Kp = mapFloat(analogRead(potPin2), 0., 4095., 0., 4.);
  Ki = mapFloat(analogRead(potPin3), 0., 4095., 0., 2.);
  Kd = mapFloat(analogRead(potPin4), 0., 4095., 0., 4.);

  //Print the controller values:
  //Serial.println(len);
  Serial.print("Set Angle: ");
  Serial.print(set_val);
  Serial.print(", ");
  Serial.print("Kp: ");
  Serial.print(Kp);
  Serial.print(", ");
  Serial.print("Ki: ");
  Serial.print(Ki);
  Serial.print(", ");
  Serial.print("Kd: ");
  Serial.println(Kd);
  //Assign pot values:
  myData.a = set_val;
  myData.b = Kp;
  myData.c = Ki;
  myData.d = Kd;

  // Send message via ESP-NOW
  esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *)&myData, sizeof(myData));

  if (result == ESP_OK) {
    //Serial.println("Sending confirmed");  //Remove if needed
  } else {
    Serial.println("Sending error");
  }
  delay(1);
}

//Define custom functions:

void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  //Serial.print("\r\nLast Packet Send Status:\t");  //Remove if needed
  //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");  //Remove if needed
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Code made by Sheil...