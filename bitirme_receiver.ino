
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#define DEBUG

#ifdef DEBUG
  #define debugPrint(x) Serial.print(x)
  #define debugPrintln(x) Serial.println(x)
  #define debugPrintfHex(x) Serial.printf("%02X", x)
#else
  #define debugPrint(x)
  #define debugPrintln(x)
  #define debugPrintfHex(x)
#endif


struct SensorData {
  char sensorType[36];
  float value;
};

static const char* PMK_KEY_STR = "U@Kw%nV&PzbRK2WM";
static const char* LMK_KEY_STR = "Ruth:WqH9@j%h5qg";

uint8_t peer_mac_addresses[1][6] = {
  {0xE0, 0xE2, 0xE6, 0x62, 0xFF, 0x28} // Device 1 MAC
  /////////// ...
};

// Create a struct_message called myData
SensorData sensorData;

void onDataRecv(const esp_now_recv_info_t* messageInfo, const uint8_t* incomingData, int len) {
  // Debug Print: MAC Address of Sender
  debugPrint("Received packet from MAC: ");
  for (int i = 0; i < 6; i++) {
    debugPrintfHex(messageInfo->src_addr[i]);
    if (i < 5) debugPrint(":");
  }
  debugPrintln();

  debugPrint("Raw Data Length: ");
  debugPrintln(len);

  SensorData receivedData;
  memcpy(&receivedData, incomingData, sizeof(receivedData));

  debugPrint("Received Sensor Type: ");
  debugPrintln(receivedData.sensorType);
  debugPrint("Received Value: ");
  debugPrintln(receivedData.value);
}

uint8_t senderMAC[] = {0xE0, 0xE2, 0xE6, 0x62, 0xFF, 0x28}; // Replace with sender's MAC address
void addSenderAsPeer() {
  for (int i = 0; i < 1; i++) {
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, peer_mac_addresses[i], 6);
    peerInfo.encrypt = true;

    for (uint8_t i = 0; i < 16; i++) {
      peerInfo.lmk[i] = LMK_KEY_STR[i];
    }

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
      debugPrintln("Failed to add sender as a peer.");
    } else {
      debugPrintln("Sender added as a peer.");
    }
  }
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);
  uint8_t baseMac[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  if (ret == ESP_OK) {
    Serial.printf("%02X:%02X:%02X:%02X:%02X:%02X\n",
      baseMac[0], baseMac[1], baseMac[2],
      baseMac[3], baseMac[4], baseMac[5]);
  } else {
    debugPrintln("Failed to read MAC address");
  }

  if (esp_now_init() != ESP_OK) {
    debugPrintln("ESP-NOW Initialization Failed.");
    return;
  } else {
    debugPrintln("ESP-NOW Initialized Successfully.");
  }

  esp_now_set_pmk((uint8_t *)PMK_KEY_STR);
  addSenderAsPeer();
  
  esp_now_register_recv_cb(onDataRecv);
}
 
void loop() {

}