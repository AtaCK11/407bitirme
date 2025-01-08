

#include <esp_now.h>
#include <WiFi.h>

#include <Wire.h>
#include <MAX30100_PulseOximeter.h>
#include <MQUnifiedsensor.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT.h>

#define ECG_BUFFER_SIZE 256  // Buffer size for ECG
#define HR_BUFFER_SIZE 5     // Buffer size for heart rate

#define QUEUE_SIZE 16        // Queue size of our RTOS

// Sample rates
#define ECG_SAMPLE_RATE 1
#define ECG_DATA_TX_RATE 200
#define HR_SAMPLE_RATE 1
#define HR_PUSH_SAMPLE_RATE 500
#define MQ135_SAMPLE_RATE 8000
#define TEMP_SAMPLE_RATE 5000
#define ROOM_SAMPLE_RATE 5000


// Calibration samples
#define MQ135_CALIBRATION_SAMPLES 20
#define MQ135_CALIBRATION_DELAY 200 // ms between samples

// Sensor Pins
#define MQ135_PIN 35
#define DS18B20_PIN 33
#define DHT11_PIN 32
#define BUZZER_PIN 5
#define RGB_R_PIN 14
#define RGB_G_PIN 12
#define RGB_B_PIN 13

 
// Defaults
#define RatioMQ135CleanAir 3.6

#define CUTOFF_FREQUENCY 0.5

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

enum SensorType {
  ECG,
  HR
};

float previousOutput = 0.0;  // feedback filtered value
float alpha = 0.0;  // filter coefficient

constexpr float math_PI = 3.141592653589793f;
void setupButterworthFilter(float sampleRate, float cutoffFreq) {
  float RC = 1.0 / (2 * math_PI * cutoffFreq);
  float dt = sampleRate / 1000.0;  // Convert ms to seconds
  alpha = dt / (RC + dt);
}

// low pass filter
float butterworthFilter(float input) {
  float output = alpha * input + (1 - alpha) * previousOutput;
  previousOutput = output;
  return output;
}

// RingBuffer structure
struct RingBuffer {
  float* buffer;
  int head;       // write pos
  int tail;       // read pos
  int count;
  int capacity;   // max cap

  void init(SensorType sensor) {
    if (sensor == SensorType::ECG) { capacity = ECG_BUFFER_SIZE; }
    else if (sensor == SensorType::HR) { capacity = HR_BUFFER_SIZE; }
    buffer = new float[capacity];
    head = 0;
    tail = 0;
    count = 0;
  }

  void push(float value) {
    if (count == capacity) {
      // overwrite oldest
      tail = (tail + 1) % capacity;
    } else {
      count++;
    }
    buffer[head] = value;
    head = (head + 1) % capacity;
  }

  float pop() {
    if (count == 0) {
      return -1; // underflow
    }
    float value = buffer[tail];
    tail = (tail + 1) % capacity;
    count--;
    return value;
  }

  bool isEmpty() {
    return count == 0;
  }

  void clear() {
    delete[] buffer;
    buffer = nullptr;
    head = 0;
    tail = 0;
    count = 0;
    capacity = 0;
  }
};

// Global RingBuffers
RingBuffer ecgBuffer;
RingBuffer hrBuffer;

// RTOS queue for sensor data
QueueHandle_t sensorQueue;

// SensorData structure
struct SensorData {
  char sensorType[36];
  float value;
};

// esp receiver
uint8_t receiverMACAddress[] = {0xA0, 0xB7, 0x65, 0x28, 0xCE, 0xA0};

static const char* PMK_KEY_STR = "U@Kw%nV&PzbRK2WM";
static const char* LMK_KEY_STR = "Ruth:WqH9@j%h5qg";

// sensors
PulseOximeter pox;
MQUnifiedsensor MQ135("ESP32", 5, 12, MQ135_PIN, "MQ-135");

OneWire oneWire(DS18B20_PIN);
DallasTemperature sensors(&oneWire);

DHT dht(DHT11_PIN, DHT11);


void setRGBColor(int red, int green, int blue)
{
  debugPrintln("\n");
  analogWrite(RGB_R_PIN, red);
  analogWrite(RGB_G_PIN, green);
  analogWrite(RGB_B_PIN, blue);
}

void onPacketSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  debugPrint("\nLast Packet:\t");
  debugPrintln(status == ESP_NOW_SEND_SUCCESS ? "Sent" : "Fail");
}

void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);

  delay(500);

  pinMode(RGB_R_PIN, OUTPUT);
  pinMode(RGB_G_PIN, OUTPUT);
  pinMode(RGB_B_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  // Print the device's MAC address
  debugPrint("Sender MAC Address: ");
  debugPrintln(WiFi.macAddress());

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    debugPrintln("Error initializing ESP-NOW");
    setRGBColor(0, 255, 255); // RED
    while (true);
  }

  delay(1000);
  // ESP NOW
  //esp_now_register_recv_cb(OnDataRecv);
  esp_now_set_pmk((uint8_t *)PMK_KEY_STR);

  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, receiverMACAddress, 6);
  peerInfo.encrypt = true;
  for (uint8_t i = 0; i < 16; i++) {
    peerInfo.lmk[i] = LMK_KEY_STR[i];
  }
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    debugPrintln("Failed to add peer");
    setRGBColor(0, 255, 255); // RED
    while (true);
  }

  esp_now_register_send_cb(onPacketSent);

  // RTOS queue
  sensorQueue = xQueueCreate(QUEUE_SIZE, sizeof(SensorData));
  if (sensorQueue == NULL) {
    debugPrintln("Error creating the queue");
    setRGBColor(0, 255, 255); // RED
    while (true);
  }
  delay(100);

  // Initialize buffers
  ecgBuffer.init(SensorType::ECG);
  hrBuffer.init(SensorType::HR);

  // Initialize sensors
  // HR
  if (!pox.begin()) 
  {
    for (int i = 0; i < 10; i++) 
    {
      debugPrintln("MAX30100 ---> FAILED");
      debugPrintln("MAX30100 ---> FAILED");
    }
    setRGBColor(0, 255, 255); // RED
    while (true);
  }
  pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);
  delay(100);

  // MQ135
  MQ135.setRegressionMethod(1);
  MQ135.init();
  debugPrint("Calibrating, please wait...");
  float r0Values[MQ135_CALIBRATION_SAMPLES];
  for (int i = 0; i < MQ135_CALIBRATION_SAMPLES; i++) 
  {
    MQ135.update();
    r0Values[i] = MQ135.calibrate(RatioMQ135CleanAir);
    delay(MQ135_CALIBRATION_DELAY);
  }

  // median R0 value
  std::sort(r0Values, r0Values + MQ135_CALIBRATION_SAMPLES);
  float medianR0 = r0Values[MQ135_CALIBRATION_SAMPLES / 2];
  MQ135.setR0(medianR0);
  debugPrint("Calibration done. R0 = ");
  debugPrintln(medianR0);

  if(isinf(medianR0)) 
  {
    setRGBColor(0, 255, 255); // RED
    debugPrintln("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply"); while(1);
  }
  if(medianR0 == 0){
    setRGBColor(0, 255, 255); // RED
    debugPrintln("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply"); while(1);
  }

  // DS18B20
  sensors.begin();

  // DHT11
  dht.begin();

  // sensor init end
  // setup low pass filter for heart rate data push function
  setupButterworthFilter(HR_PUSH_SAMPLE_RATE, CUTOFF_FREQUENCY);

  // Create RTOS tasks
  xTaskCreate(ecgTask, "ECG Task", 2048, NULL, 3, NULL);
  xTaskCreate(hrTask, "HR Task", 4096, NULL, 3, NULL);
  xTaskCreate(hrPushTask, "HR Push Task", 2048, NULL, 2, NULL);
  //xTaskCreate(sendECGTask, "Send ECG Data Task", 2048, NULL, 2, NULL);
  xTaskCreate(bodyTemperatureTask, "Body Temperature Task", 2048, NULL, 1, NULL);
  xTaskCreate(roomTempNHumTask, "Room Temperature n Humidity Task", 2048, NULL, 1, NULL);
  xTaskCreate(airQualityTask, "Air Quality Task", 2048, NULL, 1, NULL);
  xTaskCreate(processTask, "Process Task", 2048, NULL, 1, NULL);
  //xTaskCreate(debugPrintTask, "Debug Task", 2048, NULL, 1, NULL);
}

void ecgTask(void *parameter) {
  while (true) {
    float ecgData = random(60,100); // random number for now
    ecgBuffer.push(ecgData);
    vTaskDelay(pdMS_TO_TICKS(ECG_SAMPLE_RATE)); // sample rate delay
  }
}

void hrTask(void *parameter) {
  while (true) {
    pox.update();
    vTaskDelay(pdMS_TO_TICKS(HR_SAMPLE_RATE)); // sample rate delay
  }
}

void hrPushTask(void *parameter) {
  while (true) {
    float smoothedHeartRate = butterworthFilter(pox.getHeartRate());
    Serial.print("Filtered Heart Rate: ");
    Serial.println(smoothedHeartRate); Serial.println();
    Serial.print("Normal Heart Rate: ");
    Serial.println(pox.getHeartRate());

    hrBuffer.push(smoothedHeartRate);

    if (smoothedHeartRate > 150)
    {
      digitalWrite(BUZZER_PIN, HIGH);
    } else { digitalWrite(BUZZER_PIN, LOW); }

    SensorData hrSensorData = {"<HR>|SmoothedHeartRate|", smoothedHeartRate};
    esp_err_t result = esp_now_send(receiverMACAddress, (uint8_t *)&hrSensorData, sizeof(hrSensorData));
    if (result == ESP_OK) {
      debugPrint("Sent Smoothed Heart Rate: ");
      debugPrintln(smoothedHeartRate);
    } else {
      debugPrintln("Error sending smoothed heart rate data");
    }

    vTaskDelay(pdMS_TO_TICKS(HR_PUSH_SAMPLE_RATE)); // sample rate delay
  }
}

void roomTempNHumTask(void *parameter) {
  while (true) {

    // reading the values generates an average 210ms delay, might change to other DHT sensors
    SensorData tempData = {"<Room>|Humidity|", dht.readHumidity()};
    xQueueSend(sensorQueue, &tempData, portMAX_DELAY);

    tempData = {"<Room>|Temperature|", dht.readTemperature()};
    xQueueSend(sensorQueue, &tempData, portMAX_DELAY);

    vTaskDelay(pdMS_TO_TICKS(ROOM_SAMPLE_RATE)); // sample rate delay
  }
}

void bodyTemperatureTask(void *parameter) {
  while (true) {
    sensors.requestTemperatures();
    float cTemp = sensors.getTempCByIndex(0);
    SensorData tempData = {"<Body>|Temperature|", cTemp};

    if (cTemp > 39 || cTemp < 32)
    {
      setRGBColor(0, 255, 0); // MAGENTA
    } else { setRGBColor(255, 0, 255); }

    xQueueSend(sensorQueue, &tempData, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(TEMP_SAMPLE_RATE)); // sample rate delay
  }
}

void airQualityTask(void *parameter) {
  while (true) {
    MQ135.update();

    // CO
    MQ135.setA(605.18); MQ135.setB(-3.937);
    SensorData airQualityData = {"<Room>|CO|", MQ135.readSensor()};
    xQueueSend(sensorQueue, &airQualityData, portMAX_DELAY);

    // Alcohol
    MQ135.setA(77.255); MQ135.setB(-3.18);
    airQualityData = {"<Room>|Alcohol|", MQ135.readSensor()};
    xQueueSend(sensorQueue, &airQualityData, portMAX_DELAY);

    // CO2
    MQ135.setA(110.47); MQ135.setB(-2.862);
    airQualityData = {"<Room>|CO2|", MQ135.readSensor()};
    xQueueSend(sensorQueue, &airQualityData, portMAX_DELAY);

    // Toluene
    MQ135.setA(44.947); MQ135.setB(-3.445);
    airQualityData = {"<Room>|Toluene|", MQ135.readSensor()};
    xQueueSend(sensorQueue, &airQualityData, portMAX_DELAY);

    // NH4
    MQ135.setA(102.2); MQ135.setB(-2.473);
    airQualityData = {"<Room>|NH4|", MQ135.readSensor()};
    xQueueSend(sensorQueue, &airQualityData, portMAX_DELAY);

    // Acetone
    MQ135.setA(34.668); MQ135.setB(-3.369);
    airQualityData = {"<Room>|Acetone|", MQ135.readSensor()};
    xQueueSend(sensorQueue, &airQualityData, portMAX_DELAY);

    vTaskDelay(pdMS_TO_TICKS(MQ135_SAMPLE_RATE)); // sample rate delay
  }
}

void processTask(void *parameter) {
  SensorData receivedData;
  while (true) {
    if (xQueueReceive(sensorQueue, &receivedData, portMAX_DELAY)) { // if theres data in our queue
      esp_err_t result = esp_now_send(receiverMACAddress, (uint8_t *)&receivedData, sizeof(SensorData));
      if (result == ESP_OK) {
        debugPrint("Sent: ");
        debugPrint(receivedData.sensorType);
        debugPrint(": ");
        debugPrintln(receivedData.value);
      } else {
        debugPrintln("Error sending "); debugPrint(receivedData.sensorType);
      }
    }
  }
}

void sendECGTask(void *parameter) {
  while (true) {
    if (ecgBuffer.count == ECG_BUFFER_SIZE) {
      for (int i = 0; i < ECG_BUFFER_SIZE; i++) {
        float value = ecgBuffer.pop();
        SensorData ecgSensorData = {"<ECG>|Data Batch|", value};

        esp_err_t result = esp_now_send(receiverMACAddress, (uint8_t *)&ecgSensorData, sizeof(SensorData));
        if (result == ESP_OK) {
          debugPrint("Sent ECG Data: ");
          debugPrintln(value);
        } else {
          debugPrintln("Error sending ECG data");
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(ECG_DATA_TX_RATE));
  }
}


void debugPrintTask(void *parameter) {
  while (true) {
    Serial.print("ECG Buffer: [");
    for (int i = 0; i < ecgBuffer.count; i++) {
      Serial.print(ecgBuffer.buffer[(ecgBuffer.tail + i) % ecgBuffer.capacity]);
      if (i != ecgBuffer.count - 1) {
        Serial.print(", ");
      }
    }
    Serial.println("]");

    // Print Heart Rate Buffer // not important
    Serial.print("Heart Rate Buffer: [");
    for (int i = 0; i < hrBuffer.count; i++) {
      Serial.print(hrBuffer.buffer[(hrBuffer.tail + i) % hrBuffer.capacity]);
      if (i != hrBuffer.count - 1) {
        Serial.print(", ");
      }
    }
    Serial.println("]");

    vTaskDelay(pdMS_TO_TICKS(13458));
  }
}

void loop() { /* RTOS handles ... */ }
