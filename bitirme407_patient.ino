

#include <esp_now.h>
#include <WiFi.h>

#include <HTTPClient.h>
//#include <ESPAsyncWebServer.h>

#include <Wire.h>
#include <MAX30100_PulseOximeter.h>
#include <MQUnifiedsensor.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT.h>

#define ECG_BUFFER_SIZE 1024  // Buffer size for ECG
#define HR_BUFFER_SIZE 5     // Buffer size for heart rate

#define QUEUE_SIZE 128        // Queue size of our RTOS

// Sample rates
#define ECG_SAMPLE_RATE 1
#define ECG_DATA_TX_RATE 1000
#define HR_SAMPLE_RATE 1
#define HR_PUSH_SAMPLE_RATE 500
#define MQ135_SAMPLE_RATE 8000
#define TEMP_SAMPLE_RATE 5000
#define ROOM_SAMPLE_RATE 5000


// Calibration samples
#define MQ135_CALIBRATION_SAMPLES 10
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

// http://192.168.4.1/register

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
  int sensorTag;
  float sensorValue;
};


enum SensorTag {
  BODY_ECG = 1,
  BODY_HR = 2,
  BODY_BO2 = 3,
  ROOM_TEMP = 4,
  ROOM_HUM = 5,
  BODY_TEMP = 6,
  ROOM_CO = 7,
  ROOM_ALCOHOL = 8,
  ROOM_CO2 = 9,
  ROOM_TOLUENE = 10,
  ROOM_NH4 = 11,
  ROOM_ACETONE = 12,
};

// esp receiver
//uint8_t myMACAddress[8];


// server properties
const char* server_ssid = "BITIRME_AP_SERVER";
const char* server_password = "12345678";

// server IP
char pc_db_ip[32] = "";
const char* server_url = "http://192.168.4.2";

// sensors
PulseOximeter pox;
MQUnifiedsensor MQ135("ESP32", 5, 12, MQ135_PIN, "MQ-135");

OneWire oneWire(DS18B20_PIN);
DallasTemperature sensors(&oneWire);

DHT dht(DHT11_PIN, DHT11);

bool wifi_connection = false;


void setRGBColor(int red, int green, int blue)
{
  debugPrintln("\n");
  analogWrite(RGB_R_PIN, red);
  analogWrite(RGB_G_PIN, green);
  analogWrite(RGB_B_PIN, blue);
}

bool register_with_server() {
  WiFiClient client;
  HTTPClient http;

  // Debugging to check pc_db_ip
  if (strlen(pc_db_ip) == 0) {
    Serial.println("Error: PC DB IP is empty!");
    return false;
  }

  char url[64];
  snprintf(url, sizeof(url), "http://%s:5000/register", pc_db_ip);
  
  // Debugging to ensure the URL is correct
  Serial.print("Connecting to URL: ");
  Serial.println(url);
  
  String payload = "{\"mac\":\"" + WiFi.macAddress() + "\",\"ip\":\"" + WiFi.localIP().toString() + "\"}";
  http.begin(client, url);
  http.addHeader("Content-Type", "application/json");
  
  int httpCode = http.POST(payload);
  http.end();

  // Check HTTP response code
  if (httpCode == HTTP_CODE_OK) {
    Serial.println("Device registered successfully.");
    return true;
  } else {
    Serial.print("Device registration failed. HTTP code: ");
    Serial.println(httpCode);
    return false;
  }
}



bool get_pc_ip_from_ap() {
  WiFiClient client;
  HTTPClient http;

  http.begin(client, "http://192.168.4.1/get_pc_ip");  // ESP32 AP IP
  int httpCode = http.GET();

  if (httpCode == HTTP_CODE_OK) {
    String response = http.getString();
    response.trim();
    response.toCharArray(pc_db_ip, sizeof(pc_db_ip));  // Copy to global buffer
    Serial.print("Received PC IP: ");
    Serial.println(pc_db_ip);
    http.end();
    return true;
  } else {
    Serial.print("Failed to get PC IP. HTTP code: ");
    Serial.println(httpCode);
    http.end();
    return false;
  }
}



void setup() {
  Serial.begin(115200);

    // Connect to Wi-Fi
  WiFi.begin(server_ssid, server_password);
  #ifdef DEBUG
  #else
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to Wi-Fi");
  wifi_connection = true;
  #endif

  Serial.print("IP: "); Serial.println(WiFi.localIP());
  Serial.print("MAC: "); Serial.println(WiFi.macAddress());
  //myMACAddress = WiFi.macAddress();

  delay(500);

  pinMode(RGB_R_PIN, OUTPUT);
  pinMode(RGB_G_PIN, OUTPUT);
  pinMode(RGB_B_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);

  delay(1000);

  #ifdef DEBUG
  #else
  while (!get_pc_ip_from_ap()) {
    Serial.println("Retrying in 3 seconds...");
    delay(3000);
  }

  delay(1000);

  while (!register_with_server()) {
    Serial.println("Retrying in 3 seconds...");
    delay(3000);
  }

  #endif

  delay(1000);

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
    //while (true);
  }
  pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);
  delay(100);

  // MQ135
  MQ135.setRegressionMethod(1);
  MQ135.init();
  debugPrint("Calibrating, please wait...");
  float calcR0 = 0;
  for(int i = 1; i<=10; i ++)
  {
    MQ135.update(); // Update data, the arduino will read the voltage from the analog pin
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
    Serial.print(".");
  }
  MQ135.setR0(calcR0/10);

  // median R0 value
  //std::sort(r0Values, r0Values + MQ135_CALIBRATION_SAMPLES);
  //float medianR0 = r0Values[MQ135_CALIBRATION_SAMPLES / 2];
  //MQ135.setR0(medianR0);
  debugPrint("Calibration done. R0 = ");
  debugPrintln(calcR0/10);

  if(isinf(calcR0)) 
  {
    setRGBColor(0, 255, 255); // RED
    debugPrintln("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply"); //while(1);
  }
  if(calcR0 == 0){
    setRGBColor(0, 255, 255); // RED
    debugPrintln("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply"); //while(1);
  }

  // DS18B20
  sensors.begin();

  // DHT11
  dht.begin();

  // sensor init end
  // setup low pass filter for heart rate data push function
  setupButterworthFilter(HR_PUSH_SAMPLE_RATE, CUTOFF_FREQUENCY);

  setRGBColor(255, 0, 255); // GREEN

  // Create RTOS tasks
  xTaskCreate(ecgTask, "ECG Task", 2048, NULL, 3, NULL);
  xTaskCreate(hrTask, "HR Task", 4096, NULL, 3, NULL);
  xTaskCreate(hrPushTask, "HR Push Task", 2048, NULL, 2, NULL);
  xTaskCreate(sendECGTask, "Send ECG Data Task", 2048, NULL, 2, NULL);
  xTaskCreate(bodyTemperatureTask, "Body Temperature Task", 2048, NULL, 1, NULL);
  xTaskCreate(roomTempNHumTask, "Room Temperature n Humidity Task", 2048, NULL, 1, NULL);
  xTaskCreate(airQualityTask, "Air Quality Task", 2048, NULL, 1, NULL);
  xTaskCreate(processTask, "Process Task", 4096, NULL, 1, NULL);
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
    //Serial.print("Filtered Heart Rate: ");
    //Serial.println(smoothedHeartRate); Serial.println();
    //Serial.print("Normal Heart Rate: ");
    //Serial.println(pox.getHeartRate());

    hrBuffer.push(smoothedHeartRate);

    if (smoothedHeartRate > 150)
    {
      digitalWrite(BUZZER_PIN, HIGH);
    } else { digitalWrite(BUZZER_PIN, LOW); }

    SensorData hrSensorData = {SensorTag::BODY_HR, smoothedHeartRate};

    vTaskDelay(pdMS_TO_TICKS(HR_PUSH_SAMPLE_RATE)); // sample rate delay
  }
}

void roomTempNHumTask(void *parameter) {
  while (true) {

    // reading the values generates an average 210ms delay, might change to other DHT sensors
    SensorData tempData = {SensorTag::ROOM_HUM, dht.readHumidity()};
    xQueueSend(sensorQueue, &tempData, portMAX_DELAY);

    tempData = {SensorTag::ROOM_TEMP, dht.readTemperature()};
    xQueueSend(sensorQueue, &tempData, portMAX_DELAY);

    Serial.println("TEMP -- HUM");
    Serial.println(dht.readTemperature());
    Serial.println(dht.readHumidity());
    vTaskDelay(pdMS_TO_TICKS(ROOM_SAMPLE_RATE)); // sample rate delay
  }
}

void bodyTemperatureTask(void *parameter) {
  while (true) {
    sensors.requestTemperatures();
    float cTemp = sensors.getTempCByIndex(0);
    SensorData tempData = {SensorTag::BODY_TEMP, cTemp};

    if (cTemp > 39 || cTemp < 32)
    {
      setRGBColor(0, 255, 0); // MAGENTA
    } else { setRGBColor(255, 0, 255); } // GREEN

    xQueueSend(sensorQueue, &tempData, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(TEMP_SAMPLE_RATE)); // sample rate delay
  }
}

void airQualityTask(void *parameter) {
  while (true) {
    MQ135.update();

    // CO
    MQ135.setA(605.18); MQ135.setB(-3.937);
    SensorData airQualityData = {SensorTag::ROOM_CO, MQ135.readSensor()};
    xQueueSend(sensorQueue, &airQualityData, portMAX_DELAY);

    // Alcohol
    MQ135.setA(77.255); MQ135.setB(-3.18);
    airQualityData = {SensorTag::ROOM_ALCOHOL, MQ135.readSensor()};
    xQueueSend(sensorQueue, &airQualityData, portMAX_DELAY);

    // CO2
    MQ135.setA(110.47); MQ135.setB(-2.862);
    airQualityData = {SensorTag::ROOM_CO2, MQ135.readSensor()+400};
    xQueueSend(sensorQueue, &airQualityData, portMAX_DELAY);

    // Toluene
    MQ135.setA(44.947); MQ135.setB(-3.445);
    airQualityData = {SensorTag::ROOM_TOLUENE, MQ135.readSensor()};
    xQueueSend(sensorQueue, &airQualityData, portMAX_DELAY);

    // NH4
    MQ135.setA(102.2); MQ135.setB(-2.473);
    airQualityData = {SensorTag::ROOM_NH4, MQ135.readSensor()};
    xQueueSend(sensorQueue, &airQualityData, portMAX_DELAY);

    // Acetone
    MQ135.setA(34.668); MQ135.setB(-3.369);
    airQualityData = {SensorTag::ROOM_ACETONE, MQ135.readSensor()};
    xQueueSend(sensorQueue, &airQualityData, portMAX_DELAY);

    vTaskDelay(pdMS_TO_TICKS(MQ135_SAMPLE_RATE)); // sample rate delay
  }
}

void processTask(void *parameter) {
  SensorData receivedData;
  while (true) {
    if (xQueueReceive(sensorQueue, &receivedData, portMAX_DELAY)) {
      float value = receivedData.sensorValue;

      // Check memory and value sanity before processing
      if (isnan(value) || isinf(value)) {
        //Serial.printf("[SKIPPED] Invalid sensor value (NaN/Inf). Tag: %d\n", receivedData.sensorTag);
        continue;
      }

      if (value < 0 && receivedData.sensorTag != SensorTag::BODY_ECG) {
        //Serial.printf("[SKIPPED] Negative value. Tag: %d Value: %.2f\n", receivedData.sensorTag, value);
        continue;
      }

      // Send from a SERIAL

      Serial.printf("[SENSOR_DATA] -> %d | %.6f\n", receivedData.sensorTag, value);

      //Serial.printf("[DEBUG] Preparing to send -> Tag: %d, Value: %.6f\n", receivedData.sensorTag, value);

      // Create JSON manually, not using String concatenation
      char payload[128];
      snprintf(payload, sizeof(payload),
               "{\"sensorTag\":%d,\"sensorValue\":%.6f}",
               receivedData.sensorTag, value);

      if (wifi_connection) {
        WiFiClient client;
        HTTPClient http;
        char url[64];
        snprintf(url, sizeof(url), "http://%s:5000/sensor_data", pc_db_ip);

        http.begin(client, url);
        http.addHeader("Content-Type", "application/json");

        int httpCode = http.POST((uint8_t *)payload, strlen(payload));
        String response = http.getString();

        if (httpCode == HTTP_CODE_OK) {
          Serial.printf("[SUCCESS] Sent Tag: %d, Value: %.2f\n", receivedData.sensorTag, value);
        } else {
          Serial.printf("[FAIL] Code: %d | Response: %s\n", httpCode, response.c_str());
        }

        http.end();
      }

    } else {
      vTaskDelay(10 / portTICK_PERIOD_MS);
    }
  }
}


void sendECGTask(void *parameter) {
  while (true) {
    if (ecgBuffer.count == ECG_BUFFER_SIZE) {
      if (wifi_connection) {
        WiFiClient client;
        HTTPClient http;

        char url[64];
        snprintf(url, sizeof(url), "http://%s:5000/ecg_data", pc_db_ip);

        String jsonPayload = "{\"ecg_values\":[";

        for (int i = 0; i < ECG_BUFFER_SIZE; i++) {
          float value = ecgBuffer.pop();
          jsonPayload += String(value, 4);
          if (i < ECG_BUFFER_SIZE - 1) {
            jsonPayload += ",";
          }
        }

        jsonPayload += "]}";

        http.begin(client, url);
        http.addHeader("Content-Type", "application/json");

        int httpCode = http.POST(jsonPayload);
        String response = http.getString();

        if (httpCode == HTTP_CODE_OK) {
          Serial.println("[SUCCESS] ECG data sent.");
        } else {
          Serial.printf("[FAIL] Code: %d | Response: %s\n", httpCode, response.c_str());
        }

        http.end();
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
