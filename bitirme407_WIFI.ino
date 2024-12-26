#define ECG_BUFFER_SIZE 256  // Buffer size for ECG
#define HR_BUFFER_SIZE 5     // Buffer size for heart rate

#define QUEUE_SIZE 16        // Queue size of our RTOS

// Sample rates
#define ECG_SAMPLE_RATE 1
#define HR_SAMPLE_RATE 1
#define HR_PUSH_SAMPLE_RATE 2000
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

// Defaults
#define RatioMQ135CleanAir 3.6

#define CUTOFF_FREQUENCY 0.5

#include <esp_now.h>
#include <WiFi.h>

#include <Wire.h>
#include <MAX30100_PulseOximeter.h>
#include <MQUnifiedsensor.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT.h>

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
  const char *sensorType;
  float value;
};

// sensors
PulseOximeter pox;
MQUnifiedsensor MQ135("ESP32", 5, 12, MQ135_PIN, "MQ-135");

OneWire oneWire(DS18B20_PIN);
DallasTemperature sensors(&oneWire);

DHT dht(DHT11_PIN, DHT11);


void setup() {
  Serial.begin(115200);

  WiFi.mode(WIFI_STA);

  delay(500);

  // Print the device's MAC address
  Serial.print("Sender MAC Address: ");
  Serial.println(WiFi.macAddress());

  // Initialize ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // RTOS queue
  sensorQueue = xQueueCreate(QUEUE_SIZE, sizeof(SensorData));
  if (sensorQueue == NULL) {
    Serial.println("Error creating the queue");
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
      Serial.println("MAX30100 ---> FAILED");
      Serial.println("MAX30100 ---> FAILED");
    }
  }
  pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);
  delay(100);

  // MQ135
  MQ135.setRegressionMethod(1);
  MQ135.init();
  Serial.print("Calibrating, please wait...");
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
  Serial.print("Calibration done. R0 = ");
  Serial.println(medianR0);

  if(isinf(medianR0)) {Serial.println("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply"); while(1);}
  if(medianR0 == 0){Serial.println("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply"); while(1);}

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
  xTaskCreate(bodyTemperatureTask, "Body Temperature Task", 2048, NULL, 1, NULL);
  xTaskCreate(roomTempNHumTask, "Room Temperature n Humidity Task", 2048, NULL, 1, NULL);
  xTaskCreate(airQualityTask, "Air Quality Task", 2048, NULL, 1, NULL);
  xTaskCreate(processTask, "Process Task", 2048, NULL, 1, NULL);
  xTaskCreate(debugPrintTask, "Debug Task", 2048, NULL, 1, NULL);
}

void ecgTask(void *parameter) {
  while (true) {
    float ecgData = random(60, 100); // random number for now
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
    Serial.println(smoothedHeartRate);

    hrBuffer.push(smoothedHeartRate);

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
    SensorData tempData = {"<Body>|Temperature|", sensors.getTempCByIndex(0)};
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
      Serial.print(receivedData.sensorType);
      Serial.print(": ");
      Serial.println(receivedData.value);
    }
  }
}

void debugPrintTask(void *parameter) {
  while (true) {
    // Print ECG Buffer
    Serial.print("ECG Buffer: [");
    for (int i = 0; i < ecgBuffer.count; i++) {
      Serial.print(ecgBuffer.buffer[(ecgBuffer.tail + i) % ecgBuffer.capacity]);
      if (i != ecgBuffer.count - 1) {
        Serial.print(", ");
      }
    }
    Serial.println("]");

    // Print Heart Rate Buffer
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
