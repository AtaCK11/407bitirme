#define ECG_BUFFER_SIZE 256  // Buffer size for ECG
#define HR_BUFFER_SIZE 5     // Buffer size for heart rate

#define QUEUE_SIZE 10        // Queue size of our RTOS

// Sample rates
#define ECG_SAMPLE_RATE 1
#define HR_SAMPLE_RATE 1
#define HR_PUSH_SAMPLE_RATE 2000
#define AIRQ_SAMPLE_RATE 10000
#define TEMP_SAMPLE_RATE 5000

#define CUTOFF_FREQUENCY 0.5

#include <Wire.h>
#include "MAX30100_PulseOximeter.h"

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

void setup() {
  Serial.begin(115200);

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

  if (!pox.begin()) {
    Serial.println("FAILED");
    for(;;);
  }
  pox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);
  delay(100);

  // setup low pass filter for heart rate data push function
  setupButterworthFilter(HR_PUSH_SAMPLE_RATE, CUTOFF_FREQUENCY);

  // Create RTOS tasks
  xTaskCreate(ecgTask, "ECG Task", 2048, NULL, 2, NULL);
  xTaskCreate(hrTask, "HR Task", 4096, NULL, 3, NULL);
  xTaskCreate(hrPushTask, "HR Push Task", 2048, NULL, 2, NULL);
  xTaskCreate(temperatureTask, "Temperature Task", 2048, NULL, 1, NULL);
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

void temperatureTask(void *parameter) {
  while (true) {
    SensorData tempData = {"Temperature", random(36, 38)};
    xQueueSend(sensorQueue, &tempData, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(TEMP_SAMPLE_RATE)); // sample rate delay
  }
}

void airQualityTask(void *parameter) {
    while (true) {
      SensorData airQualityData = {"Air Quality", random(0, 500)};
      xQueueSend(sensorQueue, &airQualityData, portMAX_DELAY);
      vTaskDelay(pdMS_TO_TICKS(AIRQ_SAMPLE_RATE)); // sample rate delay
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
