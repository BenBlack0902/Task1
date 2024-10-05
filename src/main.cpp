// README
// I did not succeed to implement the fastbeat, but a heartbit i did succeed. THe problem which i encounter with fastbeat was that during upload my esp often "crashed" in the sense that I lost the connection with COM4 (then i needed esp restart per button + dry run + taking out and again in the USB)
// Since my logic analyzer does not work I had to work with a workaround, instead of producing an output on some designeted pin (example 18) I did the heartbeat on the built in LED of my esp32

// Therefore:
//Fastbeat Task: Runs every 1 ms and "toggles" the fastbeat pin.                          NOT DONE
//Heartbeat Task: Runs every 200 ms and "toggles" the heartbeat pin.                      DONE under condition that it is fine that the LED is beating
//DHT11 Task: Runs every 2 seconds to read and send DHT11 sensor data to the serial task. DONE
//Button Task: Monitors the button and sends io24m025 pressed.                            DONE
//Serial Task: All interaction with the serial monitor happens here.                      DONE



#include <Arduino.h>
#include <DHT.h>
#include <FreeRTOS.h>

// Pin Definitions
#define DHTPIN 2        // dht pin
#define DHTTYPE DHT11   // dht sensor type
#define btnPin 0        // button pin
#define ledPin 38       // led pin
#define TOGGLE_PIN 21   // pin for fast toggling (not used yet)

DHT dht(DHTPIN, DHTTYPE);  // dht sensor init

// Define an event group
EventGroupHandle_t eventGroup;

// Event flags
#define USB_READY_BIT (1 << 0)  // flag for usb ready
#define BUTTON_PRESSED_BIT (1 << 1)  // flag for button press

// Queue for sending data
QueueHandle_t serialQueue;  // for passing messages between tasks

// ISR for button press
void IRAM_ATTR buttonISR() {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  // set flag for button pressed
  xEventGroupSetBitsFromISR(eventGroup, BUTTON_PRESSED_BIT, &xHigherPriorityTaskWoken);

  // Yield if a higher task is waiting
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// Heartbeat Task: blink led every 200ms
void TaskHeartbeat(void *pvParameters) {
  pinMode(ledPin, OUTPUT);  // led as output cuz no logic analyzer
  
  for (;;) {
    // toggle led color green/on and off
    static bool isOn = false;

    if (isOn) {
      neopixelWrite(ledPin, 0, 0, 0);  // led off
    } else {
      neopixelWrite(ledPin, 0, 255, 0);  // led green on
    }

    isOn = !isOn;  // flip state

    // wait 200ms
    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

// Tasks
void TaskDHT11(void *pvParameters);
void TaskSerial(void *pvParameters);
void TaskButton(void *pvParameters);

void setup() {
  Serial.begin(115200);  // start serial

  // Initialize DHT11 sensor
  dht.begin();

  // setup button pin with interrupt
  pinMode(btnPin, INPUT_PULLUP);  // button with pullup resistor
  attachInterrupt(digitalPinToInterrupt(btnPin), buttonISR, FALLING);  // interrupt on press

  // Create an event group
  eventGroup = xEventGroupCreate();  // group for flags

  // Create a queue for passing messages to serial task
  serialQueue = xQueueCreate(10, sizeof(String));  // queue size 10

  // USB ready flag is set
  xEventGroupSetBits(eventGroup, USB_READY_BIT);

  // Create tasks
  xTaskCreatePinnedToCore(TaskHeartbeat, "HeartbeatTask", 4096, NULL, 1, NULL, 0);  // heartbeat task
  xTaskCreatePinnedToCore(TaskDHT11, "DHT11Task", 2048, NULL, 1, NULL, 0);  // DHT11 reading task
  xTaskCreatePinnedToCore(TaskSerial, "SerialTask", 2048, NULL, 1, NULL, 0);  // serial task
  xTaskCreatePinnedToCore(TaskButton, "ButtonTask", 2048, NULL, 1, NULL, 0);  // button task
}

void loop() {
  // nothing in loop, all FreeRTOS tasks handle it
}

// DHT11 Task: Read temp/humidity every 2 sec and send to serial task
void TaskDHT11(void *pvParameters) {
  for (;;) {
    // wait till USB is ready
    xEventGroupWaitBits(eventGroup, USB_READY_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

    // read dht sensor values
    float humidity = dht.readHumidity();
    float temperature = dht.readTemperature();  // Celsius

    // Check if readings are valid
    if (isnan(humidity) || isnan(temperature)) {
      String errorMsg = "Failed to read from DHT11 sensor!";
      xQueueSend(serialQueue, &errorMsg, portMAX_DELAY);  // Send error to serial task
    } else {
      // Format the data
      String dataString = "H " + String(humidity) + "%; T " + String(temperature) + "C";
      xQueueSend(serialQueue, &dataString, portMAX_DELAY);  // Send formatted data to serial task
    }

    // wait 2 sec before next reading
    vTaskDelay(2000 / portTICK_PERIOD_MS);  
  }
}

// Serial Task: Print whatever is in the queue to serial monitor
void TaskSerial(void *pvParameters) {
  String receivedData;
  for (;;) {
    // Wait for data from DHT11 or button
    if (xQueueReceive(serialQueue, &receivedData, portMAX_DELAY) == pdPASS) {
      Serial.println(receivedData);  // print to serial
    }
    vTaskDelay(10 / portTICK_PERIOD_MS);  // small delay
  }
}

// Button Task: Waits for button press and sends message when button pressed
void TaskButton(void *pvParameters) {
  for (;;) {
    // wait for button press flag
    xEventGroupWaitBits(eventGroup, BUTTON_PRESSED_BIT, pdTRUE, pdFALSE, portMAX_DELAY);

    // button pressed, send user ID
    String userID = "io23m025 - Benjamin Karic";
    xQueueSend(serialQueue, &userID, portMAX_DELAY);

    // Clear the BUTTON_PRESSED_BIT after handling
    xEventGroupClearBits(eventGroup, BUTTON_PRESSED_BIT);
  }
}
