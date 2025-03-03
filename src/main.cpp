// -------------------- Configuration --------------------
#define MODULE_MODE_SENDER       // Uncomment to compile as a sender
// #define MODULE_MODE_RECEIVER   // Uncomment to compile as a receiver

// Set the module's octave (0-8)
#define MODULE_OCTAVE 4

#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <HardwareTimer.h>
#include <STM32FreeRTOS.h>
#include <atomic>
#include <stm32l4xx_hal.h>
#include <stm32l4xx_hal_can.h>
#include <stm32l4xx_hal_rcc.h>
#include <stm32l4xx_hal_gpio.h>
#include <stm32l4xx_hal_cortex.h>
#include <ES_CAN.h>   // Provided CAN library

// -------------------- FreeRTOS Queues and Semaphores --------------------
QueueHandle_t msgInQ  = NULL;  // For incoming messages
QueueHandle_t msgOutQ = NULL;  // For outgoing messages
SemaphoreHandle_t CAN_TX_Semaphore = NULL;  // Counting semaphore for TX mailboxes

// We store the latest received message in sysState, protected by sysState.mutex
struct {
  std::bitset<32> inputs;
  SemaphoreHandle_t mutex;
  uint8_t RX_Message[8];  // Global buffer for the latest incoming CAN frame
} sysState;

// Constants
const char* noteNames[12] = {
  "C4", "C#4", "D4", "D#4", "E4", "F4", 
  "F#4", "G4", "G#4", "A4", "A#4", "B4"
};

const uint32_t stepSizes[12] = {
  51076056, // C4
  54113197, // C#4
  57330935, // D4
  60740009, // D#4
  64351798, // E4
  68178356, // F4
  72232452, // F#4
  76527617, // G4
  81078186, // G#4
  85899345, // A4 (440 Hz)
  91007186, // A#4
  96418755  // B4
};

// Pin definitions
const int RA0_PIN = D3;
const int RA1_PIN = D6;
const int RA2_PIN = D12;
const int REN_PIN = A5;

const int C0_PIN = A2;
const int C1_PIN = D9;
const int C2_PIN = A6;
const int C3_PIN = D1;
const int OUT_PIN = D11;

const int OUTL_PIN = A4;
const int OUTR_PIN = A3;

const int JOYY_PIN = A0;
const int JOYX_PIN = A1;

const int DEN_BIT = 3;
const int DRST_BIT = 4;
const int HKOW_BIT = 5;
const int HKOE_BIT = 6;

// Global variable to store current step size for audio generation
volatile uint32_t currentStepSize = 0;

// -------------------- Knob Class --------------------
class Knob {
private:
  std::atomic<int32_t> rotation;
  uint8_t prevState;
  int8_t lastDirection;
  int32_t lowerLimit, upperLimit;
public:
  Knob(int32_t minVal = 0, int32_t maxVal = 8)
    : rotation(0), prevState(0b00), lastDirection(0), lowerLimit(minVal), upperLimit(maxVal) {}

  void update(uint8_t currState) {
    int32_t localRotation = rotation.load();
    if ((prevState == 0b00 && currState == 0b01) ||
        (prevState == 0b01 && currState == 0b11) ||
        (prevState == 0b11 && currState == 0b10) ||
        (prevState == 0b10 && currState == 0b00)) {
      localRotation++;
      lastDirection = 1;
    } else if ((prevState == 0b00 && currState == 0b10) ||
               (prevState == 0b10 && currState == 0b11) ||
               (prevState == 0b11 && currState == 0b01) ||
               (prevState == 0b01 && currState == 0b00)) {
      localRotation--;
      lastDirection = -1;
    } else if ((prevState == 0b00 && currState == 0b11) ||
               (prevState == 0b01 && currState == 0b10) ||
               (prevState == 0b10 && currState == 0b01) ||
               (prevState == 0b11 && currState == 0b00)) {
      localRotation += lastDirection;
    }
    if (localRotation < lowerLimit) localRotation = lowerLimit;
    if (localRotation > upperLimit) localRotation = upperLimit;
    rotation.store(localRotation);
    prevState = currState;
  }
  
  void setLimits(int32_t minVal, int32_t maxVal) {
    lowerLimit = minVal;
    upperLimit = maxVal;
  }
  
  int32_t getRotation() const {
    return rotation.load();
  }
};

Knob knob3(0, 8);

// -------------------- Timer & Display --------------------
HardwareTimer sampleTimer(TIM1);
U8G2_SSD1305_128X32_ADAFRUIT_F_HW_I2C u8g2(U8G2_R0);

// -------------------- Helper Functions for Key Matrix --------------------
void setOutMuxBit(const uint8_t bitIdx, const bool value) {
  digitalWrite(REN_PIN, LOW);
  digitalWrite(RA0_PIN, bitIdx & 0x01);
  digitalWrite(RA1_PIN, bitIdx & 0x02);
  digitalWrite(RA2_PIN, bitIdx & 0x04);
  digitalWrite(OUT_PIN, value);
  digitalWrite(REN_PIN, HIGH);
  delayMicroseconds(2);
  digitalWrite(REN_PIN, LOW);
}

std::bitset<4> readCols() {
  std::bitset<4> result;
  result[0] = digitalRead(C0_PIN);
  result[1] = digitalRead(C1_PIN);
  result[2] = digitalRead(C2_PIN);
  result[3] = digitalRead(C3_PIN);
  return result;
}

void setRow(uint8_t rowIdx) {
  digitalWrite(REN_PIN, LOW);
  digitalWrite(RA0_PIN, rowIdx & 0x01);
  digitalWrite(RA1_PIN, rowIdx & 0x02);
  digitalWrite(RA2_PIN, rowIdx & 0x04);
  digitalWrite(REN_PIN, HIGH);
}

// -------------------- CAN ISRs --------------------
// CAN Rx ISR: quickly store incoming message into msgInQ
extern "C" void myCanRxISR(void) {
  uint8_t RX_Message_ISR[8];
  uint32_t ID;
  CAN_RX(ID, RX_Message_ISR);
  xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
}

// CAN Tx ISR: signals that a mailbox is free by giving the semaphore
extern "C" void myCanTxISR(void) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(CAN_TX_Semaphore, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

// -------------------- Receiver: Decode Task --------------------
#ifdef MODULE_MODE_RECEIVER
void decodeTask(void *pvParameters) {
  uint8_t localMsg[8];
  while (1) {
    if (xQueueReceive(msgInQ, localMsg, portMAX_DELAY) == pdTRUE) {
      if (localMsg[0] == 'P') { // Key pressed
        uint8_t octave = localMsg[1];
        uint8_t note   = localMsg[2];
        uint32_t step  = stepSizes[note];
        if (octave > 4) {
          step <<= (octave - 4);
        } else if (octave < 4) {
          step >>= (4 - octave);
        }
        __atomic_store_n(&currentStepSize, step, __ATOMIC_RELAXED);
      } else if (localMsg[0] == 'R') { // Key released
        __atomic_store_n(&currentStepSize, 0, __ATOMIC_RELAXED);
      }
      xSemaphoreTake(sysState.mutex, portMAX_DELAY);
      memcpy(sysState.RX_Message, localMsg, 8);
      xSemaphoreGive(sysState.mutex);
    }
  }
}
#endif

// -------------------- Transmit Task (Sender Mode) --------------------
#ifdef MODULE_MODE_SENDER
void CAN_TX_Task(void * pvParameters) {
  uint8_t msgOut[8];
  while (1) {
    xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
    xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
    CAN_TX(0x123, msgOut);
  }
}
#endif

// -------------------- scanKeysTask --------------------
void scanKeysTask(void * pvParameters) {
  const TickType_t xFrequency = 20 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  static bool prevKeyPressed[12] = {
    true, true, true, true, true, true, true, true, true, true, true, true
  };
  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    std::bitset<32> all_inputs;
    for (uint8_t row = 0; row < 4; row++) {
      setRow(row);
      delayMicroseconds(3);
      std::bitset<4> result = readCols();
      for (uint8_t col = 0; col < 4; col++) {
        int index = row * 4 + col;
        all_inputs[index] = result[col];
      }
    }
    // In sender mode, local key changes are enqueued for transmission.
    #ifdef MODULE_MODE_SENDER
    for (uint8_t i = 0; i < 12; i++) {
      bool currentPressed = !all_inputs[i];  // LOW means pressed
      if (currentPressed != prevKeyPressed[i]) {
        uint8_t TX_Message[8] = {0};
        TX_Message[0] = currentPressed ? 'P' : 'R';
        TX_Message[1] = MODULE_OCTAVE;
        TX_Message[2] = i;
        xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
        prevKeyPressed[i] = currentPressed;
      }
    }
    #endif
    // Optionally, both modes update local state:
    knob3.update((all_inputs[13] << 1) | all_inputs[12]);
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    sysState.inputs = all_inputs;
    xSemaphoreGive(sysState.mutex);
  }
}

// -------------------- displayUpdateTask --------------------
void displayUpdateTask(void * pvParameters) {
  const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    const char* pressedKey = "None";
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    for (int i = 0; i < 12; i++) {
      if (!sysState.inputs[i]) {
        pressedKey = noteNames[i];
      }
    }
    uint8_t localCopy[8];
    memcpy(localCopy, sysState.RX_Message, 8);
    xSemaphoreGive(sysState.mutex);
    Serial.print("Local pressed key: ");
    Serial.print(pressedKey);
    Serial.print(", Current stepSize: ");
    Serial.print(currentStepSize);
    Serial.print(", Volume: ");
    Serial.println(knob3.getRotation());
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(2, 10, "Local Key: ");
    u8g2.setCursor(70, 10);
    u8g2.print(pressedKey);
    u8g2.drawStr(2, 20, "Volume:");
    u8g2.setCursor(70, 20);
    u8g2.print(knob3.getRotation());
    u8g2.setCursor(2, 30);
    u8g2.print("RX Msg: ");
    u8g2.setCursor(50, 30);
    u8g2.print((char)localCopy[0]);
    u8g2.print(localCopy[1]);
    u8g2.print(localCopy[2]);
    u8g2.sendBuffer();
    digitalToggle(LED_BUILTIN);
  }
}

// -------------------- Audio Generation ISR --------------------
#ifdef MODULE_MODE_RECEIVER
void sampleISR() {
  static uint32_t phaseAcc = 0;
  uint32_t localCurrentStepSize;
  __atomic_load(&currentStepSize, &localCurrentStepSize, __ATOMIC_RELAXED);
  phaseAcc += localCurrentStepSize;
  int32_t Vout = (phaseAcc >> 24) - 128;
  Vout = Vout >> (8 - knob3.getRotation());
  analogWrite(OUTR_PIN, Vout + 128);
}
#else
// In sender mode, no audio is generated.
void sampleISR() {
  analogWrite(OUTR_PIN, 128); // idle output
}
#endif

// -------------------- setup() --------------------
void setup() {
  pinMode(RA0_PIN, OUTPUT);
  pinMode(RA1_PIN, OUTPUT);
  pinMode(RA2_PIN, OUTPUT);
  pinMode(REN_PIN, OUTPUT);
  pinMode(OUT_PIN, OUTPUT);
  pinMode(OUTL_PIN, OUTPUT);
  pinMode(OUTR_PIN, OUTPUT);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(C0_PIN, INPUT);
  pinMode(C1_PIN, INPUT);
  pinMode(C2_PIN, INPUT);
  pinMode(C3_PIN, INPUT);
  pinMode(JOYX_PIN, INPUT);
  pinMode(JOYY_PIN, INPUT);
  setOutMuxBit(DRST_BIT, LOW);
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);
  Serial.begin(9600);
  Serial.println("Hello World");
  sampleTimer.setOverflow(22000, HERTZ_FORMAT);
  sampleTimer.attachInterrupt(sampleISR);
  sampleTimer.resume();
  // Create tasks. In sender mode, create scanKeysTask and CAN_TX_Task.
  #ifdef MODULE_MODE_SENDER
  xTaskCreate(scanKeysTask,      "scanKeys",      128, NULL, 2, NULL);
  xTaskCreate(CAN_TX_Task,       "CAN_TX_Task",   128, NULL, 2, NULL);
  #endif
  // In receiver mode, create decodeTask.
  #ifdef MODULE_MODE_RECEIVER
  xTaskCreate(decodeTask,        "decodeTask",    128, NULL, 2, NULL);
  #endif
  // displayUpdateTask runs in both modes.
  xTaskCreate(displayUpdateTask, "displayUpdate", 256, NULL, 1, NULL);
  // Also, optionally, scanKeysTask could run in receiver mode to capture local keys, if desired.
  // For now, we assume sender mode handles key scanning.
  sysState.mutex = xSemaphoreCreateMutex();
  msgInQ  = xQueueCreate(36, 8);
  msgOutQ = xQueueCreate(36, 8);
  if (!msgInQ || !msgOutQ) {
    Serial.println("Error: Could not create queues!");
    while (1);
  }
  CAN_TX_Semaphore = xSemaphoreCreateCounting(3, 3);
  if (!CAN_TX_Semaphore) {
    Serial.println("Error: Could not create CAN_TX_Semaphore!");
    while (1);
  }
  // Initialize and start CAN in loopback mode
  CAN_Init(true);
  setCANFilter(0x123, 0x7ff);
  // Register our CAN Rx and Tx ISRs
  CAN_RegisterRX_ISR(myCanRxISR);
  CAN_RegisterTX_ISR(myCanTxISR);
  CAN_Start();
  vTaskStartScheduler();
}

void loop() {
  // FreeRTOS tasks handle everything.
}
