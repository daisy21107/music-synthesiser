// -------------------- Configuration --------------------
#define MODULE_MODE_SENDER       // Enable sending: local key changes will be transmitted
#define MODULE_MODE_RECEIVER     // Enable receiving: incoming messages will be decoded and displayed

// We will need to set 1 keyboard to be both Sender & Receiver.
// Other 2 keyboards will be only the Sender (so that only one keyboard plays the sound)

// Set the module's octave (0-8); this is used locally when sending.
#define MODULE_OCTAVE 4

// Set TEST_MODE as follows:
// 0 = normal operation (original main.cpp)
// 1 = test scanKeysIteration() execution time        : ~1468 µs for 32 iterations (~46 µs per iteration)
// 2 = test display update execution time             : ~1,666,056 µs for 32 iterations (~52 ms per iteration)
// 3 = test decodeTask() processing iteration         : ~129 µs for 32 iterations (~4 µs per iteration)
// 4 = test CAN_TX_Task() processing iteration        : ~ 74 µs for 32 iterations (~2.3 µs per iteration)
// 5 = test sampleISR() execution time                : ~ 290 µs for 32 iterations (~9 µs per iteration)
#ifndef TEST_MODE
  #define TEST_MODE 5
#endif

#if TEST_MODE != 0
  #define DISABLE_THREADS
#endif

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
QueueHandle_t msgInQ  = NULL;   // For incoming CAN messages
QueueHandle_t msgOutQ = NULL;   // For outgoing CAN messages
SemaphoreHandle_t CAN_TX_Semaphore = NULL;  // Counting semaphore for TX mailboxes

// We store the latest received message in sysState, protected by sysState.mutex
struct {
  std::bitset<32> inputs;
  SemaphoreHandle_t mutex;
  uint8_t RX_Message[8];  // Buffer for the latest incoming CAN frame
} sysState;

// Instead of a fixed octave noteNames array, separate pitch classes.
const char* pitchClasses[12] = { 
  "C", "C#", "D", "D#", "E", "F", 
  "F#", "G", "G#", "A", "A#", "B" 
};

// Step sizes are defined for octave 4 (e.g. A4 = 440 Hz)
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
  85899345, // A4
  91007186, // A#4
  96418755  // B4
};

// -------------------- Pin definitions --------------------
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

// -------------------- Global variable for audio generation step size --------------------
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

// -------------------- scanKeysIteration() --------------------
// When TEST_MODE==1, use the test branch to simulate worst-case for key scanning.
// Otherwise, run the normal key scanning routine.
void scanKeysIteration() {
#if TEST_MODE == 1
  // Test branch for scanKeysIteration: simulate worst-case for each key.
  for (uint8_t i = 0; i < 12; i++) {
    uint8_t TX_Message[8] = { 'P', MODULE_OCTAVE, i, 0, 0, 0, 0, 0 };
    // Update state directly (bypassing queue operations to avoid blocking)
    #ifdef MODULE_MODE_RECEIVER
      xSemaphoreTake(sysState.mutex, portMAX_DELAY);
      memcpy(sysState.RX_Message, TX_Message, 8);
      xSemaphoreGive(sysState.mutex);
      uint32_t step = stepSizes[i];
      if (MODULE_OCTAVE > 4)
        step <<= (MODULE_OCTAVE - 4);
      else if (MODULE_OCTAVE < 4)
        step >>= (4 - MODULE_OCTAVE);
      __atomic_store_n(&currentStepSize, step, __ATOMIC_RELAXED);
    #endif
  }
  // Simulate a fixed knob update.
  knob3.update(0b00);
#else
  // Normal scanKeysIteration() code:
  static bool prevKeyPressed[12] = { true, true, true, true, true, true, true, true, true, true, true, true };
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
  #ifdef MODULE_MODE_SENDER
    for (uint8_t i = 0; i < 12; i++) {
      bool currentPressed = !all_inputs[i];  // LOW means pressed
      if (currentPressed != prevKeyPressed[i]) {
        uint8_t TX_Message[8] = {0};
        TX_Message[0] = currentPressed ? 'P' : 'R';
        // Transmit the local octave value.
        TX_Message[1] = MODULE_OCTAVE;
        TX_Message[2] = i;
        xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);
        // If also a receiver, simulate self-reception:
        #ifdef MODULE_MODE_RECEIVER
          xSemaphoreTake(sysState.mutex, portMAX_DELAY);
          memcpy(sysState.RX_Message, TX_Message, 8);
          xSemaphoreGive(sysState.mutex);
          if (TX_Message[0] == 'P') {
            uint8_t note = TX_Message[2];
            uint32_t step = stepSizes[note];
            if (TX_Message[1] > 4)
              step <<= (TX_Message[1] - 4);
            else if (TX_Message[1] < 4)
              step >>= (4 - TX_Message[1]);
            __atomic_store_n(&currentStepSize, step, __ATOMIC_RELAXED);
          } else {
            __atomic_store_n(&currentStepSize, 0, __ATOMIC_RELAXED);
          }
        #endif
        prevKeyPressed[i] = currentPressed;
      }
    }
  #endif
  // Update knob rotation using bits 12 and 13.
  knob3.update((all_inputs[13] << 1) | all_inputs[12]);
  xSemaphoreTake(sysState.mutex, portMAX_DELAY);
  sysState.inputs = all_inputs;
  xSemaphoreGive(sysState.mutex);
#endif
}

// -------------------- displayUpdateTask() --------------------
// We define displayUpdateTask only if we are not testing display update (TEST_MODE==2).
#if TEST_MODE != 2
void displayUpdateTask(void * pvParameters) {
  const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  char keyLabel[16] = "None";
  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    
    // Use the transmitted octave from RX_Message to build the key label.
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    if (sysState.RX_Message[0] == 'P') { // If a key is pressed
      uint8_t octave = sysState.RX_Message[1];
      uint8_t note = sysState.RX_Message[2];
      sprintf(keyLabel, "%s%d", pitchClasses[note], octave);
    } else {
      strcpy(keyLabel, "None");
    }
    xSemaphoreGive(sysState.mutex);
    
    Serial.print("Key: ");
    Serial.print(keyLabel);
    Serial.print(", StepSize: ");
    Serial.print(currentStepSize);
    Serial.print(", Volume: ");
    Serial.println(knob3.getRotation());
    
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(2, 10, "Key: ");
    u8g2.setCursor(70, 10);
    u8g2.print(keyLabel);
    u8g2.drawStr(2, 20, "Volume:");
    u8g2.setCursor(70, 20);
    u8g2.print(knob3.getRotation());
    u8g2.drawStr(2, 30, "RX Msg: ");
    u8g2.setCursor(50, 30);
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    u8g2.print((char)sysState.RX_Message[0]);
    u8g2.print(sysState.RX_Message[1]);
    u8g2.print(sysState.RX_Message[2]);
    xSemaphoreGive(sysState.mutex);
    u8g2.sendBuffer();
    
    digitalToggle(LED_BUILTIN);
  }
}
#endif

// Test function for display update when TEST_MODE==2
#if TEST_MODE == 2
void testDisplayIteration() {
  char keyLabel[16] = "None";
  xSemaphoreTake(sysState.mutex, portMAX_DELAY);
  if (sysState.RX_Message[0] == 'P') {
    uint8_t octave = sysState.RX_Message[1];
    uint8_t note = sysState.RX_Message[2];
    sprintf(keyLabel, "%s%d", pitchClasses[note], octave);
  } else {
    strcpy(keyLabel, "None");
  }
  xSemaphoreGive(sysState.mutex);
  // For testing, just print the label and state once.
  Serial.print("Test Display - Key: ");
  Serial.print(keyLabel);
  Serial.print(", StepSize: ");
  Serial.print(currentStepSize);
  Serial.print(", Volume: ");
  Serial.println(knob3.getRotation());
}
#endif

// -------------------- Test functions for additional test modes --------------------

// Test function for decodeTask (TEST_MODE==3)
#if TEST_MODE == 3
void testDecodeIteration() {
  // Simulate receiving a test message.
  uint8_t testMsg[8] = { 'P', MODULE_OCTAVE, 5, 0, 0, 0, 0, 0 }; // e.g., key press for note index 5
  if (testMsg[0] == 'P') {
    uint8_t octave = testMsg[1];
    uint8_t note = testMsg[2];
    uint32_t step = stepSizes[note];
    if (octave > 4)
      step <<= (octave - 4);
    else if (octave < 4)
      step >>= (4 - MODULE_OCTAVE);
    __atomic_store_n(&currentStepSize, step, __ATOMIC_RELAXED);
  } else if (testMsg[0] == 'R') {
    __atomic_store_n(&currentStepSize, 0, __ATOMIC_RELAXED);
  }
  xSemaphoreTake(sysState.mutex, portMAX_DELAY);
  memcpy(sysState.RX_Message, testMsg, 8);
  xSemaphoreGive(sysState.mutex);
}
#endif

// Test function for CAN_TX_Task (TEST_MODE==4)
// Instead of calling the actual CAN_TX() which relies on hardware,
// we use a dummy function that simulates similar processing overhead.
#if TEST_MODE == 4
void dummyCANTX(uint32_t id, uint8_t* msg) {
  volatile uint32_t dummy = 0;
  for (volatile int j = 0; j < 10; j++) {
    dummy += j;
  }
}
void testCANTXIteration() {
  uint8_t testMsg[8] = { 'P', MODULE_OCTAVE, 2, 0, 0, 0, 0, 0 }; // e.g., key press for note index 2
  dummyCANTX(0x123, testMsg);
}
#endif

// For TEST_MODE==5, we will use sampleISR() directly in a loop.

// -------------------- CAN ISRs --------------------
#if TEST_MODE == 0
extern "C" void myCanRxISR(void) {
  uint8_t RX_Message_ISR[8];
  uint32_t ID;
  CAN_RX(ID, RX_Message_ISR);
  xQueueSendFromISR(msgInQ, RX_Message_ISR, NULL);
}

extern "C" void myCanTxISR(void) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xSemaphoreGiveFromISR(CAN_TX_Semaphore, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
#endif

// -------------------- decodeTask() --------------------
#ifdef MODULE_MODE_RECEIVER
#if TEST_MODE == 0
void decodeTask(void *pvParameters) {
  uint8_t localMsg[8];
  while (1) {
    if (xQueueReceive(msgInQ, localMsg, portMAX_DELAY) == pdTRUE) {
      if (localMsg[0] == 'P') {
        uint8_t octave = localMsg[1];
        uint8_t note = localMsg[2];
        uint32_t step = stepSizes[note];
        if (octave > 4) {
          step <<= (octave - 4);
        } else if (octave < 4) {
          step >>= (4 - octave);
        }
        __atomic_store_n(&currentStepSize, step, __ATOMIC_RELAXED);
      } else if (localMsg[0] == 'R') {
        __atomic_store_n(&currentStepSize, 0, __ATOMIC_RELAXED);
      }
      xSemaphoreTake(sysState.mutex, portMAX_DELAY);
      memcpy(sysState.RX_Message, localMsg, 8);
      xSemaphoreGive(sysState.mutex);
    }
  }
}
#endif
#endif

// -------------------- CAN_TX_Task() --------------------
#ifdef MODULE_MODE_SENDER
#if TEST_MODE == 0
void CAN_TX_Task(void * pvParameters) {
  uint8_t msgOut[8];
  while (1) {
    xQueueReceive(msgOutQ, msgOut, portMAX_DELAY);
    xSemaphoreTake(CAN_TX_Semaphore, portMAX_DELAY);
    CAN_TX(0x123, msgOut);
  }
}
#endif
#endif

// -------------------- sampleISR() --------------------
#ifdef MODULE_MODE_RECEIVER
#if TEST_MODE == 0
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
void sampleISR() {
  analogWrite(OUTR_PIN, 128);
}
#endif
#else
void sampleISR() {
  analogWrite(OUTR_PIN, 128);
}
#endif

// -------------------- Normal Task Functions --------------------
#if TEST_MODE == 0
void scanKeysTask(void * pvParameters) {
  const TickType_t xFrequency = 20 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    scanKeysIteration();
  }
}
#endif

// -------------------- setup() --------------------
void setup() {
  // Set up pins.
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
  
  // Begin Serial communication.
  Serial.begin(9600);
  Serial.println("Hello World");
  
  // Create the RTOS objects needed by the test functions.
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
  
  // Set up the sample timer.
  sampleTimer.setOverflow(22000, HERTZ_FORMAT);
  sampleTimer.attachInterrupt(sampleISR);
  sampleTimer.resume();
  
  // Test mode branches:
  #if TEST_MODE == 1
    Serial.println("TEST MODE 1: Timing scanKeysIteration()");
    uint32_t startTime = micros();
    for (int i = 0; i < 32; i++) {
      scanKeysIteration();
    }
    uint32_t elapsed = micros() - startTime;
    Serial.print("Total time for 32 iterations of scanKeysIteration(): ");
    Serial.println(elapsed);
    while(1);
  #elif TEST_MODE == 2
    Serial.println("TEST MODE 2: Timing display update iteration");
    uint32_t startTime = micros();
    for (int i = 0; i < 32; i++) {
      testDisplayIteration();
    }
    uint32_t elapsed = micros() - startTime;
    Serial.print("Total time for 32 iterations of display update: ");
    Serial.println(elapsed);
    while(1);
  #elif TEST_MODE == 3
    Serial.println("TEST MODE 3: Timing decodeTask() iteration");
    uint32_t startTime = micros();
    for (int i = 0; i < 32; i++) {
      testDecodeIteration();
    }
    uint32_t elapsed = micros() - startTime;
    Serial.print("Total time for 32 iterations of decodeTask processing: ");
    Serial.println(elapsed);
    while(1);
  #elif TEST_MODE == 4
    Serial.println("TEST MODE 4: Timing CAN_TX_Task() iteration");
    uint32_t startTime = micros();
    for (int i = 0; i < 32; i++) {
      testCANTXIteration();
    }
    uint32_t elapsed = micros() - startTime;
    Serial.print("Total time for 32 iterations of CAN_TX_Task processing: ");
    Serial.println(elapsed);
    while(1);
  #elif TEST_MODE == 5
    Serial.println("TEST MODE 5: Timing sampleISR() iteration");
    uint32_t startTime = micros();
    for (int i = 0; i < 32; i++) {
      sampleISR();
    }
    uint32_t elapsed = micros() - startTime;
    Serial.print("Total time for 32 iterations of sampleISR(): ");
    Serial.println(elapsed);
    while(1);
  #else
    // Normal operation: create tasks and start scheduler.
    TaskHandle_t scanKeysHandle = NULL;
    xTaskCreate(scanKeysTask, "scanKeys", 128, NULL, 2, &scanKeysHandle);
    #ifdef MODULE_MODE_SENDER
      TaskHandle_t CAN_TXHandle = NULL;
      xTaskCreate(CAN_TX_Task, "CAN_TX_Task", 128, NULL, 2, &CAN_TXHandle);
    #endif
    #ifdef MODULE_MODE_RECEIVER
      TaskHandle_t decodeHandle = NULL;
      xTaskCreate(decodeTask, "decodeTask", 128, NULL, 2, &decodeHandle);
    #endif
    TaskHandle_t displayUpdateHandle = NULL;
    xTaskCreate(displayUpdateTask, "displayUpdate", 256, NULL, 1, &displayUpdateHandle);
  #endif
  
  #if TEST_MODE == 0
    CAN_Init(false);
    setCANFilter(0x123, 0x7ff);
    CAN_RegisterRX_ISR(myCanRxISR);
    CAN_RegisterTX_ISR(myCanTxISR);
    CAN_Start();
    vTaskStartScheduler();
  #endif
}

void loop() {
  // In test modes, loop() will not run.
  // When TEST_MODE is 0, FreeRTOS tasks handle everything.
}
