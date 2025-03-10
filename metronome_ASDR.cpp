// -------------------- Configuration --------------------
#define MODULE_MODE_SENDER       // Enable sending: local key changes will be transmitted
#define MODULE_MODE_RECEIVER     // Enable receiving: incoming messages will be decoded and displayed

// We will need to set 1 keyboard to be both Sender&Receiver.
// Other 2 keyboards will be only the Sender (To make only one keyboard to play the sound)

// Set the module's octave (0-8); this is used locally when sending.
#define MODULE_OCTAVE 3

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
QueueHandle_t msgInQ  = NULL;  // For incoming CAN messages
QueueHandle_t msgOutQ = NULL;  // For outgoing CAN messages
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

// -------------------- Debug --------------------
volatile int32_t debugVout = 0; // Stores the latest waveform output

// -------------------- ASDR Variables --------------------
volatile float envelopeLevel = 0.0f;   // Current amplitude (0.0 to 1.0)
volatile float attackRate = 0.0001f;   // Speed of attack phase (increase when key pressed)
volatile float decayRate = 0.001f;      // Speed of decay (reduces to sustain level)
volatile float sustainLevel = 0.6f;    // Level where note holds when key is held (0.0 to 1.0)
volatile float releaseRate = 0.00005f; // Speed of release (fades out when key released)
volatile bool keyPressed = false;      // True when key is held
volatile bool keyReleased = false;     // True when key is released

// -------------------- Knob Class --------------------
class Knob {
private:
  std::atomic<int32_t> rotation;
  uint8_t prevState;
  int8_t lastDirection;
  int32_t lowerLimit, upperLimit;
  std::atomic<bool> isPressed;
public:
  Knob(int32_t minVal = 0, int32_t maxVal = 8)
    : rotation(0), prevState(0b00), lastDirection(0), lowerLimit(minVal), upperLimit(maxVal), isPressed(0) {}

  void update(uint8_t currState, bool pressed) {
    isPressed = pressed;
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

  bool getPressed() const {
    return isPressed.load();
  }
};

Knob knob3(0, 8);

// -------------------- Metronome Variables --------------------
volatile uint32_t metronomeInterval = 500; // Default 120 BPM (500ms per tick)
volatile bool metronomeEnabled = false;
volatile bool metronomeMode = false; // Tracks if metronome mode is active
Knob tempoKnob(40, 240); // Range from 40 BPM to 240 BPM

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
// Performs one iteration of key scanning and, in sender mode, enqueues key messages.
// Also simulates self-reception so that the module updates its own display and step size.
void scanKeysIteration() {
  static bool prevKeyPressed[12] = { false };
  std::bitset<32> all_inputs;
  
  for (uint8_t row = 0; row < 8; row++) {
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
      bool currentPressed = !all_inputs[i];  // LOW means key is pressed

      if (currentPressed != prevKeyPressed[i]) {
        uint8_t TX_Message[8] = {0};
        TX_Message[0] = currentPressed ? 'P' : 'R';
        TX_Message[1] = MODULE_OCTAVE;
        TX_Message[2] = i;
        xQueueSend(msgOutQ, TX_Message, portMAX_DELAY);

        #ifdef MODULE_MODE_RECEIVER
          xSemaphoreTake(sysState.mutex, portMAX_DELAY);
          memcpy(sysState.RX_Message, TX_Message, 8);
          xSemaphoreGive(sysState.mutex);

          if (TX_Message[0] == 'P') {  // Key Pressed → Start Attack Phase
            uint8_t note = TX_Message[2];
            uint32_t step = stepSizes[note];

            if (TX_Message[1] > 4)
              step <<= (TX_Message[1] - 4);
            else if (TX_Message[1] < 4)
              step >>= (4 - TX_Message[1]);

            __atomic_store_n(&currentStepSize, step, __ATOMIC_RELAXED);

            // Start Attack Phase
            keyPressed = true;
            keyReleased = false;
          } else {  // Key Released → Start Release Phase
            keyPressed = false;
            keyReleased = true;
          }
        #endif
        prevKeyPressed[i] = currentPressed;
      }
    }
  #endif

  knob3.update((all_inputs[13] << 1) | all_inputs[12], all_inputs[21]);
  tempoKnob.update((all_inputs[19] << 1) | all_inputs[18], all_inputs[24]);

  xSemaphoreTake(sysState.mutex, portMAX_DELAY);
  sysState.inputs = all_inputs;
  xSemaphoreGive(sysState.mutex);
}

// -------------------- scanKeysTask --------------------
void scanKeysTask(void * pvParameters) {
  const TickType_t xFrequency = 20 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    scanKeysIteration();
  }
}

// -------------------- Knob Button Press Handling --------------------
void checkKnobButton() {
  static bool lastButtonState = HIGH;
  bool currentButtonState = tempoKnob.getPressed(); // Assuming JOYY_PIN is the knob button

  if (lastButtonState == HIGH && currentButtonState == LOW) { // Detect press
      metronomeMode = !metronomeMode; // Toggle metronome mode
      metronomeEnabled = metronomeMode;
  }
  lastButtonState = currentButtonState;
}

// -------------------- displayUpdateTask --------------------
void displayUpdateTask(void * pvParameters) {
  const TickType_t xFrequency = 100 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  char keyLabel[16] = "None";
  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    checkKnobButton(); // Check if the knob button was pressed
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    if (metronomeMode) {
        uint32_t bpm = tempoKnob.getRotation();
        u8g2.drawStr(20, 16, "Metronome Mode");
        u8g2.drawStr(2, 32, "Tempo: ");
        u8g2.setCursor(50, 32);
        u8g2.print(bpm);
    } else {
        // Display other information when not in metronome mode
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
      Serial.print(knob3.getRotation());
      Serial.print(", Envelope Level: ");
      Serial.print(envelopeLevel);
      Serial.print(", Final Vout: ");
      Serial.print(debugVout+128);
      Serial.print(", Key Pressed: ");
      Serial.print(keyPressed);
      Serial.print(", Key Released: ");
      Serial.print(keyReleased);
      Serial.print(", Envelope Level: ");
      Serial.println(envelopeLevel);
      
      
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
    }
    u8g2.sendBuffer();
    digitalToggle(LED_BUILTIN);
  }
}

// -------------------- CAN ISRs --------------------
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

// -------------------- Receiver: decodeTask --------------------
#ifdef MODULE_MODE_RECEIVER
void decodeTask(void *pvParameters) {
  uint8_t localMsg[8];
  while (1) {
    if (xQueueReceive(msgInQ, localMsg, portMAX_DELAY) == pdTRUE) {
      if (localMsg[0] == 'P') {
        uint8_t octave = localMsg[1];  // Use the transmitted octave.
        uint8_t note = localMsg[2];
        uint32_t step = stepSizes[note];
        if (octave > 4) {
          step <<= (octave - 4);
        } else if (octave < 4) {
          step >>= (4 - octave);
        }
        // __atomic_store_n(&currentStepSize, step, __ATOMIC_RELAXED);
      } else if (localMsg[0] == 'R') {
        // __atomic_store_n(&currentStepSize, 0, __ATOMIC_RELAXED);
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

// -------------------- Audio Generation ISR --------------------
#ifdef MODULE_MODE_RECEIVER
void sampleISR() {
  static uint32_t phaseAcc = 0;
  uint32_t localCurrentStepSize;
  __atomic_load(&currentStepSize, &localCurrentStepSize, __ATOMIC_RELAXED);

  // ADSR Envelope Control
  if (keyPressed) {
    // Attack Phase: Increase to 1.0
    if (envelopeLevel < 1.0f) {
      envelopeLevel += attackRate;
      if (envelopeLevel > 1.0f) envelopeLevel = 1.0f;
    }
    // Decay Phase: Reduce to sustain level
    else if (envelopeLevel > sustainLevel) {
      envelopeLevel -= decayRate;
      if (envelopeLevel < sustainLevel) envelopeLevel = sustainLevel;
    }
  } 
  else if (keyReleased) {
    // Release Phase: Gradually fade out
    if (envelopeLevel > 0.0f) {
      envelopeLevel -= releaseRate;
      if (envelopeLevel < 0.0f) {
        envelopeLevel = 0.0f;
        keyReleased = false;
      }
    }
  }
  else {
    // **Step Size is ONLY reset when the envelope has fully faded out**
    __atomic_store_n(&currentStepSize, 0, __ATOMIC_RELAXED);
  }

  // **Ensure Step Size is NOT Reset Until Envelope Hits Zero**
  if (envelopeLevel > 0.0f) {
    phaseAcc += localCurrentStepSize; // Keep processing step size in release phase
  }

  // Generate raw waveform (square wave)
  int32_t rawWave = (phaseAcc >> 24) - 128;

  // Apply ADSR envelope
  int32_t Vout = static_cast<int32_t>(rawWave * envelopeLevel);

  // Apply volume control (right shift after envelope)
  Vout = Vout >> (8 - knob3.getRotation());

  // Store for debugging
  debugVout = Vout;

  // Ensure waveform is centered
  analogWrite(OUTR_PIN, Vout + 128);
}
#else
void sampleISR() {
  analogWrite(OUTR_PIN, 128); // idle output for sender mode
}
#endif

// -------------------- Metronome ISR --------------------
void metronomeISR() {
  static bool tickState = false;
  if (metronomeEnabled) {
      analogWrite(OUTR_PIN, tickState ? 255 : 0); // Toggle speaker output
      tickState = !tickState;
  }
}

// -------------------- Metronome Timer Setup --------------------
HardwareTimer metronomeTimer(TIM2);

// -------------------- Metronome Task --------------------
void metronomeTask(void *pvParameters) {
  const TickType_t xFrequency = 50 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();
  while (1) {
      vTaskDelayUntil(&xLastWakeTime, xFrequency);
      if (metronomeMode) {
          uint32_t bpm = tempoKnob.getRotation();
          metronomeInterval = 60000 / bpm; // Convert BPM to milliseconds per tick
          metronomeTimer.setOverflow(metronomeInterval * 1000, MICROSEC_FORMAT);
      }
  }
}

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
  // -----------------------------------------------------------------------------
  // pinMode(OUTR_PIN, OUTPUT); // Ensure speaker pin is set as output
    
  // Initialize Metronome Timer
  metronomeTimer.setOverflow(metronomeInterval * 1000, MICROSEC_FORMAT);
  metronomeTimer.attachInterrupt(metronomeISR);
  metronomeTimer.resume();
  
  // Create Metronome Task
  TaskHandle_t metronomeHandle = NULL;
  xTaskCreate(metronomeTask, "metronomeTask", 128, NULL, 1, &metronomeHandle);
  // ------------------------------------------------------------------------------
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
  // Create tasks for both sender and receiver functions.
  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(scanKeysTask,      "scanKeys",      128, NULL, 2, &scanKeysHandle);
  #ifdef MODULE_MODE_SENDER
  TaskHandle_t CAN_TXHandle = NULL;  
  xTaskCreate(CAN_TX_Task,       "CAN_TX_Task",   128, NULL, 2, &CAN_TXHandle);
  #endif
  #ifdef MODULE_MODE_RECEIVER
  TaskHandle_t decodeHandle = NULL;  
  xTaskCreate(decodeTask,        "decodeTask",    128, NULL, 2, &decodeHandle);
  #endif
  TaskHandle_t displayUpdateHandle = NULL;  
  xTaskCreate(displayUpdateTask, "displayUpdate", 256, NULL, 1, &displayUpdateHandle);
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
  CAN_Init(true);
  setCANFilter(0x123, 0x7ff);
  CAN_RegisterRX_ISR(myCanRxISR);
  CAN_RegisterTX_ISR(myCanTxISR);
  CAN_Start();
  vTaskStartScheduler();
}

void loop() {
  // FreeRTOS tasks handle everything.
}
