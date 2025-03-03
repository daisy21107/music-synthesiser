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

// Global outgoing message array (8 bytes)
volatile uint8_t TX_Message[8] = {0};

// Constants
const char* noteNames[12] = {"C4", "C#4", "D4", "D#4", "E4", "F4", 
                             "F#4", "G4", "G#4", "A4", "A#4", "B4"};
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

//Pin definitions
//Row select and enable
const int RA0_PIN = D3;
const int RA1_PIN = D6;
const int RA2_PIN = D12;
const int REN_PIN = A5;

// Matrix input and output
const int C0_PIN = A2;
const int C1_PIN = D9;
const int C2_PIN = A6;
const int C3_PIN = D1;
const int OUT_PIN = D11;

// Audio analogue out
const int OUTL_PIN = A4;
const int OUTR_PIN = A3;

// Joystick analogue in
const int JOYY_PIN = A0;
const int JOYX_PIN = A1;

// Output multiplexer bits
const int DEN_BIT = 3;
const int DRST_BIT = 4;
const int HKOW_BIT = 5;
const int HKOE_BIT = 6;

// Global variable to store current step size for audio generation
volatile uint32_t currentStepSize = 0;

// Knob class definition
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

// Global struct to store shared system state (for key matrix scanning)
struct {
  std::bitset<32> inputs;
  SemaphoreHandle_t mutex;
} sysState;

// Timer object
HardwareTimer sampleTimer(TIM1);

// Display driver object
U8G2_SSD1305_128X32_ADAFRUIT_F_HW_I2C u8g2(U8G2_R0);

// Function to set outputs using the key matrix
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

// scanKeysTask: Detect key state changes and update the global TX_Message, then send via CAN_TX()
void scanKeysTask(void * pvParameters) {
  const TickType_t xFrequency = 20 / portTICK_PERIOD_MS;
  TickType_t xLastWakeTime = xTaskGetTickCount();

  // Array to store previous state for 12 piano keys (true = not pressed)
  static bool prevKeyPressed[12] = { true, true, true, true, true, true, true, true, true, true, true, true };

  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
    std::bitset<32> all_inputs;
    int lastKeyPressed = -1;  // For audio tone generation

    // Scan the 4x4 key matrix
    for (uint8_t row = 0; row < 4; row++) {
      setRow(row);
      delayMicroseconds(3);
      std::bitset<4> result = readCols();
      for (uint8_t col = 0; col < 4; col++) {
        int index = row * 4 + col;
        all_inputs[index] = result[col];
        if (index < 12 && !result[col]) {  // For audio generation when key is pressed
          lastKeyPressed = index;
        }
      }
    }

    int MODULE_OCTAVE = 4;
    
    // For each piano key, detect state changes and update global TX_Message
    for (uint8_t i = 0; i < 12; i++) {
      bool currentPressed = !all_inputs[i];  // LOW indicates pressed
      if (currentPressed != prevKeyPressed[i]) {
        TX_Message[0] = currentPressed ? 'P' : 'R';  // 'P' for press, 'R' for release
        TX_Message[1] = MODULE_OCTAVE;               // Octave number
        TX_Message[2] = i;                           // Note number
        // The rest of TX_Message remains 0

        // Send the message over CAN using fixed ID 0x123
        CAN_TX(0x123, (uint8_t*)TX_Message);

        prevKeyPressed[i] = currentPressed;
      }
    }

    // Update knob rotation (using bits 12 and 13 from the matrix)
    knob3.update((all_inputs[13] << 1) | all_inputs[12]);

    // Update shared system state
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    sysState.inputs = all_inputs;
    xSemaphoreGive(sysState.mutex);

    // Determine step size for audio generation based on last key pressed
    uint32_t localCurrentStepSize = 0;
    if (lastKeyPressed != -1) {
      localCurrentStepSize = stepSizes[lastKeyPressed];
    }
    __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
  }
}

// displayUpdateTask: Display the latest TX_Message on the OLED
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
    xSemaphoreGive(sysState.mutex);

    Serial.print("Pressed key: ");
    Serial.print(pressedKey);
    Serial.print(", Step size: ");
    Serial.print(currentStepSize);
    Serial.print(", Volume: ");
    Serial.println(knob3.getRotation());

    // Update display with key info
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.drawStr(2, 10, "Pressed key: ");
    u8g2.setCursor(75, 10);
    u8g2.print(pressedKey);
    u8g2.drawStr(2, 20, "Volume:");
    u8g2.setCursor(75, 20);
    u8g2.print(knob3.getRotation());
    // Display the latest TX_Message
    u8g2.setCursor(66, 30);
    u8g2.print((char)TX_Message[0]);  // Cast first byte to char
    u8g2.print(TX_Message[1]);
    u8g2.print(TX_Message[2]);
    u8g2.sendBuffer();

    digitalToggle(LED_BUILTIN);
  }
}

void sampleISR() {
  static uint32_t phaseAcc = 0;
  uint32_t localCurrentStepSize;
  __atomic_load(&currentStepSize, &localCurrentStepSize, __ATOMIC_RELAXED);
  phaseAcc += localCurrentStepSize;
  int32_t Vout = (phaseAcc >> 24) - 128;
  Vout = Vout >> (8 - knob3.getRotation());
  analogWrite(OUTR_PIN, Vout + 128);
}

void setup() {
  // Set pin directions
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

  // Initialize display
  setOutMuxBit(DRST_BIT, LOW);
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);

  // Initialize UART
  Serial.begin(9600);
  Serial.println("Hello World");

  // Timer and interrupt setup
  sampleTimer.setOverflow(22000, HERTZ_FORMAT);
  sampleTimer.attachInterrupt(sampleISR);
  sampleTimer.resume();

  // Create RTOS tasks
  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(scanKeysTask, "scanKeys", 64, NULL, 2, &scanKeysHandle);
  TaskHandle_t displayUpdateHandle = NULL;
  xTaskCreate(displayUpdateTask, "displayUpdate", 256, NULL, 1, &displayUpdateHandle);
  
  // Create mutex for shared system state
  sysState.mutex = xSemaphoreCreateMutex();

  // Initialize CAN bus in loopback mode, set filter, and start CAN hardware
  CAN_Init(true);
  setCANFilter(0x123, 0x7ff);
  CAN_Start();

  // Start the RTOS scheduler
  vTaskStartScheduler();
}

void loop() {
  // Main loop remains empty; functionality is handled in tasks.
}
