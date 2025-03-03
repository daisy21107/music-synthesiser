#include <Arduino.h>
#include <U8g2lib.h>
#include <bitset>
#include <HardwareTimer.h>
#include <STM32FreeRTOS.h>

//Constants
  const char* noteNames[12] = {"C4", "C#4", "D4", "D#4", "E4", "F4", 
                               "F#4", "G4", "G#4", "A4", "A#4", "B4"};
  const uint32_t stepSizes[12] = { // Step sizes of 12 notes
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

  //Matrix input and output
  const int C0_PIN = A2;
  const int C1_PIN = D9;
  const int C2_PIN = A6;
  const int C3_PIN = D1;
  const int OUT_PIN = D11;

  //Audio analogue out
  const int OUTL_PIN = A4;
  const int OUTR_PIN = A3;

  //Joystick analogue in
  const int JOYY_PIN = A0;
  const int JOYX_PIN = A1;

  //Output multiplexer bits
  const int DEN_BIT = 3;
  const int DRST_BIT = 4;
  const int HKOW_BIT = 5;
  const int HKOE_BIT = 6;

// Global variable to store current step size
volatile uint32_t currentStepSize = 0;

// Global struct to store system state that is used in more than one thread
struct {
  std::bitset<32> inputs;
  SemaphoreHandle_t mutex;
  int32_t knob3Rotation; // Tracks the total rotation
} sysState;

// Timer object
HardwareTimer sampleTimer(TIM1);

//Display driver object
U8G2_SSD1305_128X32_ADAFRUIT_F_HW_I2C u8g2(U8G2_R0);

//Function to set outputs using key matrix
void setOutMuxBit(const uint8_t bitIdx, const bool value) {
  digitalWrite(REN_PIN,LOW);
  digitalWrite(RA0_PIN, bitIdx & 0x01);
  digitalWrite(RA1_PIN, bitIdx & 0x02);
  digitalWrite(RA2_PIN, bitIdx & 0x04);
  digitalWrite(OUT_PIN,value);
  digitalWrite(REN_PIN,HIGH);
  delayMicroseconds(2);
  digitalWrite(REN_PIN,LOW);
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
  // Disable row select
  digitalWrite(REN_PIN, LOW);

  // Set row select address
  digitalWrite(RA0_PIN, rowIdx & 0x01);
  digitalWrite(RA1_PIN, rowIdx & 0x02);
  digitalWrite(RA2_PIN, rowIdx & 0x04);

  // Enable row select
  digitalWrite(REN_PIN, HIGH);
}

void scanKeysTask(void * pvParameters) {
  const TickType_t xFrequency = 20/portTICK_PERIOD_MS;  // Initiation interval: 20ms
  TickType_t xLastWakeTime = xTaskGetTickCount();       // Time (tick count) of last initiation

  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    std::bitset<32> all_inputs;
    int lastKeyPressed = -1; // default: no key pressed
    for (uint8_t row = 0; row < 4; row++) {
      setRow(row);
      delayMicroseconds(3);
      std::bitset<4> result = readCols();
      for (uint8_t col = 0; col < 4; col++) {
        int index = row * 4 + col;
        all_inputs[index] = result[col];
        if (index < 12 && !result[col]) {   // Only first 12 are piano keys AND if a key is pressed
            lastKeyPressed = index;
        }
      }
    }

    // Process knob rotation (row 3, columns 0 and 1)
    uint8_t knob3CurrState = (all_inputs[13] << 1) | all_inputs[12]; // {B, A}
    static uint8_t knob3PrevState = 0b00;  // Static: retains value between function calls
    static int8_t localKnob3Rotation = 0;
    static int8_t lastDirection = 0;      // Last valid rotation direction

    // Check transition and update rotation variable
    if ((knob3PrevState == 0b00 && knob3CurrState == 0b01) ||
        (knob3PrevState == 0b01 && knob3CurrState == 0b11) ||
        (knob3PrevState == 0b11 && knob3CurrState == 0b10) ||
        (knob3PrevState == 0b10 && knob3CurrState == 0b00)) {
      localKnob3Rotation += 1; // Clockwise
      lastDirection = 1;
    } else if ((knob3PrevState == 0b00 && knob3CurrState == 0b10) ||
               (knob3PrevState == 0b10 && knob3CurrState == 0b11) ||
               (knob3PrevState == 0b11 && knob3CurrState == 0b01) ||
               (knob3PrevState == 0b01 && knob3CurrState == 0b00)) {
      localKnob3Rotation -= 1; // Counterclockwise
      lastDirection = -1;
    } else if ((knob3PrevState == 0b00 && knob3CurrState == 0b11) ||
               (knob3PrevState == 0b01 && knob3CurrState == 0b10) ||
               (knob3PrevState == 0b10 && knob3CurrState == 0b01) ||
               (knob3PrevState == 0b11 && knob3CurrState == 0b00)) {
      // Impossible transition detected
      localKnob3Rotation += lastDirection; // Assume same direction as last valid transition
    }
    knob3PrevState = knob3CurrState; 
    if (localKnob3Rotation > 8) localKnob3Rotation = 8;
    if (localKnob3Rotation < 0) localKnob3Rotation = 0;
  
    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    sysState.inputs = all_inputs;     // Update system state
    xSemaphoreGive(sysState.mutex);
    
    // Checking which key is pressed and get the step size
    uint32_t localCurrentStepSize = 0; // Temporary local variable for step size
    if (lastKeyPressed != -1) {
      localCurrentStepSize = stepSizes[lastKeyPressed];
    }
  
    // Atomic store to ensure thread safety
    __atomic_store_n(&currentStepSize, localCurrentStepSize, __ATOMIC_RELAXED);
    __atomic_store_n(&sysState.knob3Rotation, localKnob3Rotation, __ATOMIC_RELAXED);
  }
}

void displayUpdateTask(void * pvParameters) {
  const TickType_t xFrequency = 100/portTICK_PERIOD_MS;  // Initiation interval: 100ms
  TickType_t xLastWakeTime = xTaskGetTickCount();        // Time (tick count) of last initiation
  while (1) {
    vTaskDelayUntil(&xLastWakeTime, xFrequency);

    const char* pressedKey = "None";
    uint8_t knob3Rotation = 0;

    xSemaphoreTake(sysState.mutex, portMAX_DELAY);
    for (int i = 0; i < 12; i++) {    // Only first 12 bits are piano keys
      if (!sysState.inputs[i]) {      // If bit is LOW (means that key is pressed)
        pressedKey = noteNames[i];
      }
    }
    knob3Rotation = sysState.knob3Rotation;
    xSemaphoreGive(sysState.mutex);

    Serial.print("Pressed key: ");
    Serial.print(pressedKey);
    Serial.print(", Step size: ");
    Serial.print(currentStepSize);
    Serial.print(", Knob 3 Rotation: ");
    Serial.println(knob3Rotation);

    //Update display
    u8g2.clearBuffer();                   // clear the internal memory
    u8g2.setFont(u8g2_font_ncenB08_tr);   // choose a suitable font
    u8g2.drawStr(2,10,"Pressed key: ");   // write something to the internal memory
    u8g2.setCursor(75,10);
    u8g2.print(pressedKey);
    u8g2.drawStr(2, 20, "Rotation:");
    u8g2.setCursor(75, 20);
    u8g2.print(knob3Rotation);
    u8g2.sendBuffer();                    // transfer internal memory to the display

    //Toggle LED
    digitalToggle(LED_BUILTIN);
  }
}

void sampleISR() {
  static uint32_t phaseAcc = 0; // Phase accumulator, static (stores value between calls)
  uint32_t localCurrentStepSize;
  __atomic_load(&currentStepSize, &localCurrentStepSize, __ATOMIC_RELAXED);
  phaseAcc += localCurrentStepSize;  // Increment phase

  int32_t Vout = (phaseAcc >> 24) - 128;  // Convert to sawtooth waveform

  int32_t localKnob3Rotation;
  __atomic_load(&sysState.knob3Rotation, &localKnob3Rotation, __ATOMIC_RELAXED);
  Vout = Vout >> (8 - localKnob3Rotation);  // Volume control
  analogWrite(OUTR_PIN, Vout + 128);
}

void setup() {
  // put your setup code here, to run once:

  //Set pin directions
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

  //Initialise display
  setOutMuxBit(DRST_BIT, LOW);  //Assert display logic reset
  delayMicroseconds(2);
  setOutMuxBit(DRST_BIT, HIGH);  //Release display logic reset
  u8g2.begin();
  setOutMuxBit(DEN_BIT, HIGH);  //Enable display power supply

  //Initialise UART
  Serial.begin(9600);
  Serial.println("Hello World");

  // Timer and interrupt set up
  sampleTimer.setOverflow(22000, HERTZ_FORMAT);
  sampleTimer.attachInterrupt(sampleISR);
  sampleTimer.resume();

  // Create RTOS thread for key scanning
  TaskHandle_t scanKeysHandle = NULL;
  xTaskCreate(
  scanKeysTask,		    /* Function that implements the task */
  "scanKeys",		      /* Text name for the task */
  64,      		        /* Stack size in words, not bytes */
  NULL,			          /* Parameter passed into the task */
  2,			            /* Task priority */
  &scanKeysHandle );	/* Pointer to store the task handle */

  TaskHandle_t displayUpdateHandle = NULL;
  xTaskCreate(
  displayUpdateTask,		    /* Function that implements the task */
  "displayUpdate",		      /* Text name for the task */
  256,      		            /* Stack size in words, not bytes */
  NULL,			                /* Parameter passed into the task */
  1,			                  /* Task priority */
  &displayUpdateHandle );	  /* Pointer to store the task handle */
  
  // Create mutex handle
  sysState.mutex = xSemaphoreCreateMutex();

  // Start RTOS scheduler
  vTaskStartScheduler();
}

void loop() {
  // put your main code here, to run repeatedly:
  
}