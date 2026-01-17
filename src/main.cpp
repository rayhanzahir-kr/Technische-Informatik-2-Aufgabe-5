#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPU6050.h>

// ==================== OLED SETUP ====================
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// ==================== MPU6050 SETUP ====================
Adafruit_MPU6050 mpu;
float accelX = 0, accelY = 0, accelZ = 0;
float gyroX = 0, gyroY = 0, gyroZ = 0;

// ==================== SERVO SETUP (DUA SERVO) ====================
const int SERVO_PIN_27 = 27;  // Servo di pin 27 (potensiometer)
const int SERVO_PIN_23 = 23;  // Servo di pin 23 (tombol VP)
const int SERVO_PWM_CHANNEL_27 = 0;  // Channel 0 untuk servo 27
const int SERVO_PWM_CHANNEL_23 = 1;  // Channel 1 untuk servo 23
const int SERVO_PWM_FREQ = 50;
const int SERVO_PWM_RESOLUTION = 12;

// ==================== PIN DEFINITIONS ====================
const int POT_PIN = 14;
const int LDR_PIN = 13;
const int BUTTON_P3 = 35;
const int BUTTON_S2 = 34;
const int BUTTON_VP = 36;  // Tombol VP untuk servo 23

// Traffic Light LEDs
const int RED1_PIN = 19;
const int YELLOW1_PIN = 18;
const int GREEN1_PIN = 5;
const int RED2_PIN = 16;
const int YELLOW2_PIN = 4;
const int GREEN2_PIN = 2;

// Mode Indicator LEDs (4 LED)
const int MODE_LEDS[4] = {32, 33, 25, 26};

// ==================== CONSTANTS ====================
const int MODE_COUNT = 4;
const int DAY_NIGHT_THRESHOLD = 2500;  // DIUBAH dari 50 ke 1500
const unsigned long BLINK_INTERVAL = 500;
const unsigned long PEDESTRIAN_DURATION = 5000;
const unsigned long ALL_RED_DURATION = 2000;
const unsigned long YELLOW_DURATION = 3000;
const unsigned long DISPLAY_UPDATE_INTERVAL = 300;
const unsigned long MPU_READ_INTERVAL = 100;
const unsigned long BARRIER_CLOSE_DELAY = 4000;
const unsigned long LDR_UPDATE_INTERVAL = 1000;  // Tambah interval LDR

// ==================== GLOBAL VARIABLES ====================
int currentMode = 0;
bool isNightMode = false;
bool yellowState = false;
unsigned long lastBlinkTime = 0;
bool pedestrianRequest = false;
unsigned long pedestrianRequestTime = 0;
int greenPhaseDuration[4] = {5000, 7000, 10000, 15000};

// Servo variables
int currentPotValue = 0;
int currentAngle27 = 0;  // Sudut servo 27
int currentAngle23 = 0;  // Sudut servo 23
bool isBarrierOpen_23 = false;
bool isBarrierOpen_27 = false;
unsigned long barrierCloseTime = 0;

// Traffic State - URUTAN YANG BENAR
enum TrafficState {
  ALL_RED_TRANSITION,     // 0: Semua merah (start)
  YELLOW1_RED2,           // 1: Kuning 1, Merah 2
  GREEN1_RED2,            // 2: Hijau 1, Merah 2
  YELLOW1_RED2_2,         // 3: Kuning 1 lagi, Merah 2
  ALL_RED_TRANSITION_2,   // 4: Semua merah (transisi)
  RED1_YELLOW2,           // 5: Merah 1, Kuning 2
  RED1_GREEN2,            // 6: Merah 1, Hijau 2
  RED1_YELLOW2_2,         // 7: Merah 1, Kuning 2 lagi
  PEDESTRIAN_CROSSING,    // 8
  BARRIER_MODE,           // 9
  WAIT_AFTER_BARRIER      // 10
};

TrafficState currentState = ALL_RED_TRANSITION;  // Mulai dari semua merah
unsigned long stateStartTime = 0;

// Display update timing
unsigned long lastDisplayUpdate = 0;
unsigned long lastMPURead = 0;
unsigned long lastLDRCheck = 0;  // Tambah untuk LDR

// LDR variables
int ldrSmoothValue = 0;
const int LDR_SAMPLES = 5;
int ldrSamples[LDR_SAMPLES];
int ldrSampleIndex = 0;

// ==================== FUNCTION DECLARATIONS ====================
void setupOLED();
void setupMPU6050();
void readMPU6050();
void setServoAngle27(int angle);  // Servo 27
void setServoAngle23(int angle);  // Servo 23
void calibrateLDR();
void checkNightMode();
void readPotentiometer();
void checkButtons();
void emergencyStop();
void handlePedestrianRequest();
void updateModeLEDs();
void nightModeOperation();
void normalModeOperation();
void setLights(int red1, int yellow1, int green1, int red2, int yellow2, int green2);
void printState();
void updateDisplay();
void handleBarrierMode();
void handleWaitAfterBarrier();
void controlServo23WithButton();  // Kontrol servo 23 dengan tombol
void updateLDRValue();  // Fungsi baru untuk baca LDR dengan filter
void activateNightMode();    // Tambah deklarasi fungsi
void deactivateNightMode();  // Tambah deklarasi fungsi

/////
// ==================== SETUP ====================
void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("=== TRAFFIC LIGHT WITH MPU6050 & OLED ===");
  Serial.println("Dual Servo System: Pin 27 (Pot) & Pin 23 (Button)");
  Serial.print("LDR Threshold: ");
  Serial.println(DAY_NIGHT_THRESHOLD);
  
  // Setup I2C
  Wire.begin(21, 22);
  
  // Setup OLED
  setupOLED();
  
  // Setup MPU6050
  setupMPU6050();
  
  // Setup LED pins
  pinMode(RED1_PIN, OUTPUT);
  pinMode(YELLOW1_PIN, OUTPUT);
  pinMode(GREEN1_PIN, OUTPUT);
  pinMode(RED2_PIN, OUTPUT);
  pinMode(YELLOW2_PIN, OUTPUT);
  pinMode(GREEN2_PIN, OUTPUT);
  
  for (int i = 0; i < MODE_COUNT; i++) {
    pinMode(MODE_LEDS[i], OUTPUT);
    digitalWrite(MODE_LEDS[i], LOW);
  }
  
  // Setup buttons
  pinMode(BUTTON_P3, INPUT_PULLUP);
  pinMode(BUTTON_S2, INPUT_PULLUP);
  pinMode(BUTTON_VP, INPUT_PULLUP);
  
  // Setup SERVO 27 (dikontrol potensiometer)
  ledcSetup(SERVO_PWM_CHANNEL_27, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
  ledcAttachPin(SERVO_PIN_27, SERVO_PWM_CHANNEL_27);
  setServoAngle27(0);
  
  // Setup SERVO 23 (dikontrol tombol VP)
  ledcSetup(SERVO_PWM_CHANNEL_23, SERVO_PWM_FREQ, SERVO_PWM_RESOLUTION);
  ledcAttachPin(SERVO_PIN_23, SERVO_PWM_CHANNEL_23);
  setServoAngle23(0);
  
  // Inisialisasi array LDR samples
  for (int i = 0; i < LDR_SAMPLES; i++) {
    ldrSamples[i] = analogRead(LDR_PIN);
  }
  
  // Kalibrasi
  calibrateLDR();
  
  // Inisialisasi traffic light
  currentState = ALL_RED_TRANSITION;
  stateStartTime = millis();
  setLights(HIGH, LOW, LOW, HIGH, LOW, LOW);  // Semua merah
  digitalWrite(MODE_LEDS[0], HIGH);
  
  // Baca nilai awal potentiometer untuk servo 27
  currentPotValue = analogRead(POT_PIN);
  currentAngle27 = map(currentPotValue, 0, 4095, 0, 180);
  setServoAngle27(currentAngle27);
  
  // Display startup message
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Traffic System Ready");
  display.print("LDR Th: ");
  display.println(DAY_NIGHT_THRESHOLD);
  display.println("Starting ALL RED");
  display.display();
  
  delay(2000);
  
  Serial.println("System initialized");
  Serial.println("Starting from: ALL RED");
  printState();
}

void setupOLED() {
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("OLED not found");
  } else {
    Serial.println("OLED ready");
  }
}

void setupMPU6050() {
  if (!mpu.begin()) {
    Serial.println("MPU6050 not found");
    while (1) delay(10);
  }
  Serial.println("MPU6050 Found!");
  
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
}

// ==================== SERVO FUNCTIONS ====================
void setServoAngle27(int angle) {
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;
  
  int pulseWidth = map(angle, 0, 180, 500, 2500);
  int duty = (pulseWidth * 4095) / 20000;
  ledcWrite(SERVO_PWM_CHANNEL_27, duty);
  
  currentAngle27 = angle;
  
  // Update barrier state untuk servo 27
  bool newBarrierOpen = (angle > 120);
  if (newBarrierOpen != isBarrierOpen_27) {
    isBarrierOpen_27 = newBarrierOpen;
    Serial.print("Servo 27: ");
    Serial.print(isBarrierOpen_27 ? "OPEN" : "CLOSED");
    Serial.print(" (");
    Serial.print(angle);
    Serial.println("°)");
  }
}

void setServoAngle23(int angle) {
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;
  
  int pulseWidth = map(angle, 0, 180, 500, 2500);
  int duty = (pulseWidth * 4095) / 20000;
  ledcWrite(SERVO_PWM_CHANNEL_23, duty);
  
  currentAngle23 = angle;
  
  // Update barrier state untuk servo 23
  bool newBarrierOpen = (angle > 120);
  if (newBarrierOpen != isBarrierOpen_23) {
    isBarrierOpen_23 = newBarrierOpen;
    Serial.print("Servo 23: ");
    Serial.print(isBarrierOpen_23 ? "OPEN" : "CLOSED");
    Serial.print(" (");
    Serial.print(angle);
    Serial.println("°)");
    if (!isBarrierOpen_23) {
      barrierCloseTime = millis();
    }
  }
}

void controlServo23WithButton() {
  static bool lastVPState = HIGH;
  static bool servo23State = false;  // false = tutup (0°), true = buka (180°)
  static unsigned long lastPressTime = 0;
  
  unsigned long currentTime = millis();
  
  // Debouncing
  if (currentTime - lastPressTime < 300) return;
  
  bool vpState = digitalRead(BUTTON_VP);
  
  // Jika tombol ditekan (LOW)
  if (vpState == LOW && lastVPState == HIGH) {
    lastPressTime = currentTime;
    
    // Toggle servo 23 state
    servo23State = !servo23State;
    
    if (servo23State) {
      setServoAngle23(180);  // Buka servo 23 (180°)
      Serial.println("VP Button: Servo 23 OPENED (180°)");
    } else {
      setServoAngle23(0);   // Tutup servo 23 (0°)
      Serial.println("VP Button: Servo 23 CLOSED (0°)");
    }
  }
  
  lastVPState = vpState;
}

// ==================== MPU6050 READING ====================
void readMPU6050() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  accelX = a.acceleration.x;
  accelY = a.acceleration.y;
  accelZ = a.acceleration.z;
  
  gyroX = g.gyro.x;
  gyroY = g.gyro.y;
  gyroZ = g.gyro.z;
}

// ==================== LDR FUNCTIONS ====================
void updateLDRValue() {
  // Baca LDR dan simpan dalam array untuk smoothing
  ldrSamples[ldrSampleIndex] = analogRead(LDR_PIN);
  ldrSampleIndex = (ldrSampleIndex + 1) % LDR_SAMPLES;
  
  // Hitung rata-rata
  long total = 0;
  for (int i = 0; i < LDR_SAMPLES; i++) {
    total += ldrSamples[i];
  }
  ldrSmoothValue = total / LDR_SAMPLES;
}

void calibrateLDR() {
  Serial.println("=== LDR CALIBRATION ===");
  delay(1000);
  
  // Baca 20 sample untuk rata-rata
  long total = 0;
  for (int i = 0; i < 20; i++) {
    total += analogRead(LDR_PIN);
    delay(50);
  }
  int avgLDR = total / 20;
  
  Serial.print("LDR Average: ");
  Serial.println(avgLDR);
  Serial.print("Threshold: ");
  Serial.println(DAY_NIGHT_THRESHOLD);
  Serial.print("Status: ");
  Serial.println(avgLDR > DAY_NIGHT_THRESHOLD ? "NIGHT (gelap)" : "DAY (terang)");
  
  // Tampilkan di OLED
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("LDR Calibration");
  display.print("Avg: ");
  display.println(avgLDR);
  display.print("Thresh: ");
  display.println(DAY_NIGHT_THRESHOLD);
  display.print("Status: ");
  display.println(avgLDR > DAY_NIGHT_THRESHOLD ? "NIGHT" : "DAY");
  display.display();
  delay(3000);
}

void checkNightMode() {
  static unsigned long lastCheck = 0;
  static unsigned long lastDebug = 0;
  
  unsigned long currentTime = millis();
  
  // Update nilai LDR setiap 100ms
  if (currentTime - lastCheck >= 100) {
    updateLDRValue();
    lastCheck = currentTime;
  }
  
  // Debug info setiap 3 detik
  if (currentTime - lastDebug >= 3000) {
    Serial.println("=== LDR STATUS ===");
    Serial.print("LDR Value: ");
    Serial.print(ldrSmoothValue);
    Serial.print(" | Threshold: ");
    Serial.println(DAY_NIGHT_THRESHOLD);
    Serial.print("Interpretation: ");
    
    if (ldrSmoothValue < 2000) {
      Serial.println("SANGAT GELAP (malam)");
    } else if (ldrSmoothValue < 2500) {
      Serial.println("GELAP (sore/malam)");
    } else if (ldrSmoothValue < 3000) {
      Serial.println("TERANG (siang)");
    } else {
      Serial.println("SANGAT TERANG (siang terik)");
    }
    
    Serial.print("Night Mode: ");
    Serial.println(isNightMode ? "ACTIVE" : "INACTIVE");
    Serial.println("=================");
    lastDebug = currentTime;
  }
  
  // LOGIKA DIBALIK: 
  // Jika LDR < threshold = GELAP = NIGHT MODE
  // Jika LDR > threshold = TERANG = DAY MODE
  bool nightDetected = (ldrSmoothValue < DAY_NIGHT_THRESHOLD);
  
  // Hysteresis untuk mencegah flickering
  static bool lastDetection = nightDetected;
  static unsigned long lastChangeTime = 0;
  
  if (nightDetected != lastDetection) {
    if (currentTime - lastChangeTime > 2000) { // Tunggu 2 detik stabil
      if (nightDetected && !isNightMode) {
        // Aktifkan NIGHT MODE (gelap)
        activateNightMode();
      } 
      else if (!nightDetected && isNightMode) {
        // Aktifkan DAY MODE (terang)
        deactivateNightMode();
      }
      lastChangeTime = currentTime;
      lastDetection = nightDetected;
    }
  } else {
    lastChangeTime = currentTime;
  }
}

// Fungsi untuk mengaktifkan night mode
void activateNightMode() {
  isNightMode = true;
  pedestrianRequest = false;
  
  Serial.println("=== NIGHT MODE ACTIVATED ===");
  Serial.print("LDR Value: ");
  Serial.println(ldrSmoothValue);
  Serial.println("Reason: LDR < threshold (GELAP)");
  
  // Matikan semua LED mode indicator
  for (int i = 0; i < MODE_COUNT; i++) {
    digitalWrite(MODE_LEDS[i], LOW);
  }
  
  // Set semua traffic light ke kuning blink
  yellowState = true;
  setLights(LOW, yellowState, LOW, LOW, yellowState, LOW);
  lastBlinkTime = millis();
  
  // Tampilkan di OLED
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("NIGHT MODE");
  display.println("Yellow Blink");
  display.print("LDR: ");
  display.println(ldrSmoothValue);
  display.print("Status: GELAP");
  display.display();
  delay(2000);
}

// Fungsi untuk mengaktifkan day mode  
void deactivateNightMode() {
  isNightMode = false;
  
  Serial.println("=== DAY MODE ACTIVATED ===");
  Serial.print("LDR Value: ");
  Serial.println(ldrSmoothValue);
  Serial.println("Reason: LDR > threshold (TERANG)");
  
  // Kembali ke traffic normal
  currentState = ALL_RED_TRANSITION;
  stateStartTime = millis();
  setLights(HIGH, LOW, LOW, HIGH, LOW, LOW);
  updateModeLEDs();
  
  // Tampilkan di OLED
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("DAY MODE");
  display.println("Normal Operation");
  display.print("LDR: ");
  display.println(ldrSmoothValue);
  display.print("Status: TERANG");
  display.display();
  delay(2000);
  
  printState();
}


// ==================== MAIN LOOP ====================
void loop() {
  unsigned long currentTime = millis();
  
  // 1. Baca MPU6050
  if (currentTime - lastMPURead >= MPU_READ_INTERVAL) {
    readMPU6050();
    lastMPURead = currentTime;
  }

  // 2. Check night mode (dipanggil setiap loop)
  checkNightMode();

  // 3. Kontrol servo 23 dengan tombol VP
  controlServo23WithButton();
  
  // 4. Kontrol servo 27 dengan potensiometer
  static unsigned long lastPotRead = 0;
  if (currentTime - lastPotRead >= 50) {
    int potValue = analogRead(POT_PIN);
    if (abs(potValue - currentPotValue) > 10) {
      currentPotValue = potValue;
      int angle = map(potValue, 0, 4095, 0, 180);
      setServoAngle27(angle);
    }
    lastPotRead = currentTime;
  }
  
  // 5. Read potentiometer for mode
  readPotentiometer();
  
  // 6. Check buttons (jika bukan night mode)
  if (!isNightMode) {
    checkButtons();
  }
  
  // 7. Handle pedestrian request
  if (pedestrianRequest) {
    handlePedestrianRequest();
  }
  
  // 8. Update display
  if (currentTime - lastDisplayUpdate >= DISPLAY_UPDATE_INTERVAL) {
    updateDisplay();
    lastDisplayUpdate = currentTime;
  }
  
  // 9. Main traffic logic
  if (isNightMode) {
    nightModeOperation();
  } else {
    // Check barrier mode
    if (isBarrierOpen_27) {
      // Jika barrier terbuka, PASTIKAN di BARRIER_MODE
      if (currentState != BARRIER_MODE && currentState != WAIT_AFTER_BARRIER) {
        currentState = BARRIER_MODE;
        stateStartTime = millis();
        Serial.println("BARRIER OPEN - Switching to BARRIER MODE (All Red)");
        setLights(HIGH, LOW, LOW, HIGH, LOW, LOW);  // Langsung semua merah
      }
    } else {
      // Barrier tertutup, jalankan traffic normal
      switch(currentState) {
        case BARRIER_MODE:
          handleBarrierMode();
          break;
        case WAIT_AFTER_BARRIER:
          handleWaitAfterBarrier();
          break;
        case PEDESTRIAN_CROSSING:
          break;
        default:
          normalModeOperation();
          break;
      }
    }
  }
  
  delay(10);
}

// ==================== HELPER FUNCTIONS ====================
void setLights(int red1, int yellow1, int green1, int red2, int yellow2, int green2) {
  digitalWrite(RED1_PIN, red1);
  digitalWrite(YELLOW1_PIN, yellow1);
  digitalWrite(GREEN1_PIN, green1);
  digitalWrite(RED2_PIN, red2);
  digitalWrite(YELLOW2_PIN, yellow2);
  digitalWrite(GREEN2_PIN, green2);
}

void printState() {
  Serial.print("State: ");
  switch(currentState) {
    case ALL_RED_TRANSITION: Serial.println("ALL RED (Start)"); break;
    case YELLOW1_RED2: Serial.println("YELLOW 1 - RED 2"); break;
    case GREEN1_RED2: Serial.println("GREEN 1 - RED 2"); break;
    case YELLOW1_RED2_2: Serial.println("YELLOW 1 (2) - RED 2"); break;
    case ALL_RED_TRANSITION_2: Serial.println("ALL RED (Transition)"); break;
    case RED1_YELLOW2: Serial.println("RED 1 - YELLOW 2"); break;
    case RED1_GREEN2: Serial.println("RED 1 - GREEN 2"); break;
    case RED1_YELLOW2_2: Serial.println("RED 1 - YELLOW 2 (2)"); break;
    case PEDESTRIAN_CROSSING: Serial.println("PEDESTRIAN CROSSING"); break;
    case BARRIER_MODE: Serial.println("BARRIER MODE"); break;
    case WAIT_AFTER_BARRIER: Serial.println("WAIT AFTER BARRIER"); break;
  }
}

void readPotentiometer() {
  int potValue = analogRead(POT_PIN);
  int newMode = potValue / 1024;
  
  if (newMode >= MODE_COUNT) {
    newMode = MODE_COUNT - 1;
  }
  
  if (newMode != currentMode) {
    currentMode = newMode;
    updateModeLEDs();
    
    Serial.print("Changed to Mode ");
    Serial.print(currentMode + 1);
    Serial.print(" (");
    Serial.print(greenPhaseDuration[currentMode] / 1000);
    Serial.println("s green)");
  }
}

void checkButtons() {
  static bool lastS2State = HIGH;
  static bool lastP3State = HIGH;
  static unsigned long lastDebounceTime = 0;
  
  unsigned long now = millis();
  if (now - lastDebounceTime < 100) return;
  lastDebounceTime = now;
  
  bool s2State = digitalRead(BUTTON_S2);
  bool p3State = digitalRead(BUTTON_P3);
  
  // Button S2 - Pedestrian (INSTANT dengan lampu merah)
  if (s2State == LOW && lastS2State == HIGH) {
    Serial.println("Button S2: INSTANT Pedestrian Request");
    
    // Langsung set semua lampu merah
    setLights(HIGH, LOW, LOW, HIGH, LOW, LOW);
    
    if (!pedestrianRequest) {
      pedestrianRequest = true;
      pedestrianRequestTime = millis();
      
      // Langsung masuk ke state pedestrian crossing
      if (currentState != PEDESTRIAN_CROSSING) {
        currentState = PEDESTRIAN_CROSSING;
        stateStartTime = millis();
        Serial.println("=== PEDESTRIAN CROSSING ACTIVATED ===");
      }
    }
  }
  
  // Button P3 - Emergency (INSTANT)
  if (p3State == LOW && lastP3State == HIGH) {
    Serial.println("Button P3: INSTANT Emergency Stop");
    emergencyStop();
  }
  
  lastS2State = s2State;
  lastP3State = p3State;
}

void emergencyStop() {
  Serial.println("!!! INSTANT EMERGENCY - ALL RED NOW !!!");
  setLights(HIGH, LOW, LOW, HIGH, LOW, LOW);
  
  // Reset semua state
  pedestrianRequest = false;
  
  // Kembali ke state awal
  currentState = ALL_RED_TRANSITION;
  stateStartTime = millis();
  
  Serial.println("Emergency activated - Starting from ALL RED");
  printState();
}

void handlePedestrianRequest() {
  if (currentState != PEDESTRIAN_CROSSING) {
    currentState = PEDESTRIAN_CROSSING;
    Serial.println("=== PEDESTRIAN CROSSING STARTED ===");
    setLights(HIGH, LOW, LOW, HIGH, LOW, LOW);  // Langsung semua merah
    stateStartTime = millis();
  }
  
  // Cek waktu (5 detik)
  if (millis() - pedestrianRequestTime >= PEDESTRIAN_DURATION) {
    Serial.println("Pedestrian crossing finished");
    pedestrianRequest = false;
    currentState = ALL_RED_TRANSITION;
    stateStartTime = millis();
    printState();
  }
}

void updateModeLEDs() {
  for (int i = 0; i < MODE_COUNT; i++) {
    digitalWrite(MODE_LEDS[i], LOW);
  }
  digitalWrite(MODE_LEDS[currentMode], HIGH);
}

void nightModeOperation() {
  if (!isNightMode) {
    return;
  }
  
  unsigned long currentTime = millis();
  
  // Blink kuning setiap BLINK_INTERVAL
  if (currentTime - lastBlinkTime >= BLINK_INTERVAL) {
    yellowState = !yellowState;
    setLights(LOW, yellowState, LOW, LOW, yellowState, LOW);
    lastBlinkTime = currentTime;
  }
}

void normalModeOperation() {
  if (pedestrianRequest || isBarrierOpen_27) {
    return;
  }
  
  unsigned long currentTime = millis();
  unsigned long elapsed = currentTime - stateStartTime;
  
  // Debug state setiap 2 detik
  static unsigned long lastDebugTime = 0;
  if (currentTime - lastDebugTime > 2000) {
    Serial.print("Traffic State: ");
    printState();
    Serial.print("Elapsed: ");
    Serial.println(elapsed);
    lastDebugTime = currentTime;
  }
  
  switch (currentState) {
    case ALL_RED_TRANSITION:  // 0: SEMUA MERAH (Start)
      setLights(HIGH, LOW, LOW, HIGH, LOW, LOW);
      
      if (elapsed >= ALL_RED_DURATION) {  // 2 detik
        currentState = YELLOW1_RED2;  // Ke KUNING
        stateStartTime = currentTime;
        Serial.println("Switching to YELLOW 1");
        printState();
      }
      break;
      
    case YELLOW1_RED2:  // 1: KUNING 1, MERAH 2
      setLights(LOW, HIGH, LOW, HIGH, LOW, LOW);
      
      if (elapsed >= YELLOW_DURATION) {  // 3 detik
        currentState = GREEN1_RED2;  // Ke HIJAU
        stateStartTime = currentTime;
        Serial.println("Switching to GREEN 1");
        printState();
      }
      break;
      
    case GREEN1_RED2:  // 2: HIJAU 1, MERAH 2
      setLights(LOW, LOW, HIGH, HIGH, LOW, LOW);
      
      if (elapsed >= greenPhaseDuration[currentMode]) {  // 5-15 detik
        currentState = YELLOW1_RED2_2;  // Ke KUNING lagi
        stateStartTime = currentTime;
        Serial.println("Green ended, switching to YELLOW");
        printState();
      }
      break;
      
    case YELLOW1_RED2_2:  // 3: KUNING 1 lagi, MERAH 2
      setLights(LOW, HIGH, LOW, HIGH, LOW, LOW);
      
      if (elapsed >= YELLOW_DURATION) {  // 3 detik
        currentState = ALL_RED_TRANSITION_2;  // Ke SEMUA MERAH
        stateStartTime = currentTime;
        Serial.println("Yellow ended, ALL RED");
        printState();
      }
      break;
      
    case ALL_RED_TRANSITION_2:  // 4: SEMUA MERAH (transisi)
      setLights(HIGH, LOW, LOW, HIGH, LOW, LOW);
      
      if (elapsed >= ALL_RED_DURATION) {  // 2 detik
        currentState = RED1_YELLOW2;  // Ke KUNING 2
        stateStartTime = currentTime;
        Serial.println("Switching to YELLOW 2");
        printState();
      }
      break;
      
    case RED1_YELLOW2:  // 5: MERAH 1, KUNING 2
      setLights(HIGH, LOW, LOW, LOW, HIGH, LOW);
      
      if (elapsed >= YELLOW_DURATION) {  // 3 detik
        currentState = RED1_GREEN2;  // Ke HIJAU 2
        stateStartTime = currentTime;
        Serial.println("Switching to GREEN 2");
        printState();
      }
      break;
      
    case RED1_GREEN2:  // 6: MERAH 1, HIJAU 2
      setLights(HIGH, LOW, LOW, LOW, LOW, HIGH);
      
      if (elapsed >= greenPhaseDuration[currentMode]) {  // 5-15 detik
        currentState = RED1_YELLOW2_2;  // Ke KUNING 2 lagi
        stateStartTime = currentTime;
        Serial.println("Green 2 ended, switching to YELLOW");
        printState();
      }
      break;
      
    case RED1_YELLOW2_2:  // 7: MERAH 1, KUNING 2 lagi
      setLights(HIGH, LOW, LOW, LOW, HIGH, LOW);
      
      if (elapsed >= YELLOW_DURATION) {  // 3 detik
        currentState = ALL_RED_TRANSITION;  // Kembali ke awal
        stateStartTime = currentTime;
        Serial.println("Cycle complete, back to ALL RED");
        printState();
      }
      break;
      
    default:
      currentState = ALL_RED_TRANSITION;
      stateStartTime = currentTime;
      Serial.println("Unknown state, resetting to ALL RED");
      break;
  }
}

void handleBarrierMode() {
  // Selama barrier terbuka: SEMUA LAMPU MERAH
  setLights(HIGH, LOW, LOW, HIGH, LOW, LOW);
  
  // Cek jika barrier sudah ditutup
  if (!isBarrierOpen_27) {
    Serial.println("Barrier closed - switching to WAIT_AFTER_BARRIER");
    currentState = WAIT_AFTER_BARRIER;
    stateStartTime = millis();
    barrierCloseTime = millis();
  }
}

void handleWaitAfterBarrier() {
  // Tetap merah selama 4 detik setelah barrier tutup
  setLights(HIGH, LOW, LOW, HIGH, LOW, LOW);
  
  // Tunggu 4 detik, lalu kembali ke siklus normal
  if (millis() - barrierCloseTime >= BARRIER_CLOSE_DELAY) {
    Serial.println("Wait finished - returning to normal traffic cycle");
    
    // Kembali ke awal siklus traffic
    currentState = ALL_RED_TRANSITION;
    stateStartTime = millis();
    barrierCloseTime = 0;
    
    printState();
  }
}

// ==================== DISPLAY FUNCTION ====================
void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  
  // Line 1: System Status
  display.setCursor(0, 0);
  display.print("S:");
  const char* states[] = {
    "AR",    // ALL_RED_TRANSITION
    "Y1R2",  // YELLOW1_RED2
    "G1R2",  // GREEN1_RED2
    "Y1R2",  // YELLOW1_RED2_2
    "AR2",   // ALL_RED_TRANSITION_2
    "R1Y2",  // RED1_YELLOW2
    "R1G2",  // RED1_GREEN2
    "R1Y2",  // RED1_YELLOW2_2
    "PED",   // PEDESTRIAN_CROSSING
    "BAR",   // BARRIER_MODE
    "WAIT"   // WAIT_AFTER_BARRIER
  };
  
  int stateIdx = currentState;
  if (stateIdx < 11) {
    display.print(states[stateIdx]);
  }
  
  display.print(" M:");
  display.print(currentMode + 1);
  display.print(" N:");
  display.print(isNightMode ? "Y" : "N");
  
  // Line 2: LDR Status
  display.setCursor(0, 10);
  display.print("LDR:");
  display.print(ldrSmoothValue);
  display.print("/");
  display.print(DAY_NIGHT_THRESHOLD);
  display.print(" ");
  display.print(isNightMode ? "NIGHT" : "DAY");
  
  // Line 3: Servo Status
  display.setCursor(0, 20);
  display.print("S27:");
  display.print(isBarrierOpen_27 ? "OPEN" : "CLOSED");
  display.print(" A:");
  display.print(currentAngle27);
  
  // Line 4: Servo 23 Status
  display.setCursor(0, 30);
  display.print("S23:");
  display.print(isBarrierOpen_23 ? "OPEN" : "CLOSED");
  
  // Line 5: MPU6050 & Buttons
  display.setCursor(0, 40);
  display.print("X:");
  display.print(accelX, 1);
  display.print(" S2:");
  display.print(digitalRead(BUTTON_S2) == LOW ? "P" : "_");
  display.print(" P3:");
  display.print(digitalRead(BUTTON_P3) == LOW ? "P" : "_");
  
  // Line 6: Pedestrian Status
  display.setCursor(0, 50);
  display.print("Ped:");
  if (pedestrianRequest) {
    display.print("CROSS");
    display.print((PEDESTRIAN_DURATION - (millis() - pedestrianRequestTime)) / 1000);
    display.print("s");
  } else {
    display.print("READY");
  }
  
  display.display();
}