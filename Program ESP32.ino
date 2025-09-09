#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "HX711.h"

/* ================== LCD 16x2 (TAMBAHAN) ================== */
#include <LiquidCrystal_I2C.h>
#define LCD_ADDR 0x27          // ganti ke 0x3F jika modulmu 0x3F
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);

// kontrol slide
static uint8_t  lcdSlide = 0;                 // 0..N-1
static unsigned long lcdLastMs = 0;
static const uint16_t LCD_INTERVAL_MS = 700;  // interval putar slide

// event khusus (prioritas tampil): IR2-5 “masuk wadah X” + “servo X kembali”
static unsigned long lcdEventUntilMs = 0;     // tampilkan event hingga waktu ini
static int  lcdEventServo = 0;                // 2..5 jika ada event wadah
static bool lcdEventUnknown = false;          // event “benda tak dikenal” (IR1)
static const uint16_t LCD_EVENT_MS = 1500;

// simpan edge IR untuk deteksi event tanpa ganggu flag utama
static uint8_t irPrev[5] = {0,0,0,0,0};

inline void lcdPrint2(const String& l1, const String& l2){
  lcd.clear();
  lcd.setCursor(0,0); lcd.print(l1);
  lcd.setCursor(0,1); lcd.print(l2);
}
/* ================== /LCD 16x2 ================== */

/* ================== Serial Debug ================== */
#define SERIAL_BAUD 115200
unsigned long lastDbg = 0;   // heartbeat print

/* ================== WiFi & MQTT ================== */
const char* ssid = "Xiaomi 14T";
const char* password = "11111111";
const char* mqtt_server = "10.31.203.73";

WiFiClient espClient;
PubSubClient client(espClient);

/* ================== Servo (PCA9685) ================== */
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();  // addr default 0x40
// Channel PWM: 0=servo1(dorong), 1=servo2, 2=servo3, 3=servo4, 4=servo5
#define SERVO_MIN 150
#define SERVO_MAX 600
inline uint16_t angleToPulse(int angle) { return map(angle, -90, 90, SERVO_MIN, SERVO_MAX); }
int servoAngles[5] = {0,0,0,0,0};

// === Setpoint sudut ===
#define SERVO1_PUSH_DEG   45
#define ROUTE_OPEN_DEG     -45 // arah dibalik; ubah ke 45 jika perlu
#define ROUTE_CLOSE_DEG    0

/* ================== IR Sensor ================== */
// IR index: 0..4  → IR1..IR5
// Pasangan: servo2↔IR2(index 1), servo3↔IR3(2), servo4↔IR4(3), servo5↔IR5(4)
const int sensorPins[5] = {14, 33, 25, 26, 27};
#define IR_ACTIVE_HIGH 0   // 0 = sensor aktif-LOW (umum), 1 = aktif-HIGH
int irValues[5] = {0,0,0,0,0};

// === Interrupt flags (cepat tanggap) ===
volatile uint32_t irFlags = 0;   // bit i = 1 berarti IR[i] baru terpicu
inline void setIrFlag(uint8_t idx){ irFlags |= (1UL << idx); }
inline void clrIrFlag(uint8_t idx){ irFlags &= ~(1UL << idx); }
inline bool getIrFlag(uint8_t idx){ return (irFlags & (1UL << idx)) != 0; }

// ISR khusus IR2..IR5 (index 1..4)
void IRAM_ATTR isrIR2() {
#if IR_ACTIVE_HIGH
  if (digitalRead(33) == HIGH) setIrFlag(1);
#else
  if (digitalRead(33) == LOW)  setIrFlag(1);
#endif
}
void IRAM_ATTR isrIR3() {
#if IR_ACTIVE_HIGH
  if (digitalRead(25) == HIGH) setIrFlag(2);
#else
  if (digitalRead(25) == LOW)  setIrFlag(2);
#endif
}
void IRAM_ATTR isrIR4() {
#if IR_ACTIVE_HIGH
  if (digitalRead(26) == HIGH) setIrFlag(3);
#else
  if (digitalRead(26) == LOW)  setIrFlag(3);
#endif
}
void IRAM_ATTR isrIR5() {
#if IR_ACTIVE_HIGH
  if (digitalRead(27) == HIGH) setIrFlag(4);
#else
  if (digitalRead(27) == LOW)  setIrFlag(4);
#endif
}

/* ================== Load Cell (HX711) ================== */
HX711 scale;
float calibration_factor = -1016;
float units = 0.0f;

/* ================== Kategori via MQTT ================== */
String kategoriTomat = "unknown"; // "baik" atau "reject"
unsigned long kategoriUpdatedMs = 0;

/* ================== Batas Berat + Histeresis ================== */
const float MIN_WEIGHT   = 5.0;    // minimal dianggap ada tomat
const float MED_MIN      = 100.0;  // 100–<150
const float BIG_MIN      = 150.0;  // >=150
const float HYS          = 2.0;    // histeresis ±2 g

enum WeightBand { WB_NONE=0, WB_KECIL=1, WB_SEDANG=2, WB_BESAR=3 };
WeightBand lastBand = WB_NONE;

// ---- PROTOTYPE utk hindari konflik auto-prototype Arduino ----
WeightBand classifyWeightHys(float w);
const char* weightBandToStr(WeightBand b);

/* ================== MOTOR DC (L298/L293) ================== */
#define PWM_ON_ENA 0          // 1 = PWM di ENA; 0 = PWM di IN1/IN2 (ENA=HIGH)
#define STOP_MODE_BRAKE 0     // 0 = COAST, 1 = BRAKE

const int ENA_PIN = 23;  // ENA
const int IN1_PIN = 18;  // arah
const int IN2_PIN = 19;  // arah

const int MOTOR_PWM_FREQ = 10000;  // 10 kHz
const int MOTOR_PWM_RES  = 8;      // 8-bit
const uint8_t MOTOR_DUTY_DEFAULT = 140;

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  // no channel needed
#else
  const int PWM_CH_ENA = 0;
  const int PWM_CH_IN1 = 1;
  const int PWM_CH_IN2 = 2;
#endif

void motorSetup();
void motorForward(uint8_t duty);
void motorReverse(uint8_t duty);
void motorStop();
void motorSoftStart(uint8_t target, uint8_t step=6, uint16_t step_ms=4);
void motorKickTo(uint8_t target, uint16_t kick_ms=150);

/* ===== fungsi klasifikasi dengan histeresis ===== */
WeightBand classifyWeightHys(float w) {
  if (w < MIN_WEIGHT) return WB_NONE;

  switch (lastBand) {
    case WB_KECIL:
      if (w >= MED_MIN + HYS) lastBand = WB_SEDANG;
      else lastBand = WB_KECIL;
      break;
    case WB_SEDANG:
      if (w >= BIG_MIN + HYS) lastBand = WB_BESAR;
      else if (w < MED_MIN - HYS) lastBand = WB_KECIL;
      else lastBand = WB_SEDANG;
      break;
    case WB_BESAR:
      if (w < BIG_MIN - HYS) lastBand = WB_SEDANG;
      else lastBand = WB_BESAR;
      break;
    case WB_NONE:
    default:
      if (w >= BIG_MIN) lastBand = WB_BESAR;
      else if (w >= MED_MIN) lastBand = WB_SEDANG;
      else lastBand = WB_KECIL;
      break;
  }
  return lastBand;
}

const char* weightBandToStr(WeightBand b) {
  switch (b) {
    case WB_KECIL:  return "kecil";
    case WB_SEDANG: return "sedang";
    case WB_BESAR:  return "besar";
    default:        return "-";
  }
}

/* ================== State Machine ================== */
enum RunState { IDLE, ARMED, PUSHING, OPEN_ROUTE, WAIT_IR, RESETTING };
RunState state = IDLE;

int targetServo = 0;                 // 0=none, 2/3/4/5 sesuai tujuan
unsigned long tStart = 0;

// === Servo1 1 detik ===
const unsigned long pushDelayMs   = 1000; // 1 detik dorong
const unsigned long routeTimeoutMs= 4000; // failsafe jika IR tak aktif

// Publish throttle
const uint32_t statusPeriodMs = 150;
uint32_t lastStatusMs = 0;

/* ================== Helpers ================== */
void setServoDeg(int ch, int deg) {
  if (ch < 0 || ch > 4) return;
  servoAngles[ch] = deg;
  pwm.setPWM(ch, 0, angleToPulse(deg));
}

void allServosTo(int deg) {
  for (int i=0;i<5;i++) setServoDeg(i, deg);
}

void publishStatus() {
  String data = "{";
  data += "\"Berat\":" + String(units, 2) + ",";
  data += "\"Kategori\":\"" + kategoriTomat + "\",";
  data += "\"KategoriBerat\":\"" + String(weightBandToStr(lastBand)) + "\",";
  data += "\"IR\":[" + String(irValues[0]) + "," + String(irValues[1]) + "," + String(irValues[2]) + "," + String(irValues[3]) + "," + String(irValues[4]) + "],";
  data += "\"Unknown\":" + String(irValues[0]) + ",";
  data += "\"Servo\":[" + String(servoAngles[0]) + "," + String(servoAngles[1]) + "," + String(servoAngles[2]) + "," + String(servoAngles[3]) + "," + String(servoAngles[4]) + "],";
  data += "\"State\":\"";
  switch (state) {
    case IDLE:       data += "IDLE"; break;
    case ARMED:      data += "ARMED"; break;
    case PUSHING:    data += "PUSHING"; break;
    case OPEN_ROUTE: data += "OPEN_ROUTE"; break;
    case WAIT_IR:    data += "WAIT_IR"; break;
    case RESETTING:  data += "RESETTING"; break;
  }
  data += "\",";
  data += "\"TargetServo\":" + String(targetServo);
  data += "}";
  client.publish("tomat/status", data.c_str());
}

/* ========== Motor helpers ========== */
static inline void writePWM_ENA(uint8_t d){
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(ENA_PIN, d);
#else
  ledcWrite(PWM_CH_ENA, d);
#endif
}
static inline void writePWM_IN1(uint8_t d){
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(IN1_PIN, d);
#else
  ledcWrite(PWM_CH_IN1, d);
#endif
}
static inline void writePWM_IN2(uint8_t d){
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  ledcWrite(IN2_PIN, d);
#else
  ledcWrite(PWM_CH_IN2, d);
#endif
}

void motorStop() {
#if STOP_MODE_BRAKE
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, HIGH);
#else
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, LOW);
#endif

#if PWM_ON_ENA
  writePWM_ENA(0);
#else
  writePWM_IN1(0);
  writePWM_IN2(0);
#endif
}

void motorSetup() {
  pinMode(IN1_PIN, OUTPUT);
  pinMode(IN2_PIN, OUTPUT);

#if ESP_ARDUINO_VERSION_MAJOR >= 3
  #if PWM_ON_ENA
    ledcAttach(ENA_PIN, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
  #else
    ledcAttach(IN1_PIN, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
    ledcAttach(IN2_PIN, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
    pinMode(ENA_PIN, OUTPUT);
    digitalWrite(ENA_PIN, HIGH);
  #endif
#else
  #if PWM_ON_ENA
    ledcSetup(PWM_CH_ENA, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
    ledcAttachPin(ENA_PIN, PWM_CH_ENA);
  #else
    ledcSetup(PWM_CH_IN1, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
    ledcSetup(PWM_CH_IN2, MOTOR_PWM_FREQ, MOTOR_PWM_RES);
    ledcAttachPin(IN1_PIN, PWM_CH_IN1);
    ledcAttachPin(IN2_PIN, PWM_CH_IN2);
    pinMode(ENA_PIN, OUTPUT);
    digitalWrite(ENA_PIN, HIGH);
  #endif
#endif

  motorStop();
}

void motorForward(uint8_t duty) {
#if PWM_ON_ENA
  digitalWrite(IN1_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  writePWM_ENA(duty);
#else
  digitalWrite(ENA_PIN, HIGH);
  digitalWrite(IN2_PIN, LOW);
  writePWM_IN1(duty);
#endif
}

void motorReverse(uint8_t duty) {
#if PWM_ON_ENA
  digitalWrite(IN1_PIN, LOW);
  digitalWrite(IN2_PIN, HIGH);
  writePWM_ENA(duty);
#else
  digitalWrite(ENA_PIN, HIGH);
  digitalWrite(IN1_PIN, LOW);
  writePWM_IN2(duty);
#endif
}

void motorSoftStart(uint8_t target, uint8_t step, uint16_t step_ms){
  for (uint16_t d=0; d<=(uint16_t)target; d+=step){
    motorForward((uint8_t)((d>255)?255:d));
    delay(step_ms);
  }
  motorForward(target);
}

void motorKickTo(uint8_t target, uint16_t kick_ms){
  motorForward(255);
  delay(kick_ms);
  motorForward(target);
}

/* ================== WiFi/MQTT ================== */
void setup_wifi() {
  WiFi.begin(ssid, password);
  Serial.print("WiFi connecting to "); Serial.println(ssid);
  while (WiFi.status() != WL_CONNECTED) {
    delay(250);
    Serial.print(".");
  }
  Serial.println("\nWiFi Connected");
  Serial.print("IP: "); Serial.println(WiFi.localIP());
  Serial.print("RSSI: "); Serial.println(WiFi.RSSI());
}

// (opsional) dukung perintah return dari Python
int extractServoNo(const String& msg) {
  for (size_t i = 0; i < msg.length(); ++i) {
    if (isDigit(msg[i])) return msg.substring(i).toInt();
  }
  return 0;
}
void doReturnServo(int servoNo) {
  if (servoNo < 2 || servoNo > 5) return;
  int ch = servoNo - 1;
  setServoDeg(ch, ROUTE_CLOSE_DEG);  // tutup hanya servo tujuan
  targetServo = 0;
  tStart = millis();
  state = RESETTING;
  Serial.printf("[RETURN] Servo %d -> 0°, state=RESETTING\n", servoNo);
}

void onMqtt(char* topic, byte* payload, unsigned int length) {
  String tpc(topic);
  String msg; msg.reserve(length);
  for (unsigned int i=0;i<length;i++) msg += (char)payload[i];
  msg.trim();

  if (tpc == "tomat/kategori") {
    msg.toLowerCase();
    Serial.print("[MQTT] tomat/kategori = "); Serial.println(msg);
    if (msg == "baik" || msg == "reject") kategoriTomat = msg;
    else kategoriTomat = "unknown";
    kategoriUpdatedMs = millis();
    return;
  }

  if (tpc == "tomat/servo/return") {   // opsional
    int servoNo = extractServoNo(msg);
    doReturnServo(servoNo);
    return;
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("MQTT connecting... ");
    if (client.connect("ESP32Client")) {
      Serial.println("OK");
      client.subscribe("tomat/kategori");
      client.subscribe("tomat/servo/return");   // opsional
      Serial.println("MQTT subscribed: tomat/kategori, tomat/servo/return");
    } else {
      Serial.print("fail, rc="); Serial.print(client.state());
      Serial.println(" retry in 1.2s");
      delay(1200);
    }
  }
}

/* ================== Setup/Loop ================== */
void setup() {
  Serial.begin(SERIAL_BAUD);
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(onMqtt);

  // LCD init (TAMBAHAN)
  lcd.init();
  lcd.backlight();
  lcdPrint2("Inisialisasi...", "MQTT & Sensor");

  // Konfigurasi pin IR
  for (int i=0;i<5;i++) {
#if IR_ACTIVE_HIGH
    pinMode(sensorPins[i], INPUT);       // aktif-HIGH → tanpa pullup
#else
    pinMode(sensorPins[i], INPUT_PULLUP);// aktif-LOW  → gunakan pullup internal
#endif
  }

  // Pasang interrupt khusus IR2..IR5 (index 1..4)
#if IR_ACTIVE_HIGH
  attachInterrupt(digitalPinToInterrupt(sensorPins[1]), isrIR2, RISING);
  attachInterrupt(digitalPinToInterrupt(sensorPins[2]), isrIR3, RISING);
  attachInterrupt(digitalPinToInterrupt(sensorPins[3]), isrIR4, RISING);
  attachInterrupt(digitalPinToInterrupt(sensorPins[4]), isrIR5, RISING);
#else
  attachInterrupt(digitalPinToInterrupt(sensorPins[1]), isrIR2, FALLING);
  attachInterrupt(digitalPinToInterrupt(sensorPins[2]), isrIR3, FALLING);
  attachInterrupt(digitalPinToInterrupt(sensorPins[3]), isrIR4, FALLING);
  attachInterrupt(digitalPinToInterrupt(sensorPins[4]), isrIR5, FALLING);
#endif

  // HX711
  scale.begin(34, 4);
  scale.set_scale(calibration_factor);
  scale.tare();
  scale.power_up();

  // Servo
  pwm.begin();
  pwm.setPWMFreq(50);
  delay(10);
  allServosTo(ROUTE_CLOSE_DEG);

  // Motor
  motorSetup();
  // motorSoftStart(MOTOR_DUTY_DEFAULT, 6, 4);
  motorKickTo(MOTOR_DUTY_DEFAULT, 180);

  lcdPrint2("System Ready", "Menunggu data...");
  Serial.println("System Ready (MQTT, IR-INT, HX711, Servo, Motor DC)");
}

/* ================== LCD updater (TAMBAHAN) ================== */
void lcdUpdate(){
  unsigned long now = millis();

  // --- Deteksi event baru berbasis edge ---
  // IR1: benda tak dikenal → tampil saat terdeteksi saja
  if (irValues[0]==1 && irPrev[0]==0){
    lcdEventUnknown = true;
    lcdEventServo = 0;
    lcdEventUntilMs = now + LCD_EVENT_MS;
  }
  // IR2..IR5: “masuk wadah X” + “servo X kembali”
  for (int i=1;i<=4;i++){
    if (irValues[i]==1 && irPrev[i]==0){
      lcdEventUnknown = false;
      lcdEventServo = i+1;              // 2..5
      lcdEventUntilMs = now + LCD_EVENT_MS;
      break; // satu event cukup
    }
  }

  // --- Prioritas: tampilkan event jika aktif ---
  if (now < lcdEventUntilMs){
    if (lcdEventUnknown){
      lcdPrint2("Benda tak", "dikenal!");
    }else if (lcdEventServo>=2 && lcdEventServo<=5){
      char l1[17]; snprintf(l1, sizeof(l1), "Masuk wadah %d", lcdEventServo);
      char l2[17]; snprintf(l2, sizeof(l2), "Servo %d kembali", lcdEventServo);
      lcdPrint2(String(l1), String(l2));
    }
    return;
  }

  // --- Kumpulan slide yang akan diputar ---
  // Slide 1: Berat + Kualitas (selalu ada)
  // Slide 2: “Servo mendorong” (muncul hanya bila Servo1 aktif)
  // Slide 3: “Servo X mendorong” (muncul hanya bila salah satu servo 2-5 sedang open)
  bool servo1Aktif = (servoAngles[0] == SERVO1_PUSH_DEG) || (state == PUSHING);
  int  openCh = -1;
  for (int ch=1; ch<=4; ch++){
    if (servoAngles[ch] == ROUTE_OPEN_DEG){ openCh = ch; break; } // ch=1..4 (servo2..5)
  }

  // daftar slide dinamis
  uint8_t slides[3];  // maksimal 3 jenis
  uint8_t nSlides = 0;
  slides[nSlides++] = 1;               // slide 1 selalu
  if (servo1Aktif) slides[nSlides++] = 2;
  if (openCh >= 1) slides[nSlides++] = 3;

  if (nSlides == 0) { // fallback (harusnya tak terjadi)
    slides[nSlides++] = 1;
  }

  // gilir slide
  if (now - lcdLastMs >= LCD_INTERVAL_MS){
    lcdLastMs = now;
    lcdSlide++;
    if (lcdSlide >= nSlides) lcdSlide = 0;
  }

  uint8_t cur = slides[lcdSlide];
  if (cur == 1){
    // Slide 1: Berat + Kualitas YOLO (kategoriTomat)
    String q = (kategoriTomat == "unknown") ? "-" : kategoriTomat;
    char l1[17]; snprintf(l1, sizeof(l1), "Berat: %.2f g", units);
    char l2[17]; snprintf(l2, sizeof(l2), "Kualitas: %s", q.c_str());
    lcdPrint2(String(l1), String(l2));
  } else if (cur == 2){
    // Slide 2: Servo 1 mendorong
    lcdPrint2("Servo 1", "mendorong");
  } else if (cur == 3){
    // Slide 3: Servo (2..5) mendorong (rute open)
    char l1[17]; snprintf(l1, sizeof(l1), "Servo %d", openCh+1);
    lcdPrint2(String(l1), "mendorong");
  }
}
/* ================== /LCD updater ================== */

void loop() {
  if (!client.connected()) reconnect();
  client.loop();

  // ==== IR read (untuk telemetry saja; keputusan cepat pakai irFlags) ====
  for (int i=0;i<5;i++) {
    int raw = digitalRead(sensorPins[i]);
#if IR_ACTIVE_HIGH
    irValues[i] = (raw == HIGH) ? 1 : 0;   // aktif-HIGH: HIGH = TRIPPED(1)
#else
    irValues[i] = (raw == LOW)  ? 1 : 0;   // aktif-LOW : LOW  = TRIPPED(1)
#endif
  }

  // ==== Load cell ====
  int hx_avg = (state==ARMED || state==PUSHING || state==WAIT_IR) ? 1 : 10;
  units = (float)scale.get_units(hx_avg);
  if (units < 0) units = 0.0f;

  // klasifikasi berat (histeresis)
  WeightBand band = classifyWeightHys(units);
  unsigned long now = millis();

  switch (state) {
    case IDLE: {
      bool enoughWeight = (band != WB_NONE);
      bool freshCategory = (now - kategoriUpdatedMs <= 1200) && (kategoriTomat == "baik" || kategoriTomat == "reject");
      if (enoughWeight && freshCategory) {
        if (kategoriTomat == "baik") {
          if      (band == WB_KECIL)  targetServo = 2;
          else if (band == WB_SEDANG) targetServo = 3;
          else                        targetServo = 4;
        } else {
          targetServo = 5;
        }
        // pastikan rute lain tertutup saat mulai
        for (int ch=1; ch<=4; ch++) setServoDeg(ch, ROUTE_CLOSE_DEG);
        setServoDeg(0, 0);
        // clear flag IR tujuan agar tidak baca event lama
        if (targetServo >= 2 && targetServo <= 5) clrIrFlag(targetServo-1);
        state = ARMED;
      }
      break;
    }

    case ARMED: {
      // Mulai PUSHING + sekaligus buka rute tujuan (bergerak bersamaan)
      setServoDeg(0, SERVO1_PUSH_DEG);  // pendorong ON
      if (targetServo >= 2 && targetServo <= 5) {
        int ch = targetServo - 1;       // 2->1,3->2,4->3,5->4
        setServoDeg(ch, ROUTE_OPEN_DEG);// ARAH DIBALIK (135°)
        clrIrFlag(ch);                  // buang event lama (safety)
      }
      tStart = now;
      state = PUSHING;
      break;
    }

    case PUSHING: {
      // Jika IR tujuan terpicu lebih cepat, tutup dan hentikan pendorong
      int irIndex = (targetServo >= 2 && targetServo <= 5) ? (targetServo - 1) : -1;
      if (irIndex >= 1 && irIndex <= 4 && getIrFlag(irIndex)) {
        setServoDeg(irIndex, ROUTE_CLOSE_DEG);
        clrIrFlag(irIndex);
        setServoDeg(0, 0);
        state = RESETTING;
        tStart = now;
        break;
      }
      if (now - tStart >= pushDelayMs) {
        setServoDeg(0, 0);              // pendorong OFF setelah 1 detik
        tStart = now;
        state = WAIT_IR;                // tunggu IR (rute sudah open)
      }
      break;
    }

    case OPEN_ROUTE: { // tidak dipakai
      state = WAIT_IR;
      break;
    }

    case WAIT_IR: {
      int irIndex = (targetServo >= 2 && targetServo <= 5) ? (targetServo - 1) : -1;
      bool tripped  = (irIndex >= 1 && irIndex <= 4) ? ( getIrFlag(irIndex) || (irValues[irIndex]==1) ) : false;
      bool timedOut = (now - tStart >= routeTimeoutMs);

      if (tripped) {
        int ch = targetServo - 1;
        setServoDeg(ch, ROUTE_CLOSE_DEG);   // tutup HANYA servo tujuan
        clrIrFlag(ch);
        state = RESETTING;
        tStart = now;
      }
      else if (timedOut) {
        setServoDeg(0, 0);
        tStart = now;
      }
      break;
    }

    case RESETTING: {
      if (band == WB_NONE || (now - tStart >= 500)) {
        targetServo = 0;
        kategoriTomat = "unknown";
        state = IDLE;
      }
      break;
    }
  }

  // ================== LCD UPDATE (TAMBAHAN) ==================
  // (slide 1,2,3 otomatis digilir; slide event IR2-5 = "masuk wadah X | servo X kembali";
  //  event IR1 = "benda tak dikenal")
  lcdUpdate();

  // Publish status (throttled)
  if (millis() - lastStatusMs >= statusPeriodMs) {
    lastStatusMs = millis();
    publishStatus();
  }

  // Heartbeat debug tiap 2 detik
  if (millis() - lastDbg >= 2000) {
    lastDbg = millis();
    Serial.printf("WiFi:%s RSSI:%d  MQTT:%s  Berat:%.2f  State:%d  Target:%d  IR:[%d,%d,%d,%d,%d] Flags:0x%02lx\n",
                  (WiFi.isConnected() ? "OK" : "DOWN"),
                  WiFi.RSSI(),
                  (client.connected() ? "OK" : "DOWN"),
                  units, (int)state, targetServo,
                  irValues[0], irValues[1], irValues[2], irValues[3], irValues[4],
                  (unsigned long)irFlags);
  }

  // simpan state IR untuk deteksi edge berikutnya
  for (int i=0;i<5;i++) irPrev[i] = irValues[i];
}
