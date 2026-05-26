#include <Arduino.h>
#include <Servo.h>

// ================= RS485 / Motors =================
#define RS485_DIR PB0

static const uint8_t ADDR_H = 0x01;  // горизонталь (FA01)
static const uint8_t ADDR_V = 0x02;  // вертикаль  (FA02)

// ================= Limits =================
// 4x Hall A3144 (active-low: magnet -> LOW)
#define V_TOP    PB13
#define V_BOTTOM PB12
#define H_LEFT   PB14
#define H_RIGHT  PB15

static const bool LIMIT_ACTIVE_LOW = true;

// ===== IRQ mode for STM32F1 (maple core) =====
#if defined(EXTI_FALLING)
static const ExtIntTriggerMode IRQ_MODE = EXTI_FALLING;
#elif defined(EXTI_RISING_FALLING)
static const ExtIntTriggerMode IRQ_MODE = EXTI_RISING_FALLING; // fallback
#else
// крайний fallback (если core другой) — обычно 1/2, но лучше чтобы EXTI_* были доступны
static const ExtIntTriggerMode IRQ_MODE = (ExtIntTriggerMode)1;
#endif

volatile uint8_t limLatched = 0;
enum : uint8_t {
  LIM_VTOP   = 1 << 0,
  LIM_VBOT   = 1 << 1,
  LIM_HLEFT  = 1 << 2,
  LIM_HRIGHT = 1 << 3
};

static inline void latch(uint8_t bit) { limLatched |= bit; }

void isrVTop()   { latch(LIM_VTOP);   }
void isrVBot()   { latch(LIM_VBOT);   }
void isrHLeft()  { latch(LIM_HLEFT);  }
void isrHRight() { latch(LIM_HRIGHT); }

static inline bool consumeLatched(uint8_t bit) {
  noInterrupts();
  bool hit = (limLatched & bit) != 0;
  limLatched &= ~bit;
  interrupts();
  return hit;
}

// level + debounce + latched edge
static bool hallHitStableLatched(uint8_t pin, uint8_t bit) {
  // уровень (если сейчас стоим на магните)
  for (int i = 0; i < 3; i++) {
    int v = digitalRead(pin);
    bool hit = LIMIT_ACTIVE_LOW ? (v == LOW) : (v == HIGH);
    if (hit) return true;
    delay(2);
  }
  // если проскочили магнит “внутри шага”
  if (consumeLatched(bit)) return true;
  return false;
}

// ================= Servo + OUT pins =================
#define SERVO1_PIN PB8
#define SERVO2_PIN PB7
#define OUT1_PIN   PB6
#define OUT2_PIN   PB5

// ================= Debug UART2 =================
#define DBG_BAUD 115200

// LED (BluePill)
static const uint8_t LED = PC13;

// ================= Servo tuning =================
static uint16_t S1_US_MIN = 700,  S1_US_MAX = 2300, S1_US_CTR = 1500;
static uint16_t S2_US_MIN = 700,  S2_US_MAX = 2300, S2_US_CTR = 1500;
static uint16_t INIT_DELAY_MS = 500;

// ================= Motor tuning =================
static uint8_t  ACC_HOME = 0x01;
static uint8_t  ACC_RUN  = 0x01;

// Vertical
static uint16_t V_SPEED_FAST = 0x0050;
static uint16_t V_SPEED_SLOW = 0x0020;
static uint16_t V_SPEED_RUN  = 0x0030;
static uint32_t V_JOG_FAST   = 5;
static uint32_t V_JOG_SLOW   = 1;
static uint32_t V_BACKOFF    = 20;
static int32_t  V_CENTER_OFFSET = 30;   // + вниз, - вверх

// Horizontal
//static uint16_t H_SPEED_FAST = 0x0030;
//static uint16_t H_SPEED_SLOW = 0x0010;
//static uint16_t H_SPEED_RUN  = 0x0020;
static uint16_t H_SPEED_FAST = 0x0050;
static uint16_t H_SPEED_SLOW = 0x0020;
static uint16_t H_SPEED_RUN  = 0x0030;
static uint32_t H_JOG_FAST   = 10;
static uint32_t H_JOG_SLOW   = 10;
static uint32_t H_BACKOFF    = 20;
static int32_t  H_CENTER_OFFSET = 0;

// Safety
static const uint32_t MAX_RANGE_STEPS = 200000;
static const uint32_t MAX_HOMING_MS   = 30000;

// Directions
static const bool DIR_UP_REVERSE    = false;
static const bool DIR_DOWN_REVERSE  = true;
static const bool DIR_RIGHT_REVERSE = true;
static const bool DIR_LEFT_REVERSE  = false;

// ================= State =================
static bool homedV = false, homedH = false;
static uint32_t rangeV = 0, rangeH = 0;
static int32_t posV = 0, posH = 0;

static bool out1_state = false, out2_state = false;

static Servo s1, s2;
static uint16_t s1_us = 1500, s2_us = 1500;

// ================= Utils =================
static uint8_t csum8(const uint8_t* b, size_t n) {
  uint16_t s = 0;
  for (size_t i = 0; i < n; i++) s += b[i];
  return (uint8_t)(s & 0xFF);
}

static void rs485Send(const uint8_t* p, size_t n) {
  digitalWrite(LED, LOW);
  digitalWrite(RS485_DIR, HIGH);
  delayMicroseconds(10);

  Serial1.write(p, n);
  Serial1.flush();
  delayMicroseconds((uint32_t)(n * 10000000UL / 115200UL) + 800);

  digitalWrite(RS485_DIR, LOW);
  digitalWrite(LED, HIGH);
}

static void cmdF7(uint8_t addr) {
  uint8_t p[4] = {0xFA, addr, 0xF7, 0};
  p[3] = csum8(p, 3);
  rs485Send(p, sizeof(p));
}

// FD: FA addr FD [speed_hi|dir] speed_lo acc steps(4 bytes LE) checksum
static void cmdFD(uint8_t addr, bool reverse, uint16_t speed, uint8_t acc, uint32_t steps) {
  uint8_t p[11];
  p[0] = 0xFA; p[1] = addr; p[2] = 0xFD;

  uint8_t speed_hi = (uint8_t)((speed >> 8) & 0x7F);
  if (reverse) speed_hi |= 0x80;
  p[3] = speed_hi;
  p[4] = (uint8_t)(speed & 0xFF);

  p[5] = acc;

  p[6] = (uint8_t)(steps & 0xFF);
  p[7] = (uint8_t)((steps >> 8) & 0xFF);
  p[8] = (uint8_t)((steps >> 16) & 0xFF);
  p[9] = (uint8_t)((steps >> 24) & 0xFF);

  p[10] = csum8(p, 10);
  rs485Send(p, sizeof(p));
}

// ===== Optional poll (RX may be empty until you wire RO->PA10 etc.) =====
static void send8500(uint8_t addr) {
  uint8_t p[5] = {0xFA, addr, 0x85, 0x00, 0};
  p[4] = csum8(p, 4);
  rs485Send(p, sizeof(p));
}
static void drainSerial1() { while (Serial1.available()) (void)Serial1.read(); }
static void dumpSerial1Hex(uint32_t windowMs) {
  uint32_t t0 = millis();
  bool any = false;
  while (millis() - t0 < windowMs) {
    while (Serial1.available()) {
      uint8_t b = (uint8_t)Serial1.read();
      any = true;
      if (b < 16) Serial2.print('0');
      Serial2.print(b, HEX);
      Serial2.print(' ');
    }
  }
  if (!any) Serial2.print("(no data)");
  Serial2.println();
}
static void pollMotor(const char* tag, uint8_t addr) {
  Serial2.print(tag); Serial2.print(" poll FA");
  if (addr < 16) Serial2.print('0');
  Serial2.print(addr, HEX); Serial2.print(": ");
  drainSerial1();
  send8500(addr);
  delay(30);
  dumpSerial1Hex(300);
}
static void pollAll(const char* tag) { pollMotor(tag, ADDR_V); pollMotor(tag, ADDR_H); }

// ================= Homing primitives =================
static bool fastToLimit(uint8_t addr, bool (*hit)(), bool dirReverse,
                        uint16_t speedFast, uint32_t jogFast, uint8_t acc) {
  uint32_t t0 = millis();
  while (!hit()) {
    if (millis() - t0 > MAX_HOMING_MS) return false;
    cmdFD(addr, dirReverse, speedFast, acc, jogFast);
    delay(5);
  }
  cmdF7(addr); delay(120);
  return true;
}
static void backoff(uint8_t addr, bool oppositeDirReverse,
                    uint16_t speedSlow, uint32_t backoffSteps, uint8_t acc) {
  cmdFD(addr, oppositeDirReverse, speedSlow, acc, backoffSteps);
  delay(150);
  cmdF7(addr); delay(120);
}
static bool slowToLimit(uint8_t addr, bool (*hit)(), bool dirReverse,
                        uint16_t speedSlow, uint32_t jogSlow, uint8_t acc) {
  uint32_t t0 = millis();
  while (!hit()) {
    if (millis() - t0 > MAX_HOMING_MS) return false;
    cmdFD(addr, dirReverse, speedSlow, acc, jogSlow);
    delay(5);
  }
  cmdF7(addr); delay(150);
  return true;
}
static bool seekLimit2Stage(uint8_t addr, bool (*hit)(), bool dirToLimit, bool dirOpposite,
                            uint16_t speedFast, uint32_t jogFast,
                            uint16_t speedSlow, uint32_t jogSlow,
                            uint32_t backoffSteps, uint8_t acc) {
  if (!fastToLimit(addr, hit, dirToLimit, speedFast, jogFast, acc)) return false;
  backoff(addr, dirOpposite, speedSlow, backoffSteps, acc);
  return slowToLimit(addr, hit, dirToLimit, speedSlow, jogSlow, acc);
}
static bool countToLimitSlow(uint8_t addr, bool (*hit)(), bool dirReverse, uint32_t& countedOut,
                             uint16_t speedSlow, uint32_t jogSlow, uint8_t acc) {
  uint32_t t0 = millis();
  uint32_t counted = 0;
  while (!hit()) {
    if (millis() - t0 > MAX_HOMING_MS) return false;
    cmdFD(addr, dirReverse, speedSlow, acc, jogSlow);
    counted += jogSlow;
    if (counted > MAX_RANGE_STEPS) return false;
    delay(5);
  }
  cmdF7(addr); delay(150);
  countedOut = counted;
  return true;
}

// ================= Homing: Vertical =================
static bool hitVTop()    { return hallHitStableLatched(V_TOP,    LIM_VTOP); }
static bool hitVBottom() { return hallHitStableLatched(V_BOTTOM, LIM_VBOT); }

static void goToCenterFromVTop(uint32_t range) {
  int32_t half = (int32_t)(range / 2);
  int32_t stepsDown = half + V_CENTER_OFFSET;
  if (stepsDown < 0) stepsDown = 0;

  cmdFD(ADDR_V, DIR_DOWN_REVERSE, V_SPEED_SLOW, ACC_HOME, (uint32_t)stepsDown);
  delay(600);
  cmdF7(ADDR_V);

  posV = (int32_t)range - stepsDown;
  if (posV < 0) posV = 0;
}

static bool homeVertical() {
  Serial2.println("V: seek TOP 2-stage...");
  if (!seekLimit2Stage(ADDR_V, hitVTop, DIR_UP_REVERSE, DIR_DOWN_REVERSE,
                       V_SPEED_FAST, V_JOG_FAST, V_SPEED_SLOW, V_JOG_SLOW, V_BACKOFF, ACC_HOME)) return false;

  Serial2.println("V: count TOP->BOTTOM (SLOW)...");
  uint32_t r1 = 0;
  if (!countToLimitSlow(ADDR_V, hitVBottom, DIR_DOWN_REVERSE, r1, V_SPEED_SLOW, V_JOG_SLOW, ACC_HOME)) return false;

  Serial2.println("V: count BOTTOM->TOP (SLOW)...");
  uint32_t r2 = 0;
  if (!countToLimitSlow(ADDR_V, hitVTop, DIR_UP_REVERSE, r2, V_SPEED_SLOW, V_JOG_SLOW, ACC_HOME)) return false;

  rangeV = (r1 + r2) / 2;
  Serial2.print("V AVG="); Serial2.println(rangeV);

  Serial2.println("V: center...");
  goToCenterFromVTop(rangeV);

  homedV = true;
  return true;
}

// ================= Homing: Horizontal =================
static bool hitHLeft()   { return hallHitStableLatched(H_LEFT,   LIM_HLEFT); }
static bool hitHRight()  { return hallHitStableLatched(H_RIGHT,  LIM_HRIGHT); }

static void goToCenterFromHRight(uint32_t range) {
  int32_t half = (int32_t)(range / 2);
  int32_t stepsLeft = half + H_CENTER_OFFSET;
  if (stepsLeft < 0) stepsLeft = 0;

  cmdFD(ADDR_H, DIR_LEFT_REVERSE, H_SPEED_SLOW, ACC_HOME, (uint32_t)stepsLeft);
  delay(600);
  cmdF7(ADDR_H);

  posH = (int32_t)range - stepsLeft;
  if (posH < 0) posH = 0;
}

static bool homeHorizontal() {
  Serial2.println("H: seek RIGHT 2-stage...");
  if (!seekLimit2Stage(ADDR_H, hitHRight, DIR_RIGHT_REVERSE, DIR_LEFT_REVERSE,
                       H_SPEED_FAST, H_JOG_FAST, H_SPEED_SLOW, H_JOG_SLOW, H_BACKOFF, ACC_HOME)) return false;

  Serial2.println("H: count RIGHT->LEFT (SLOW)...");
  uint32_t r1 = 0;
  if (!countToLimitSlow(ADDR_H, hitHLeft, DIR_LEFT_REVERSE, r1, H_SPEED_SLOW, H_JOG_SLOW, ACC_HOME)) return false;

  Serial2.println("H: count LEFT->RIGHT (SLOW)...");
  uint32_t r2 = 0;
  if (!countToLimitSlow(ADDR_H, hitHRight, DIR_RIGHT_REVERSE, r2, H_SPEED_SLOW, H_JOG_SLOW, ACC_HOME)) return false;

  rangeH = (r1 + r2) / 2;
  Serial2.print("H AVG="); Serial2.println(rangeH);

  Serial2.println("H: center...");
  goToCenterFromHRight(rangeH);

  homedH = true;
  return true;
}

// ================= Safe moves =================
static void safeMoveH(int32_t delta) {
  if (!homedH) { Serial2.println("H: not homed"); return; }
  int32_t target = posH + delta;
  if (target < 0) target = 0;
  if (target > (int32_t)rangeH) target = (int32_t)rangeH;

  int32_t move = target - posH;
  if (move == 0) return;

  if (move > 0 && hitHRight()) { cmdF7(ADDR_H); return; }
  if (move < 0 && hitHLeft())  { cmdF7(ADDR_H); return; }

  bool dir = (move > 0) ? DIR_RIGHT_REVERSE : DIR_LEFT_REVERSE;
  cmdFD(ADDR_H, dir, H_SPEED_RUN, ACC_RUN, (uint32_t)abs(move));
  posH = target;
}

static void safeMoveV(int32_t delta) {
  if (!homedV) { Serial2.println("V: not homed"); return; }
  int32_t target = posV + delta;
  if (target < 0) target = 0;
  if (target > (int32_t)rangeV) target = (int32_t)rangeV;

  int32_t move = target - posV;
  if (move == 0) return;

  if (move > 0 && hitVTop())    { cmdF7(ADDR_V); return; }
  if (move < 0 && hitVBottom()) { cmdF7(ADDR_V); return; }

  bool dir = (move > 0) ? DIR_UP_REVERSE : DIR_DOWN_REVERSE;
  cmdFD(ADDR_V, dir, V_SPEED_RUN, ACC_RUN, (uint32_t)abs(move));
  posV = target;
}

// ================= OUT + Servo control =================
static void setOut1(bool on) { out1_state = on; digitalWrite(OUT1_PIN, on ? HIGH : LOW); }
static void setOut2(bool on) { out2_state = on; digitalWrite(OUT2_PIN, on ? HIGH : LOW); }

static uint16_t clampU16(uint16_t v, uint16_t lo, uint16_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static void setServo1US(uint16_t us) {
  us = clampU16(us, S1_US_MIN, S1_US_MAX);
  s1_us = us;
  s1.writeMicroseconds(us);
}
static void setServo2US(uint16_t us) {
  us = clampU16(us, S2_US_MIN, S2_US_MAX);
  s2_us = us;
  s2.writeMicroseconds(us);
}

static void servoInitSequence() {
  if (!s1.attached()) s1.attach(SERVO1_PIN, S1_US_MIN, S1_US_MAX);
  if (!s2.attached()) s2.attach(SERVO2_PIN, S2_US_MIN, S2_US_MAX);

  Serial2.println("SERVOINIT: S1 max->min->center, then S2 max->min->center");

  setServo1US(S1_US_MAX); delay(INIT_DELAY_MS);
  setServo1US(S1_US_MIN); delay(INIT_DELAY_MS);
  setServo1US(S1_US_CTR); delay(INIT_DELAY_MS);

  setServo2US(S2_US_MAX); delay(INIT_DELAY_MS);
  setServo2US(S2_US_MIN); delay(INIT_DELAY_MS);
  setServo2US(S2_US_CTR); delay(INIT_DELAY_MS);

  Serial2.println("SERVOINIT: done");
}

// ================= Serial2 command parser =================
static char lineBuf[128];
static uint8_t linePos = 0;

static long parseLong(const char* s) { return strtol(s, nullptr, 0); }

static void printHelp() {
  Serial2.println("CMD:");
  Serial2.println("  HELP | HOME | STATUS | STOP");
  Serial2.println("  H <steps> | V <steps> | SPEEDH 0x.. | SPEEDV 0x.. | ACC <0..255>");
  Serial2.println("  SERVOINIT | SCENTER | S1 <us> | S2 <us> | S1ADD <d> | S2ADD <d>");
  Serial2.println("  OUT1 <0|1> | OUT2 <0|1>");
  Serial2.println("  SET S1MIN/S1MAX/S1CTR/S2MIN/S2MAX/S2CTR <us> | SET INITDELAY <ms>");
}

static void cmdStatus() {
  Serial2.print("posH="); Serial2.print(posH); Serial2.print("/"); Serial2.print(rangeH);
  Serial2.print(" posV="); Serial2.print(posV); Serial2.print("/"); Serial2.print(rangeV);
  Serial2.print(" OUT1="); Serial2.print(out1_state ? 1 : 0);
  Serial2.print(" OUT2="); Serial2.print(out2_state ? 1 : 0);
  Serial2.print(" S1_us="); Serial2.print(s1_us);
  Serial2.print(" S2_us="); Serial2.println(s2_us);

  pollAll("STATUS");
}

static void cmdHome() {
  Serial2.println("PRE-POLL:");
  pollAll("PRE");

  homedV = homedH = false;
  rangeV = rangeH = 0;
  posV = posH = 0;

  (void)homeVertical();
  delay(200);
  (void)homeHorizontal();

  Serial2.println("POST-POLL:");
  pollAll("POST");

  Serial2.println("HOME done.");
}

static void handleLine(char* s) {
  while (*s == ' ' || *s == '\t') s++;
  if (*s == 0) return;

  char* cmd = strtok(s, " \t");
  if (!cmd) return;

  if (!strcasecmp(cmd, "HELP"))   { printHelp(); return; }
  if (!strcasecmp(cmd, "STATUS")) { cmdStatus(); return; }
  if (!strcasecmp(cmd, "STOP"))   { cmdF7(0x00); Serial2.println("STOP (broadcast)"); return; }
  if (!strcasecmp(cmd, "HOME"))   { cmdHome(); return; }

  if (!strcasecmp(cmd, "ACC")) {
    char* a = strtok(nullptr, " \t");
    if (!a) { Serial2.println("ACC <0..255>"); return; }
    long v = parseLong(a); if (v < 0) v = 0; if (v > 255) v = 255;
    ACC_RUN = (uint8_t)v;
    Serial2.print("ACC_RUN="); Serial2.println((int)ACC_RUN);
    return;
  }

  if (!strcasecmp(cmd, "SPEEDH")) {
    char* a = strtok(nullptr, " \t");
    if (!a) { Serial2.println("SPEEDH 0x...."); return; }
    long v = parseLong(a); if (v < 0) v = 0; if (v > 0x7FFF) v = 0x7FFF;
    H_SPEED_RUN = (uint16_t)v;
    Serial2.print("H_SPEED_RUN="); Serial2.println((int)H_SPEED_RUN);
    return;
  }

  if (!strcasecmp(cmd, "SPEEDV")) {
    char* a = strtok(nullptr, " \t");
    if (!a) { Serial2.println("SPEEDV 0x...."); return; }
    long v = parseLong(a); if (v < 0) v = 0; if (v > 0x7FFF) v = 0x7FFF;
    V_SPEED_RUN = (uint16_t)v;
    Serial2.print("V_SPEED_RUN="); Serial2.println((int)V_SPEED_RUN);
    return;
  }

  if (!strcasecmp(cmd, "H")) {
    char* a = strtok(nullptr, " \t");
    if (!a) { Serial2.println("H <steps>"); return; }
    safeMoveH((int32_t)parseLong(a));
    return;
  }

  if (!strcasecmp(cmd, "V")) {
    char* a = strtok(nullptr, " \t");
    if (!a) { Serial2.println("V <steps>"); return; }
    safeMoveV((int32_t)parseLong(a));
    return;
  }

  if (!strcasecmp(cmd, "SERVOINIT")) { servoInitSequence(); return; }
  if (!strcasecmp(cmd, "SCENTER"))   { setServo1US(S1_US_CTR); setServo2US(S2_US_CTR); cmdStatus(); return; }

  if (!strcasecmp(cmd, "S1"))    { char* a=strtok(nullptr," \t"); if(!a) return; setServo1US((uint16_t)parseLong(a)); return; }
  if (!strcasecmp(cmd, "S2"))    { char* a=strtok(nullptr," \t"); if(!a) return; setServo2US((uint16_t)parseLong(a)); return; }
  if (!strcasecmp(cmd, "S1ADD")) { char* a=strtok(nullptr," \t"); if(!a) return; setServo1US((uint16_t)((long)s1_us + parseLong(a))); return; }
  if (!strcasecmp(cmd, "S2ADD")) { char* a=strtok(nullptr," \t"); if(!a) return; setServo2US((uint16_t)((long)s2_us + parseLong(a))); return; }

  if (!strcasecmp(cmd, "OUT1")) { char* a=strtok(nullptr," \t"); if(!a) return; setOut1(parseLong(a)?true:false); return; }
  if (!strcasecmp(cmd, "OUT2")) { char* a=strtok(nullptr," \t"); if(!a) return; setOut2(parseLong(a)?true:false); return; }

  if (!strcasecmp(cmd, "SET")) {
    char* key = strtok(nullptr, " \t");
    char* val = strtok(nullptr, " \t");
    if (!key || !val) return;
    long v = parseLong(val);

    if (!strcasecmp(key, "S1MIN")) { S1_US_MIN = (uint16_t)v; Serial2.println("OK"); return; }
    if (!strcasecmp(key, "S1MAX")) { S1_US_MAX = (uint16_t)v; Serial2.println("OK"); return; }
    if (!strcasecmp(key, "S1CTR")) { S1_US_CTR = (uint16_t)v; Serial2.println("OK"); return; }

    if (!strcasecmp(key, "S2MIN")) { S2_US_MIN = (uint16_t)v; Serial2.println("OK"); return; }
    if (!strcasecmp(key, "S2MAX")) { S2_US_MAX = (uint16_t)v; Serial2.println("OK"); return; }
    if (!strcasecmp(key, "S2CTR")) { S2_US_CTR = (uint16_t)v; Serial2.println("OK"); return; }

    if (!strcasecmp(key, "INITDELAY")) { INIT_DELAY_MS = (uint16_t)v; Serial2.println("OK"); return; }

    Serial2.println("ERR: unknown SET key");
    return;
  }

  Serial2.println("Unknown cmd (HELP)");
}

static void serial2Pump() {
  while (Serial2.available()) {
    char c = (char)Serial2.read();
    if (c == '\r') continue;
    if (c == '\n') {
      lineBuf[linePos] = 0;
      handleLine(lineBuf);
      linePos = 0;
      continue;
    }
    if (linePos < sizeof(lineBuf) - 1) lineBuf[linePos++] = c;
  }
}

// ================= Arduino =================
void setup() {
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);

  pinMode(RS485_DIR, OUTPUT);
  digitalWrite(RS485_DIR, LOW);

  // OUT pins
  pinMode(OUT1_PIN, OUTPUT);
  pinMode(OUT2_PIN, OUTPUT);
  setOut1(false);
  setOut2(false);

  // limits (Hall A3144 needs pull-up)
  pinMode(V_TOP, INPUT_PULLUP);
  pinMode(V_BOTTOM, INPUT_PULLUP);
  pinMode(H_LEFT, INPUT_PULLUP);
  pinMode(H_RIGHT, INPUT_PULLUP);

  // IMPORTANT: maple-core attachInterrupt() takes PIN (PBxx), not digitalPinToInterrupt()
  attachInterrupt(V_TOP,    isrVTop,   IRQ_MODE);
  attachInterrupt(V_BOTTOM, isrVBot,   IRQ_MODE);
  attachInterrupt(H_LEFT,   isrHLeft,  IRQ_MODE);
  attachInterrupt(H_RIGHT,  isrHRight, IRQ_MODE);

  Serial1.begin(115200);
  Serial2.begin(DBG_BAUD);
  delay(200);
  Serial2.println("READY. Type HELP");

  // attach servos
  s1.attach(SERVO1_PIN, S1_US_MIN, S1_US_MAX);
  s2.attach(SERVO2_PIN, S2_US_MIN, S2_US_MAX);
  setServo1US(S1_US_CTR);
  setServo2US(S2_US_CTR);

  // motor homing on boot
  pollAll("PRE");
  (void)homeVertical();
  delay(200);
  (void)homeHorizontal();
  pollAll("POST");

  cmdStatus();
  Serial2.println("No auto-move after init. Commands via Serial2.");
}

void loop() {
  serial2Pump();
  delay(2);
}
