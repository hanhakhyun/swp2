#include <Servo.h>
#include <math.h>

// ====== 핀 설정 ======
#define PIN_SERVO 10
#define PIN_TRIG  12
#define PIN_ECHO  13

// ====== 서보/동작 파라미터 ======
static const int   ANGLE_CLOSED = 0;     // 닫힘 각도
static const int   ANGLE_OPENED = 90;    // 열림 각도

// 비대칭 속도: 둘 다 부드럽지만 전체 속도 빠르게 (딜레이 줄임)
static const unsigned long OPEN_MS  = 900;   // 개방 시간(↑)
static const unsigned long CLOSE_MS = 1300;  // 폐쇄 시간(↓)

// ====== 거리/센서 파라미터 ======
static const float SND_VEL  = 346.0;
static const int   INTERVAL = 25;
static const int   PULSE_US = 10;
static const float EMA_ALPHA = 0.3;

static const int   OPEN_DIST_MM  = 220;  // 이하면 열기
static const int   CLOSE_DIST_MM = 260;  // 이면 닫기
static const unsigned long ECHO_TIMEOUT_US = (INTERVAL / 2) * 1000UL;

// ====== 전역 ======
Servo gate;
unsigned long lastSample = 0;
bool  emaInited = false;
float distEma = 9999.0;

enum GateState { IDLE, OPENING, CLOSING };
GateState state = IDLE;
unsigned long moveStart = 0;
int startAngle = ANGLE_CLOSED;
int targetAngle = ANGLE_CLOSED;

// ====== 유틸 ======
float measureDistanceMM() {
  digitalWrite(PIN_TRIG, LOW);  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(PULSE_US);
  digitalWrite(PIN_TRIG, LOW);
  unsigned long duration = pulseIn(PIN_ECHO, HIGH, ECHO_TIMEOUT_US);
  if (duration == 0) return NAN;
  return 0.173f * (float)duration; // ≈ 0.173 mm/us
}

float emaUpdate(float prev, float sample) {
  if (isnan(sample)) return prev;
  if (!emaInited) { emaInited = true; return sample; }
  return EMA_ALPHA * sample + (1.0f - EMA_ALPHA) * prev;
}

// Smootherstep easing: 6u^5 − 15u^4 + 10u^3 (양끝 완만, 중앙 부드럽게)
static inline float easeSmootherstep(float u) {
  if (u < 0.0f) u = 0.0f;
  if (u > 1.0f) u = 1.0f;
  return u*u*u*(u*(u*6 - 15) + 10);
}

void startMove(int newTarget) {
  startAngle  = gate.read();
  targetAngle = newTarget;
  moveStart   = millis();
  state = (targetAngle > startAngle) ? OPENING : CLOSING;
}

void updateMove() {
  if (state == IDLE) return;

  unsigned long duration = (state == OPENING) ? OPEN_MS : CLOSE_MS;
  unsigned long elapsed  = millis() - moveStart;

  if (elapsed >= duration) {
    gate.write(targetAngle);
    state = IDLE;
    return;
  }

  float u = (float)elapsed / (float)duration;
  float e = easeSmootherstep(u);
  float angle = startAngle + (targetAngle - startAngle) * e;
  gate.write((int)(angle + 0.5f));
}

void setup() {
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  gate.attach(PIN_SERVO);
  gate.write(ANGLE_CLOSED);
  delay(300);
}

void loop() {
  unsigned long now = millis();
  if (now - lastSample >= INTERVAL) {
    lastSample = now;
    float d = measureDistanceMM();
    distEma = emaUpdate(distEma, d);

    if (state == IDLE) {
      if (distEma <= OPEN_DIST_MM) {
        if (gate.read() != ANGLE_OPENED) startMove(ANGLE_OPENED);
      } else if (distEma >= CLOSE_DIST_MM) {
        if (gate.read() != ANGLE_CLOSED) startMove(ANGLE_CLOSED);
      }
    }
  }
  updateMove();
}
