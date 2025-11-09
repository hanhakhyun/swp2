#include <Servo.h>

// --- Pins ---
#define PIN_IR    A0
#define PIN_LED   9
#define PIN_SERVO 10

// --- LED Active level (배선에 따라 바꾸세요) ---
#define LED_ACTIVE_HIGH 0   // LED가 HIGH에서 켜지면 1, LOW에서 켜지면 0

// --- Servo pulse (µs) ---
#define _DUTY_MIN 544
#define _DUTY_NEU 1500
#define _DUTY_MAX 2400

// --- Distance range (mm) ---
#define _DIST_MIN 100.0f   // 10 cm
#define _DIST_MAX 250.0f   // 25 cm

// --- Filters & loop (빠르게) ---
#define EMA_ALPHA       0.35f   // 반응 빠르게
#define LOOP_INTERVAL   20      // ms

// --- Servo anti-twitch (빠르면서 부드럽게) ---
#define DUTY_LP_GAIN    0.45f   // 듀티 LPF (↑ 빠르게)
#define MAX_STEP_US     30      // 루프당 최대 변화(µs) ≈ 1500 µs/s
#define DUTY_DEADBAND   3       // 미세 변화 무시(µs)

// --- LED hysteresis (깜빡임 방지) ---
#define LED_HYST_MM     5.0f

Servo myservo;
unsigned long last_loop_time;
float dist_ema = _DIST_MIN;
int   duty_curr = _DUTY_MIN;
bool  led_on = false;

static inline float clampf(float x, float lo, float hi){
  if (x < lo) return lo; if (x > hi) return hi; return x;
}
static inline void setLed(bool on){
#if LED_ACTIVE_HIGH
  digitalWrite(PIN_LED, on ? HIGH : LOW);
#else
  digitalWrite(PIN_LED, on ? LOW : HIGH);
#endif
}

void setup(){
  pinMode(PIN_LED, OUTPUT);
  myservo.attach(PIN_SERVO);
  Serial.begin(1000000);

  last_loop_time = millis();

  int a0 = analogRead(PIN_IR);
  if (a0 > 10) {
    float d0 = ((6762.0f / (a0 - 9.0f)) - 4.0f) * 10.0f - 60.0; // 초기 버전 공식
    dist_ema = clampf(d0, _DIST_MIN, _DIST_MAX);
  }
  duty_curr = _DUTY_MIN;
  myservo.writeMicroseconds(duty_curr);
  setLed(false);
}

void loop(){
  unsigned long now = millis();
  if (now - last_loop_time < LOOP_INTERVAL) return;
  last_loop_time += LOOP_INTERVAL;

  int a_value = analogRead(PIN_IR);

  // 이상치 가드
  if (a_value <= 10 || a_value >= 1018){
    setLed(false);
    myservo.writeMicroseconds(duty_curr);
    printFrame(a_value, 0, dist_ema, duty_curr);
    return;
  }

  // 1) IR -> 거리(mm)  (초기 버전 공식: -60 보정 없이)
  float dist_raw = ((6762.0f / (a_value - 9.0f)) - 4.0f) * 10.0f;

  // 2) LED: raw 기준 즉시 판단 + 히스테리시스(더 확실히 켜지게)
  float on_low  = _DIST_MIN + LED_HYST_MM;
  float on_high = _DIST_MAX - LED_HYST_MM;
  if (!led_on && dist_raw >= on_low && dist_raw <= on_high) led_on = true;
  if ( led_on && (dist_raw <= _DIST_MIN || dist_raw >= _DIST_MAX)) led_on = false;
  setLed(led_on);

  // 3) EMA (제어용: 덜 튀게)
  float d_clamped = clampf(dist_raw, _DIST_MIN, _DIST_MAX);
  dist_ema = EMA_ALPHA * d_clamped + (1.0f - EMA_ALPHA) * dist_ema;

  // 4) 거리 → 목표 듀티(µs) (map() 미사용)
  int duty_target;
  float t = (dist_ema - _DIST_MIN) / (_DIST_MAX - _DIST_MIN); // 0~1
  t = clampf(t, 0.0f, 1.0f);
  duty_target = (int)(_DUTY_MIN + (_DUTY_MAX - _DUTY_MIN) * t + 0.5f);

  // 5) 서보: 얕은 LPF + 데드밴드 + 슬루 제한 (반응 빠르게)
  int duty_lpf = duty_curr + (int)((duty_target - duty_curr) * DUTY_LP_GAIN);
  int delta    = duty_lpf - duty_curr;

  if (abs(delta) <= DUTY_DEADBAND) delta = 0;
  if (delta >  MAX_STEP_US) delta =  MAX_STEP_US;
  if (delta < -MAX_STEP_US) delta = -MAX_STEP_US;

  duty_curr += delta;
  if (duty_curr < _DUTY_MIN) duty_curr = _DUTY_MIN;
  if (duty_curr > _DUTY_MAX) duty_curr = _DUTY_MAX;

  myservo.writeMicroseconds(duty_curr);

  // 6) 시리얼 출력(슬라이드 포맷)
  printFrame(a_value, dist_raw, dist_ema, duty_curr);
}

void printFrame(int a_value, float dist_raw, float dist_ema, int duty){
  Serial.print("_DUTY_MIN:");  Serial.print(_DUTY_MIN);
  Serial.print("_DIST_MIN:");  Serial.print(_DIST_MIN);
  Serial.print(",IR:");        Serial.print(a_value);
  Serial.print(",dist_raw:");  Serial.print(dist_raw);
  Serial.print(",ema:");       Serial.print(dist_ema);
  Serial.print(",servo:");     Serial.print(duty);
  Serial.print(",_DIST_MAX:"); Serial.print(_DIST_MAX);
  Serial.print(",_DUTY_MAX:"); Serial.print(_DUTY_MAX);
  Serial.println();
}

