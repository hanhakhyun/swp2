/*
 * 11P07: 초음파 센서 → 서보 각도 (연속 매핑 + 안튀게 만들기)
 * - 18 cm(180 mm) → 0°
 * - 36 cm(360 mm) → 180°
 * - 그 사이: 선형으로 연속 변화
 * - 노이즈 억제: 범위필터 + EMA + 각도 스루레이트 + 데드밴드
 */

#include <Servo.h>

// ====== 핀 설정 ======
#define PIN_LED   9
#define PIN_TRIG  12
#define PIN_ECHO  13
#define PIN_SERVO 10

// ====== 초음파/샘플링 ======
#define SND_VEL 346.0
#define INTERVAL 25            // ms
#define PULSE_DURATION 10      // us
#define _DIST_MIN 180.0        // mm = 18 cm -> 0°
#define _DIST_MAX 360.0        // mm = 36 cm -> 180°
#define TIMEOUT ((INTERVAL/2)*1000UL)  // us
#define SCALE (0.001 * 0.5 * SND_VEL)  // mm/us (≈0.173)

// ====== 필터/스무딩 ======
#define _EMA_ALPHA 0.25        // EMA 가중치(낮을수록 더 부드러움)
#define MAX_DEG_PER_SEC 120.0  // 각도 변화 최대 속도(°/s) - 튐 방지 핵심
#define ANGLE_DEADBAND 0.5     // 목표와 현재가 0.5° 이내면 움직이지 않음

// ====== 서보 펄스폭(서보마다 보정 필요) ======
#define _DUTY_MIN 1000   // 0°
#define _DUTY_NEU 1500   // 90°
#define _DUTY_MAX 2000   // 180°

Servo myservo;

// ====== 상태 변수 ======
unsigned long last_sampling_time = 0;
float dist_prev = _DIST_MIN;      // 범위필터 후 최근값(mm)
float dist_ema  = _DIST_MIN;      // EMA 결과(mm)
float angle_curr = 0.0;           // 현재 서보 각도(스무딩 적용)
float angle_target = 0.0;         // 목표 각도(연속 매핑)

// ====== 유틸 ======
static inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

// 초음파 측정(mm)
float USS_measure(int TRIG, int ECHO) {
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);
  unsigned long dur = pulseIn(ECHO, HIGH, TIMEOUT); // us
  return dur * SCALE; // mm
}

void setup() {
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, HIGH); // active-low: OFF
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);

  myservo.attach(PIN_SERVO);
  myservo.writeMicroseconds(_DUTY_MIN); // 시작을 0°로
  angle_curr = 0.0;
  angle_target = 0.0;

  Serial.begin(57600);
  last_sampling_time = millis();
}

void loop() {
  // 샘플링 주기 제어
  if (millis() - last_sampling_time < INTERVAL) return;
  last_sampling_time += INTERVAL;

  // 1) 초음파 원시값
  float dist_raw = USS_measure(PIN_TRIG, PIN_ECHO);

  // 2) 범위 필터 (18~36 cm 밖이면 이전값 유지)
  float dist_filtered;
  if (dist_raw == 0.0 || dist_raw < _DIST_MIN || dist_raw > _DIST_MAX) {
    dist_filtered = dist_prev;
  } else {
    dist_filtered = dist_raw;
    dist_prev = dist_raw;
  }

  // 3) EMA 필터
  dist_ema = _EMA_ALPHA * dist_filtered + (1.0f - _EMA_ALPHA) * dist_ema;

  // 4) 거리→목표 각도(연속 선형 매핑)
  if (dist_ema <= _DIST_MIN) {
    angle_target = 0.0f;
  } else if (dist_ema >= _DIST_MAX) {
    angle_target = 180.0f;
  } else {
    angle_target = (dist_ema - _DIST_MIN) * (180.0f / (_DIST_MAX - _DIST_MIN));
  }
  angle_target = clampf(angle_target, 0.0f, 180.0f);

  // 5) 각도 스루레이트(튀는 현상 억제)
  float max_step = (MAX_DEG_PER_SEC * (INTERVAL / 1000.0f)); // 이번 루프에서 허용되는 최대 변화량(°)
  float err = angle_target - angle_curr;

  if (fabs(err) <= ANGLE_DEADBAND) {
    // 아주 작은 변화는 무시 (데드밴드)
    // angle_curr 그대로 유지
  } else {
    // 허용 범위 내에서만 점진적으로 따라감
    if (err >  max_step) err =  max_step;
    if (err < -max_step) err = -max_step;
    angle_curr += err;
  }

  // 6) 각도→펄스폭 변환 & 서보 구동
  int duty_us = (int)(_DUTY_MIN + (angle_curr / 180.0f) * (_DUTY_MAX - _DUTY_MIN));
  myservo.writeMicroseconds(duty_us);

  // (옵션) LED: 유효범위 안이면 ON
  if (dist_raw >= _DIST_MIN && dist_raw <= _DIST_MAX && dist_raw != 0.0) {
    digitalWrite(PIN_LED, LOW);   // ON (active-low)
  } else {
    digitalWrite(PIN_LED, HIGH);  // OFF
  }

  // 7) 시리얼 출력 (플로터용)
  Serial.print("Min:");     Serial.print(_DIST_MIN);
  Serial.print(",dist:");   Serial.print(dist_raw);
  Serial.print(",ema:");    Serial.print(dist_ema);
  Serial.print(",Servo:");  Serial.print(angle_curr);   // 실제 구동 각도
  Serial.print(",Max:");    Serial.print(_DIST_MAX);
  Serial.println();
}
