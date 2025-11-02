#include <Servo.h>

// ========= 모드 선택 =========
// 1) 60초 동안 0° -> 180° (3 °/s)
// 2) 300초 동안 0° -> 90° (0.3 °/s)
// 3) 사용자 정의 (아래 USER CONFIG 사용)
#define EXPERIMENT_MODE 2

// ========= 공통 설정 =========
#define PIN_SERVO    10
#define INTERVAL_MS  10   // 갱신 주기(ms): 5~20 권장

// ========= USER CONFIG (MODE 3에서만 사용) =========
#define USER_ANGLE_START  0.0f     // 시작 각도
#define USER_ANGLE_TARGET 90.0f    // 목표 각도
#define USER_SPEED_DPS    1.0f     // 등속(°/s)

// ========= 내부 상수 =========
static const float US_MIN = 500.0f;     // 펄스 하한(µs) — 서보에 맞게 600/2400 등으로 조절 가능
static const float US_MAX = 2500.0f;    // 펄스 상한(µs)
static const float US_PER_DEG = 2000.0f / 180.0f; // 1°당 µs 변화

// ========= 내부 변수 =========
Servo s;
unsigned long t0_ms, last_tick;
float start_deg, target_deg, speed_dps, dir; // dir = +1 or -1
float us_start, us_target;
float last_us_written; // 마지막으로 write한 정수 µs

// 각→µs (float)
static inline float deg2us(float deg){
  deg = constrain(deg, 0.0f, 180.0f);
  return US_MIN + deg * US_PER_DEG;
}

void setup(){
  Serial.begin(115200);
  s.attach(PIN_SERVO, (int)US_MIN, (int)US_MAX);

  // ----- 모드 프리셋 -----
  if (EXPERIMENT_MODE == 1){
    // 60초 동안 0° -> 180° (3 °/s)
    start_deg = 0.0f;     target_deg = 180.0f;  speed_dps = 3.0f;
  } else if (EXPERIMENT_MODE == 2){
    // 300초 동안 0° -> 90° (0.3 °/s)
    start_deg = 0.0f;     target_deg = 90.0f;   speed_dps = 0.3f;
  } else {
    // 사용자 정의
    start_deg = USER_ANGLE_START;
    target_deg = USER_ANGLE_TARGET;
    speed_dps = USER_SPEED_DPS;
  }

  dir       = (target_deg >= start_deg) ? 1.0f : -1.0f;
  us_start  = deg2us(start_deg);
  us_target = deg2us(target_deg);

  t0_ms = millis();
  last_tick = t0_ms;
  last_us_written = roundf(us_start);
  s.writeMicroseconds((int)last_us_written);

  Serial.println("== CONST SPEED (MODE) START ==");
  Serial.print("start=");  Serial.print(start_deg);
  Serial.print("  target="); Serial.print(target_deg);
  Serial.print("  speed(dps)="); Serial.println(speed_dps);
}

void loop(){
  unsigned long now = millis();
  if (now - last_tick < INTERVAL_MS) return;
  last_tick = now;

  // 등속: 절대시간 기반 각도 = 시작각 + dir * 속도 * 시간
  float elapsed_s   = (now - t0_ms) / 1000.0f;
  float desired_deg = start_deg + dir * speed_dps * elapsed_s;

  // 목표 넘지 않도록 클램프
  if (dir > 0) desired_deg = min(desired_deg, target_deg);
  else         desired_deg = max(desired_deg, target_deg);

  float desired_us = deg2us(desired_deg);
  desired_us = constrain(desired_us, US_MIN, US_MAX);

  // 1µs 계단 추종: 매우 느린 속도에서도 평균 속도 일정하게
  float delta = desired_us - last_us_written;
  if (fabs(delta) >= 1.0f){
    int step = (delta > 0 ? 1 : -1);
    last_us_written += step;
    s.writeMicroseconds((int)last_us_written);
  }

  // 디버깅 출력
  Serial.print("deg="); Serial.print(desired_deg, 3);
  Serial.print("  us(desired)="); Serial.print((int)roundf(desired_us));
  Serial.print("  us(written)="); Serial.println((int)last_us_written);

  // 종료 조건 도달 시 필요하면 분리/정지 처리
  // if ((int)roundf(last_us_written) == (int)roundf(us_target)) {
  //   s.detach();
  //   while (1) {}  // 끝에서 멈추고 싶다면 사용
  // }
}
