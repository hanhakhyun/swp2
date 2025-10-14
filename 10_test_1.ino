#include <Servo.h>
#include <math.h>

// ====== 핀 설정 ======
#define PIN_SERVO 10
#define PIN_TRIG  12
#define PIN_ECHO  13

// ====== 서보/동작 파라미터 ======
static const int   ANGLE_CLOSED = 0;    // 닫힘 각도 (차단기 내려감)
static const int   ANGLE_OPENED = 90;   // 열림 각도 (차단기 올라감) — 서보 설치에 맞게 조정
static const unsigned long MOVE_MS = 1200; // 1.2초로 충분히 부드럽게

// ====== 거리/센서 파라미터 ======
static const float SND_VEL  = 346.0;     // m/s (24℃ 기준)
static const int   INTERVAL = 25;        // 센서 샘플링 주기(ms)
static const int   PULSE_US = 10;        // 트리거 펄스(us)
static const float EMA_ALPHA = 0.3;      // EMA 가중치 (0~1), 1이면 필터 해제
// 히스테리시스: 가까워지면 OPEN_DIST 이하에서 열고, 멀어지면 CLOSE_DIST 이상에서 닫음
static const int   OPEN_DIST_MM  = 220;  // 이 거리 이하면 "열기"
static const int   CLOSE_DIST_MM = 260;  // 이 거리 이상이면 "닫기"
// pulseIn 타임아웃(us): INTERVAL의 절반 정도 권장
static const unsigned long ECHO_TIMEOUT_US = (INTERVAL / 2) * 1000UL;

// ====== 전역 ======
Servo gate;
unsigned long lastSample = 0;
bool emaInited = false;
float distEma = 9999.0;

// 모션 상태
enum GateState {IDLE, OPENING, CLOSING};
GateState state = IDLE;
unsigned long moveStart = 0;
int startAngle = ANGLE_CLOSED;
int targetAngle = ANGLE_CLOSED;

// ====== 유틸 ======
float measureDistanceMM() {
  // 트리거
  digitalWrite(PIN_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_TRIG, HIGH);
  delayMicroseconds(PULSE_US);
  digitalWrite(PIN_TRIG, LOW);

  // 에코 시간 측정(us)
  unsigned long duration = pulseIn(PIN_ECHO, HIGH, ECHO_TIMEOUT_US);
  if (duration == 0) return NAN; // 타임아웃

  // 거리(mm) = (음속 m/s * 1000 mm/m * 왕복시간 us * 1e-6 s/us) / 2
  // = 0.5 * 346000 * duration * 1e-6 = 0.173 * duration
  return 0.173f * (float)duration;
}

float emaUpdate(float prev, float sample) {
  if (isnan(sample)) return prev; // 타임아웃은 EMA 유지
  if (!emaInited) { emaInited = true; return sample; }
  return EMA_ALPHA * sample + (1.0f - EMA_ALPHA) * prev;
}

// Sigmoid easing: u∈[0,1] -> [0,1], 중앙 가속·후반 감속이 자연스럽게
// 로지스틱 정규화: 중앙(0.5) 대칭, k로 곡률 조절
float easeSigmoid(float u) {
  const float k = 8.0f; // 6~10 사이 권장 (값↑ => 중앙 가속감↑, 양끝 더 부드러움)
  // logistic(x)=1/(1+e^-x), 구간 [0,1]에서 정규화
  float L0 = 1.0f / (1.0f + expf(+k * 0.5f));   // u=0 -> x=-k/2
  float L1 = 1.0f / (1.0f + expf(-k * 0.5f));   // u=1 -> x=+k/2
  float Lu = 1.0f / (1.0f + expf(-k * (u - 0.5f)));
  return (Lu - L0) / (L1 - L0);
}

void startMove(int newTarget) {
  // 현재 각도에서 새 목표로 이동 시작
  startAngle = gate.read(); // 라이브러리 추정값(근사), 시작 직전에 보정
  targetAngle = newTarget;
  moveStart = millis();
  state = (targetAngle > startAngle) ? OPENING : CLOSING;
}

void updateMove() {
  if (state == IDLE) return;
  unsigned long elapsed = millis() - moveStart;
  if (elapsed >= MOVE_MS) {
    gate.write(targetAngle);
    state = IDLE;
    return;
  }
  float u = (float)elapsed / (float)MOVE_MS;        // 0→1
  float e = easeSigmoid(u);                         // 시그모이드 이징
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
  // 1) 주기적으로 거리 샘플링 + EMA
  unsigned long now = millis();
  if (now - lastSample >= INTERVAL) {
    lastSample = now;
    float d = measureDistanceMM();
    distEma = emaUpdate(distEma, d);

    // 2) 상태 전환 (히스테리시스)
    if (state == IDLE) {
      if (distEma <= OPEN_DIST_MM) {
        if (gate.read() != ANGLE_OPENED) startMove(ANGLE_OPENED);
      } else if (distEma >= CLOSE_DIST_MM) {
        if (gate.read() != ANGLE_CLOSED) startMove(ANGLE_CLOSED);
      }
    }
  }

  // 3) 모션 업데이트(부드럽게 이동)
  updateMove();
}
