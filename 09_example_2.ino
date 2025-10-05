// Arduino pin assignment
#define PIN_LED  9
#define PIN_TRIG 12
#define PIN_ECHO 13

// configurable parameters
#define SND_VEL 346.0
#define INTERVAL 25
#define PULSE_DURATION 10
#define _DIST_MIN 100
#define _DIST_MAX 300
#define TIMEOUT ((INTERVAL / 2) * 1000.0)
#define SCALE (0.001 * 0.5 * SND_VEL)

// ====== 필터 파라미터 ======
#define N 3            // <- 3, 10, 30 으로 바꿔가며 테스트
#define _EMA_ALPHA 0.3    // 0~1 (1이면 EMA 효과 없음)

// globals
unsigned long last_sampling_time;
float samples[N];
int   idx = 0;
bool  filled = false;
bool  ema_inited = false;
float dist_ema = 0.0;

// === USS 측정 함수 ===
float USS_measure(int TRIG, int ECHO) {
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);
  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE; // unit: mm
}

// === 중위수 계산 함수 ===
// (VLA 제거: 고정 길이 배열 N 사용)
float getMedian(const float* buf, int size) {
  float a[N];
  for (int i = 0; i < size; i++) a[i] = buf[i];

  // 간단한 선택 정렬
  for (int i = 0; i < size - 1; i++) {
    int m = i;
    for (int j = i + 1; j < size; j++) {
      if (a[j] < a[m]) m = j;
    }
    if (m != i) { float t = a[i]; a[i] = a[m]; a[m] = t; }
  }

  return (size % 2) ? a[size / 2] : (a[size / 2 - 1] + a[size / 2]) * 0.5f;
}

void setup() {
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
  digitalWrite(PIN_TRIG, LOW);
  Serial.begin(57600);

  last_sampling_time = millis();  // 초기화
}

void loop() {
  if (millis() < last_sampling_time + INTERVAL) return;

  float dist_raw = USS_measure(PIN_TRIG, PIN_ECHO);

  // 유효 범위 체크 (직전 유효값 사용 금지)
  bool valid = (dist_raw >= _DIST_MIN && dist_raw <= _DIST_MAX);

  // EMA 업데이트: 유효 샘플일 때만
  if (valid) {
    if (!ema_inited) { dist_ema = dist_raw; ema_inited = true; }
    else             { dist_ema = _EMA_ALPHA * dist_raw + (1.0 - _EMA_ALPHA) * dist_ema; }

    // median 버퍼에 유효 샘플 저장
    samples[idx++] = dist_raw;
    if (idx >= N) { idx = 0; filled = true; }
  }

  // median 계산 (버퍼 채운 만큼)
  int count = filled ? N : idx;
  float dist_median = (count > 0) ? getMedian(samples, count) : dist_raw;

  // ===== 시리얼 플로터 출력 (19 도전과제 형식 일치) =====
  const float cap = (float)_DIST_MAX + 100.0f;
  Serial.print("Min:");     Serial.print(_DIST_MIN);
  Serial.print(",raw:");    Serial.print(dist_raw);
  Serial.print(",ema:");    Serial.print(dist_ema);
  Serial.print(",median:"); Serial.print(dist_median);
  Serial.print(",Max:");    Serial.print(_DIST_MAX);
  Serial.println("");

  // LED 표시 (유효 시 ON, 무효 시 OFF)
  digitalWrite(PIN_LED, valid ? LOW : HIGH);

  last_sampling_time += INTERVAL;
}
