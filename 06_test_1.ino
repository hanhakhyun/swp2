/*
 * 도전과제 2: PWM 함수 구현 (software PWM)
 * - 사용 API: digitalWrite(), delayMicroseconds()
 * - 구현 함수:
 *     void set_period(int period_us); // 100 ~ 10000 us
 *     void set_duty(int duty_pct);    // 0 ~ 100 %
 * - 밝기 제어: 1초 동안 최소→최대→최소 (triangle), 101단계(0~100)
 * - period는 코드 상단 PERIOD_US로 직접 수정(예: 10000, 1000, 100)
 * 20252368 소프트웨어학부 한학현
 */

const int LED_PIN = 7;
#define PERIOD_US 10000   // 바꾸기: 100(0.1ms), 1000(1ms), 10000(10ms)

volatile unsigned int g_period_us = PERIOD_US; // 100~10000
volatile int          g_duty_pct  = 0;         // 0~100

static inline int clamp(int x, int lo, int hi) {
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

void set_period(int period_us) {
  g_period_us = (unsigned int)clamp(period_us, 100, 10000);
}

void set_duty(int duty_pct) {
  g_duty_pct = clamp(duty_pct, 0, 100);
}

// 현재 g_period_us로 PWM 1주기 출력
void pwm_one_cycle(int duty_pct) {
  // 0% / 100%는 분기 (토글 최소화)
  if (duty_pct <= 0) {
    digitalWrite(LED_PIN, LOW);
    delayMicroseconds(g_period_us);
    return;
  }
  if (duty_pct >= 100) {
    digitalWrite(LED_PIN, HIGH);
    delayMicroseconds(g_period_us);
    return;
  }

  unsigned int on_us  = (unsigned long)g_period_us * (unsigned int)duty_pct / 100UL;
  unsigned int off_us = g_period_us - on_us;

  if (on_us)  { digitalWrite(LED_PIN, HIGH); delayMicroseconds(on_us); }
  if (off_us) { digitalWrite(LED_PIN, LOW ); delayMicroseconds(off_us); }
}

// 1초 동안 0→100→0 트라이앵글 (101단계 기준)
// 단계 시퀀스: 0..100 (101단계) + 99..1 (99단계) = 총 200스텝
void run_triangle_1s() {
  const int steps_total = 200;                     // 0..100 + 99..1
  unsigned long cycles_total = 1000000UL / g_period_us;  // 1초 동안 총 사이클 수
  if (cycles_total == 0) cycles_total = 1;

  unsigned long base_cycles = cycles_total / steps_total;  // 스텝당 기본 사이클 수
  unsigned long remainder   = cycles_total % steps_total;  // 앞쪽 remainder 스텝에 +1

  int step_index = 0;

  // 상승: 0 → 100
  for (int d = 0; d <= 100; ++d) {
    set_duty(d);
    unsigned long do_cycles = base_cycles + ((step_index < (int)remainder) ? 1 : 0);
    if (do_cycles == 0) do_cycles = 1;
    for (unsigned long i = 0; i < do_cycles; ++i) pwm_one_cycle(g_duty_pct);
    step_index++;
  }

  // 하강: 99 → 1  (0과 100은 중복 방지)
  for (int d = 99; d >= 1; --d) {
    set_duty(d);
    unsigned long do_cycles = base_cycles + ((step_index < (int)remainder) ? 1 : 0);
    if (do_cycles == 0) do_cycles = 1;
    for (unsigned long i = 0; i < do_cycles; ++i) pwm_one_cycle(g_duty_pct);
    step_index++;
  }

  digitalWrite(LED_PIN, LOW); // 마무리 OFF
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  set_period(PERIOD_US);
}

void loop() {
  run_triangle_1s();
  
}
