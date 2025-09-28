#define PIN_LED 7
unsigned int count, toggle, sec;

void setup() {
  pinMode(PIN_LED, OUTPUT);
 
   toggle = 1;
   count = 0;
   sec = 1000;
}

void loop() {
  if(count < 12) {
    toggle = toggle_state(toggle);
    digitalWrite(PIN_LED, toggle);
    delay(sec);
    sec = sec_state(sec);
    count++;
  }
  else {
    while(1){
      
      }
    }
}

int toggle_state(int toggle) {
  return !toggle;
  }

 int sec_state(int sec) {
  return 100;
  }

