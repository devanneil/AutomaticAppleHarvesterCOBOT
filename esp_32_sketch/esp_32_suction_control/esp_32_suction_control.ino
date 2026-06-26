const uint8_t HEADER = 0xAA;

const int relayPins[4] = {15, 16, 17, 18}; // 15, RX, TX, MO
const int switchPins[4] = {19, 23, 21, 22}; // MI, SDL, 21, SCL
uint8_t relayState = 0;

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 4; i++) {
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i], LOW);
  }
  for (int i = 0; i < 4; i++) {
    pinMode(switchPins[i], INPUT_PULLUP);
  }
}

void loop() {
  if (Serial.available() >= 2) {
    uint8_t header = Serial.read();

    if (header == HEADER) {
      uint8_t state = Serial.read();
      applyRelayState(state);
    }
  }
  uint8_t switchState = check_switch_state();

  // optional ACK
  Serial.write(HEADER);
  Serial.write(relayState | (switchState));
}

void applyRelayState(uint8_t state) {
  relayState = state;

  for (int i = 0; i < 4; i++) {
    digitalWrite(relayPins[i], (state >> i) & 0x01);
  }
}

uint8_t check_switch_state()
{
    uint8_t state = 0;

    for (uint8_t i = 0; i < 4; i++) {
        // Active-low switch (INPUT_PULLUP)
        if (digitalRead(switchPins[i]) == LOW) {
            state |= (1 << i);
        }
    }

    return state;
}