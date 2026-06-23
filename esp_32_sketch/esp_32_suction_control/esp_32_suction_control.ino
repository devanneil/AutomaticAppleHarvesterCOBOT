const uint8_t HEADER = 0xAA;

const int relayPins[4] = {15, 16, 17, 18};
uint8_t relayState = 0;

void setup() {
  Serial.begin(115200);

  for (int i = 0; i < 4; i++) {
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i], LOW);
  }
}

void loop() {
  if (Serial.available() >= 2) {
    uint8_t header = Serial.read();

    if (header == HEADER) {
      uint8_t state = Serial.read();
      applyRelayState(state);

      // optional ACK
      Serial.write(HEADER);
      Serial.write(relayState);
    }
  }
}

void applyRelayState(uint8_t state) {
  relayState = state;

  for (int i = 0; i < 4; i++) {
    digitalWrite(relayPins[i], (state >> i) & 0x01);
  }
}