/*
 * Arduino program
 * 
 */

int i;

union eusartTransmit {
    unsigned char raw;

    struct split {
        unsigned int address : 2;
        unsigned int data : 6;
    } split;
} eusartTransmit;

void setup() {
  Serial.begin(115200);

  delay(500);
  Serial.println("hello");
}

void loop() {
  eusartTransmit.split.address = 0b00;
  eusartTransmit.split.data = 0b100000;
  Serial.write(eusartTransmit.raw);
  delay(3000);
  eusartTransmit.split.data = 0b000000;
  Serial.write(eusartTransmit.raw);
  delay(500);

  
  eusartTransmit.split.address = 0b10;
  eusartTransmit.split.data = 0b100000;
  Serial.write(eusartTransmit.raw);
  delay(3000);
  eusartTransmit.split.data = 0b000000;
  Serial.write(eusartTransmit.raw);
  delay(500);
}
