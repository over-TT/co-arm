#include <Arduino.h>
#include <ArmHatController.h>

armhat::ArmHatRuntime armController(Serial, Serial1);

void setup() {
  // UART0 is the polling-only host link. Do not attach both the Pi GPIO UART
  // and the ESP32 Type-C USB serial bridge at the same time.
  Serial.begin(115200);
  armController.begin(1000000, 18, 19);
}

void loop() {
  armController.poll();
  armController.tick();
  // Read native signed absolute feedback for tracked servos. Runs after safety
  // supervision and never inside a host command, so it only adds bus reads.
  armController.sampleOdometers();
  // Safety supervision runs every millisecond while the idle task gets CPU
  // time instead of leaving this core in a full-speed busy loop.
  delay(1);
}
