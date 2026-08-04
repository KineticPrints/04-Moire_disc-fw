#include <Arduino.h>
#include <Adafruit_NeoPixel.h>

// --- Pin assignments (Seeed XIAO ESP32-C6 aliases) ---
const int DIR1_PIN  = D7;   // motor 1 direction
const int STEP1_PIN = D8;   // motor 1 step
const int DIR2_PIN  = D6;   // motor 2 direction
const int STEP2_PIN = D5;   // motor 2 step
const int PIXEL_PIN = D10;  // NeoPixel data line

const int NUM_PIXELS = 14;

// Step rate in steps per second. The LEDC peripheral generates the step
// pulses in hardware, so the rate stays exact no matter what loop() does.
const uint32_t STEP_RATE_HZ = 1500;
const uint8_t  STEP_RES_BITS = 10;                    // duty range 0..1023
const uint32_t STEP_DUTY = (1 << STEP_RES_BITS) / 2;  // 50% square wave

Adafruit_NeoPixel pixels(NUM_PIXELS, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  Serial.begin(115200);

  // --- Steppers: fixed direction, hardware-generated step pulses ---
  pinMode(DIR1_PIN, OUTPUT);
  pinMode(DIR2_PIN, OUTPUT);
  digitalWrite(DIR1_PIN, HIGH);
  digitalWrite(DIR2_PIN, HIGH);

  ledcAttach(STEP1_PIN, STEP_RATE_HZ, STEP_RES_BITS);
  ledcAttach(STEP2_PIN, STEP_RATE_HZ, STEP_RES_BITS);
  ledcWrite(STEP1_PIN, STEP_DUTY);
  ledcWrite(STEP2_PIN, STEP_DUTY);

  // --- NeoPixels: static green-to-blue gradient across the strip ---
  pixels.begin();
  pixels.setBrightness(60);
  for (int i = 0; i < NUM_PIXELS; i++) {
    uint8_t blend = (255 * i) / (NUM_PIXELS - 1);  // 0 at one end, 255 at the other
    pixels.setPixelColor(i, pixels.Color(0, 255 - blend, blend));
  }
  pixels.show();

  Serial.println("Steppers running at 500 steps/s, 14 LEDs green->blue.");
}

void loop() {
  // Steps are produced by hardware and the strip is static, so there is
  // nothing to do here yet beyond a slow heartbeat on the serial monitor.
  Serial.println("running");
  delay(1000);
}
