#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <math.h>

#include "moire_link.h"

// --- Pin assignments (Seeed XIAO ESP32-C6 aliases) ---
const int DIR1_PIN  = D7;   // motor 1 direction
const int STEP1_PIN = D8;   // motor 1 step
const int DIR2_PIN  = D6;   // motor 2 direction
const int STEP2_PIN = D5;   // motor 2 step
const int PIXEL_PIN = D10;  // NeoPixel data line

const int NUM_PIXELS = 14;

// The LEDC peripheral generates the step pulses in hardware, so the rate stays
// exact no matter what loop() does.
//
// Resolution sets the usable frequency range: f = clk / (div * 2^bits), with a
// divider capped at 1024 off an 80 MHz source. At the 10 bits this sketch used
// to run, the slowest possible output is ~76 Hz -- too coarse for a sine that
// sweeps down to zero. The C6's LEDC timers are 20 bits wide, so 15 bits gives
// roughly 2.4 Hz .. 2441 Hz, which brackets MAX_STEP_HZ with room to spare.
const uint8_t  STEP_RES_BITS = 15;
const uint32_t STEP_DUTY     = 1UL << (STEP_RES_BITS - 1);  // 50% square wave

const int32_t MAX_STEP_HZ = 2000;  // full knob deflection
const int32_t MIN_STEP_HZ = 5;     // below this the motor is simply stopped

// Knob mapping. The deadband is what makes "zero at the center detent" usable
// given ADC noise, and it also guarantees the motor is stopped whenever the
// direction pin flips.
const int32_t KNOB_CENTER   = 2048;
const int32_t KNOB_DEADBAND = 150;

const float AUTO_FREQ_MIN_HZ = 0.02f;               // 50 s period
const float AUTO_FREQ_MAX_HZ = 0.5f;                // 2 s period
const float AUTO_PHASE_LAG   = (float)M_PI / 2.0f;  // motor 2 lags by 90 degrees

const unsigned long LINK_TIMEOUT_MS  = 500;
const unsigned long UPDATE_PERIOD_MS = 20;  // matches the controller's send rate

Adafruit_NeoPixel pixels(NUM_PIXELS, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

const int STEP_PINS[2] = {STEP1_PIN, STEP2_PIN};
const int DIR_PINS[2]  = {DIR1_PIN, DIR2_PIN};

// --- Latched state, one set per mode ---------------------------------------
// Changing modes on the controller must not disturb the other subsystems, so
// every mode keeps its own parameters and the disc keeps acting on all of them.
int32_t  speedTarget[2] = {0, 0};                 // MODE_SPEED, signed steps/s
uint16_t colorHue = 21845, colorSpread = 21845;   // MODE_COLOR, reproduces the
                                                  // old green->blue gradient
float    autoFreqHz = 0.1f, autoAmpHz = 0.0f;     // MODE_AUTO
float    autoPhase  = 0.0f;                       // keeps running in COLOR mode

uint8_t activeMode     = MODE_SPEED;
uint8_t lastMotionMode = MODE_SPEED;  // which mode the motors follow in COLOR

// --- Staging buffer written by the ESP-NOW callback ------------------------
// The callback runs in the WiFi task, so it only validates and copies; all the
// mapping and I/O happens in loop().
volatile bool          rxPending = false;
volatile MoirePacket   rxPacket;
volatile uint32_t      rxCount = 0, rxDropped = 0;
volatile unsigned long lastRxMs = 0;

// --- Motors ----------------------------------------------------------------
// The one place that touches the step/dir pins. Sign picks the direction,
// magnitude picks the step rate.
void setMotor(int idx, int32_t hz) {
  static int32_t applied[2] = {0, 0};

  bool forward = (hz >= 0);
  int32_t mag = forward ? hz : -hz;
  if (mag > MAX_STEP_HZ) mag = MAX_STEP_HZ;
  if (mag < MIN_STEP_HZ) mag = 0;
  int32_t signedHz = forward ? mag : -mag;

  // Reconfiguring the timer is not free, so ignore sub-2 Hz wobble.
  if (signedHz == applied[idx]) return;
  if (signedHz != 0 && applied[idx] != 0 && forward == (applied[idx] >= 0) &&
      abs(signedHz - applied[idx]) < 2) {
    return;
  }

  if (mag == 0) {
    ledcWrite(STEP_PINS[idx], 0);
    applied[idx] = 0;
    return;
  }

  // Only ever flip direction while the pulses are stopped.
  if (applied[idx] == 0 || forward != (applied[idx] >= 0)) {
    ledcWrite(STEP_PINS[idx], 0);
    digitalWrite(DIR_PINS[idx], forward ? HIGH : LOW);
  }

  if (ledcChangeFrequency(STEP_PINS[idx], (uint32_t)mag, STEP_RES_BITS) == 0) {
    static bool warned = false;
    if (!warned) {
      warned = true;
      Serial.print("ledcChangeFrequency rejected ");
      Serial.print(mag);
      Serial.println(" Hz");
    }
    return;
  }
  ledcWrite(STEP_PINS[idx], STEP_DUTY);
  applied[idx] = signedHz;
}

void setupMotors() {
  pinMode(DIR1_PIN, OUTPUT);
  pinMode(DIR2_PIN, OUTPUT);
  digitalWrite(DIR1_PIN, HIGH);
  digitalWrite(DIR2_PIN, HIGH);

  // The two frequencies below differ on purpose, and it is load-bearing.
  // ledcAttach() reuses an existing LEDC timer whenever it finds one with a
  // matching frequency and resolution, and two channels sharing a timer means
  // changing one motor's speed silently changes the other's. Attaching at
  // different rates forces the second pin onto its own timer.
  ledcAttach(STEP1_PIN, 1000, STEP_RES_BITS);
  ledcAttach(STEP2_PIN, 1001, STEP_RES_BITS);
  ledcWrite(STEP1_PIN, 0);
  ledcWrite(STEP2_PIN, 0);
}

// --- LED strip -------------------------------------------------------------
void renderStrip(uint16_t hue, uint16_t spread) {
  for (int i = 0; i < NUM_PIXELS; i++) {
    uint16_t h = hue + (uint16_t)((uint32_t)spread * i / (NUM_PIXELS - 1));
    pixels.setPixelColor(i, pixels.gamma32(pixels.ColorHSV(h, 255, 255)));
  }
  pixels.show();
}

// Slow red pulse, so a dead link is obvious from across the room.
void renderDisconnected() {
  float t = (millis() % 2000) / 2000.0f;
  uint8_t level = (uint8_t)(20 + 60 * (0.5f - 0.5f * cosf(2.0f * (float)M_PI * t)));
  for (int i = 0; i < NUM_PIXELS; i++) {
    pixels.setPixelColor(i, pixels.Color(level, 0, 0));
  }
  pixels.show();
}

// --- Radio -----------------------------------------------------------------
void onPacket(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  (void)info;
  if (len != (int)sizeof(MoirePacket)) {
    rxDropped++;
    return;
  }

  MoirePacket pkt;
  memcpy(&pkt, data, sizeof(pkt));
  if (pkt.magic != MOIRE_MAGIC || pkt.version != MOIRE_PROTO_VERSION ||
      pkt.mode >= MODE_COUNT) {
    rxDropped++;
    return;
  }

  memcpy((void *)&rxPacket, &pkt, sizeof(pkt));
  rxPending = true;
  rxCount++;
  lastRxMs = millis();
}

void setupRadio() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.setSleep(false);

  // Neither board joins an AP, so the channel has to be forced by hand.
  esp_wifi_set_channel(MOIRE_WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    return;
  }
  esp_now_register_recv_cb(onPacket);
}

// --- Mode handling ---------------------------------------------------------
// Raw knob -> signed step rate, with a dead zone around the center detent.
int32_t knobToSpeed(uint16_t knob) {
  int32_t centered = (int32_t)knob - KNOB_CENTER;
  if (abs(centered) <= KNOB_DEADBAND) return 0;

  int32_t span   = (centered > 0) ? (MOIRE_KNOB_MAX - KNOB_CENTER - KNOB_DEADBAND)
                                  : (KNOB_CENTER - KNOB_DEADBAND);
  int32_t offset = (centered > 0) ? (centered - KNOB_DEADBAND)
                                  : (centered + KNOB_DEADBAND);
  return (int32_t)((int64_t)offset * MAX_STEP_HZ / span);
}

void applyPacket(const MoirePacket &pkt) {
  activeMode = pkt.mode;

  switch (pkt.mode) {
    case MODE_SPEED:
      speedTarget[0] = knobToSpeed(pkt.knob1);
      speedTarget[1] = knobToSpeed(pkt.knob2);
      lastMotionMode = MODE_SPEED;
      break;

    case MODE_COLOR:
      colorHue    = (uint16_t)((uint32_t)pkt.knob1 * 65535 / MOIRE_KNOB_MAX);
      colorSpread = (uint16_t)((uint32_t)pkt.knob2 * 32768 / MOIRE_KNOB_MAX);
      break;

    case MODE_AUTO:
      autoFreqHz = AUTO_FREQ_MIN_HZ + (AUTO_FREQ_MAX_HZ - AUTO_FREQ_MIN_HZ) *
                                          pkt.knob1 / MOIRE_KNOB_MAX;
      autoAmpHz  = (float)MAX_STEP_HZ * pkt.knob2 / MOIRE_KNOB_MAX;
      lastMotionMode = MODE_AUTO;
      break;
  }
}

void updateMotors(float dt) {
  if (lastMotionMode == MODE_AUTO) {
    // Integrating a phase accumulator (rather than evaluating cos(2*pi*f*t))
    // means turning the frequency knob never causes a jump in speed.
    autoPhase += 2.0f * (float)M_PI * autoFreqHz * dt;
    if (autoPhase > 2.0f * (float)M_PI) autoPhase -= 2.0f * (float)M_PI;

    // (1 - cos)/2 spans 0..1, so each motor sweeps from a standstill up to the
    // amplitude and back, in one direction only.
    float s1 = autoAmpHz * 0.5f * (1.0f - cosf(autoPhase));
    float s2 = autoAmpHz * 0.5f * (1.0f - cosf(autoPhase - AUTO_PHASE_LAG));
    setMotor(0, (int32_t)s1);
    setMotor(1, (int32_t)s2);
  } else {
    setMotor(0, speedTarget[0]);
    setMotor(1, speedTarget[1]);
  }
}

void setup() {
  Serial.begin(115200);

  setupMotors();

  pixels.begin();
  pixels.setBrightness(60);
  renderStrip(colorHue, colorSpread);

  setupRadio();

  Serial.print("Moire disc ready on channel ");
  Serial.print(MOIRE_WIFI_CHANNEL);
  Serial.print(", listening for broadcasts. Max ");
  Serial.print(MAX_STEP_HZ);
  Serial.print(" steps/s, ");
  Serial.print(NUM_PIXELS);
  Serial.println(" LEDs.");
}

void loop() {
  static unsigned long lastUpdate = 0;
  static bool linked = false;
  static uint16_t shownHue = 0xFFFF, shownSpread = 0xFFFF;

  unsigned long now = millis();
  if (now - lastUpdate < UPDATE_PERIOD_MS) return;
  float dt = (now - lastUpdate) / 1000.0f;
  lastUpdate = now;

  if (rxPending) {
    MoirePacket pkt;
    noInterrupts();
    memcpy(&pkt, (const void *)&rxPacket, sizeof(pkt));
    rxPending = false;
    interrupts();
    applyPacket(pkt);

    if (!linked) {
      linked = true;
      shownHue = shownSpread = 0xFFFF;  // force a repaint over the red pulse
      Serial.println("Link up.");
    }
  }

  if (now - lastRxMs > LINK_TIMEOUT_MS) {
    // A controller that browns out must not leave the motors spinning.
    if (linked) {
      linked = false;
      Serial.println("Link lost - motors stopped.");
    }
    setMotor(0, 0);
    setMotor(1, 0);
    renderDisconnected();
  } else {
    updateMotors(dt);
    if (colorHue != shownHue || colorSpread != shownSpread) {
      shownHue    = colorHue;
      shownSpread = colorSpread;
      renderStrip(colorHue, colorSpread);
    }
  }

  // --- Serial readout (throttled so it doesn't flood the monitor) ---
  static unsigned long lastPrint = 0;
  if (now - lastPrint >= 250) {
    lastPrint = now;
    if (!linked) {
      Serial.println("waiting for controller...");
    } else {
      Serial.print("MODE: ");
      Serial.print(activeMode == MODE_SPEED   ? "SPEED"
                   : activeMode == MODE_COLOR ? "COLOR"
                                              : "AUTO");
      Serial.print("\tM1: ");
      Serial.print(speedTarget[0]);
      Serial.print("\tM2: ");
      Serial.print(speedTarget[1]);
      Serial.print("\tHUE: ");
      Serial.print(colorHue);
      Serial.print("/");
      Serial.print(colorSpread);
      Serial.print("\tAUTO: ");
      Serial.print(autoFreqHz, 3);
      Serial.print(" Hz x ");
      Serial.print((int)autoAmpHz);
      Serial.print("\tRX: ");
      Serial.print(rxCount);
      Serial.print(" drop ");
      Serial.println(rxDropped);
    }
  }
}
