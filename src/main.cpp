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
// Resolution sets the usable frequency range, and the duty resolution is
// otherwise irrelevant here -- the step pin only ever emits a 50% square wave.
// The C6's LEDC timers are 20 bits wide; 14 of them puts MAX_STEP_HZ and
// MIN_STEP_HZ comfortably inside the achievable band (see the asserts below).
//
// This was 15 bits, which looked fine against an 80 MHz source but is not what
// the peripheral actually runs on -- see LEDC_SRC_HZ. The real ceiling was
// 1220 Hz, so every request above it was rejected, the motor held its last good
// rate, and the retry logged an error every tick.
const uint8_t  STEP_RES_BITS = 14;
const uint32_t STEP_DUTY     = 1UL << (STEP_RES_BITS - 1);  // 50% square wave

const int32_t MAX_STEP_HZ = 2000;  // full knob deflection
const int32_t MIN_STEP_HZ = 5;     // below this the motor is simply stopped

// f = LEDC_SRC_HZ / (div * 2^bits), with div in [1, 1024].
//
// LEDC_SRC_HZ is the crystal, not the 80 MHz APB: the Arduino core pins LEDC to
// XTAL on every chip that supports it, the C6 included.
//
//   #ifdef SOC_LEDC_SUPPORT_XTAL_CLOCK
//   #define LEDC_DEFAULT_CLK LEDC_USE_XTAL_CLK
//     -- cores/esp32/esp32-hal-ledc.c
//
// The asserts turn a future mismatch between the resolution and the step-rate
// limits into a build failure, rather than a run of ledcChangeFrequency errors
// and a silently dead stretch at one end of the knob.
const uint32_t LEDC_SRC_HZ   = 40000000UL;
const int32_t  LEDC_TOP_HZ   = LEDC_SRC_HZ >> STEP_RES_BITS;            // div = 1
const int32_t  LEDC_FLOOR_HZ = (LEDC_SRC_HZ >> STEP_RES_BITS) / 1024;   // div = 1024

static_assert(MAX_STEP_HZ <= LEDC_TOP_HZ,
              "MAX_STEP_HZ is above the LEDC ceiling - lower STEP_RES_BITS");
static_assert(MIN_STEP_HZ >= LEDC_FLOOR_HZ,
              "MIN_STEP_HZ is below the LEDC floor - raise STEP_RES_BITS");

// Knob mapping. The deadband is what makes "zero at the center detent" usable
// given ADC noise, and it also guarantees the motor is stopped whenever the
// direction pin flips.
const int32_t KNOB_CENTER   = 2048;
const int32_t KNOB_DEADBAND = 150;

const float AUTO_FREQ_MIN_HZ = 0.02f;               // 50 s period
const float AUTO_FREQ_MAX_HZ = 0.5f;                // 2 s period
const float AUTO_PHASE_LAG   = (float)M_PI / 2.0f;  // motor 2 runs 90 deg ahead

// The sine trough. Deliberately well clear of zero: LEDC rejects a frequency of
// 0, and a sweep that dips under MIN_STEP_HZ makes setMotor stop the motor,
// drop applied[] to 0 and then restart it through the direction block once per
// cycle. This is the one number to tune if the sweep looks too flat or the
// motor struggles at the bottom.
const float AUTO_FLOOR_HZ = 500.0f;

const unsigned long LINK_TIMEOUT_MS  = 500;
const unsigned long UPDATE_PERIOD_MS = 20;  // matches the controller's send rate

// The colour pattern travels round the ring in every mode, controller or not.
// The knobs set what the pattern is; this sets how fast it turns.
const float RING_ROT_PERIOD_S = 24.0f;  // seconds for one lap of the ring

// --- Standalone mode -------------------------------------------------------
// The disc is a display piece first: with no controller powered on it still has
// to run on its own, from cold boot onwards. The two motors turn at the same
// fixed rate in opposite senses, and the strip walks a full rainbow round.
const int32_t STANDALONE_STEP_HZ    = 1000;
const uint16_t STANDALONE_SPREAD    = 65535;  // one full rainbow across the ring
const float    STANDALONE_ROT_PERIOD_S = 30.0f;  // seconds per hue revolution

Adafruit_NeoPixel pixels(NUM_PIXELS, PIXEL_PIN, NEO_GRB + NEO_KHZ800);

const int STEP_PINS[2] = {STEP1_PIN, STEP2_PIN};
const int DIR_PINS[2]  = {DIR1_PIN, DIR2_PIN};

// --- Latched state, one set per mode ---------------------------------------
// Changing modes on the controller must not disturb the other subsystems, so
// every mode keeps its own parameters and the disc keeps acting on all of them.
int32_t  speedTarget[2] = {0, 0};                 // MODE_SPEED, signed steps/s
int32_t  motorCmd[2]    = {0, 0};                 // last rate handed to setMotor,
                                                  // whatever mode produced it
uint16_t colorHue = 21845, colorSpread = 21845;   // MODE_COLOR, reproduces the
                                                  // old green->blue gradient
float    autoFreqHz = 0.1f, autoAmpHz = 0.0f;     // MODE_AUTO
float    autoPhase  = 0.0f;                       // keeps running in COLOR mode
float    ringPhase  = 0.0f;                       // pattern centre, in pixels;
                                                  // advances in every mode

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

  motorCmd[idx] = hz;  // for the readout; speedTarget[] is SPEED mode's only

  bool forward = (hz >= 0);
  int32_t mag = forward ? hz : -hz;
  if (mag > MAX_STEP_HZ) mag = MAX_STEP_HZ;
  if (mag < MIN_STEP_HZ) mag = 0;
  int32_t signedHz = forward ? mag : -mag;

  // These two aren't about CPU cost. ledcChangeFrequency() reconfigures the
  // timer wholesale rather than poking a divider, which disturbs the pulse in
  // flight -- so calling it on an unchanged value, or on the 1 Hz of ADC dither
  // that survives the controller's smoothing, would put a glitch in the step
  // train 50 times a second. Skip both cases and rate changes stay clean.
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

  // DIR flips live, mid pulse train. Tested on the machine: the drivers take it
  // at full speed. Motor 2's DIR line is wired the other way round.
  digitalWrite(DIR_PINS[idx], (idx == 1 ? !forward : forward) ? HIGH : LOW);

  ledcChangeFrequency(STEP_PINS[idx], (uint32_t)mag, STEP_RES_BITS);
  ledcWrite(STEP_PINS[idx], STEP_DUTY);
  applied[idx] = signedHz;
}

void setupMotors() {
  pinMode(DIR1_PIN, OUTPUT);
  pinMode(DIR2_PIN, OUTPUT);
  digitalWrite(DIR1_PIN, HIGH);
  digitalWrite(DIR2_PIN, LOW);

  // The two frequencies below differ on purpose, and it is load-bearing.
  // ledcAttach() reuses an existing LEDC timer whenever it finds one with a
  // matching frequency and resolution, and two channels sharing a timer means
  // changing one motor's speed silently changes the other's. Attaching at
  // different rates forces the second pin onto its own timer.
  ledcAttach(STEP1_PIN, 1000, STEP_RES_BITS);
  ledcAttach(STEP2_PIN, 1001, STEP_RES_BITS);
  ledcWrite(STEP1_PIN, 0);
  ledcWrite(STEP2_PIN, 0);

  // ...and check that it actually happened, rather than trusting it. If the two
  // channels did share a timer, AUTO is where it would show first: it is the
  // only mode that retunes both motors every tick, so each call would drag the
  // other along and the phase offset would collapse. Duty is still 0 here, so
  // the motors stay put during the test.
  ledcChangeFrequency(STEP1_PIN, 800, STEP_RES_BITS);
  ledcChangeFrequency(STEP2_PIN, 1600, STEP_RES_BITS);
  uint32_t f1 = ledcReadFreq(STEP1_PIN);
  uint32_t f2 = ledcReadFreq(STEP2_PIN);
  Serial.print("timer check: M1 asked 800 got ");
  Serial.print(f1);
  Serial.print(", M2 asked 1600 got ");
  Serial.println(f2);
  if (!(f1 > 700 && f1 < 900 && f2 > 1500 && f2 < 1700)) {
    Serial.println("WARNING: step channels share a timer, AUTO phase will collapse");
  }
}

// --- LED strip -------------------------------------------------------------
// `hue` is the colour at the pattern's centre and `spread` is how far the hue
// has travelled by the time it reaches the far side of the ring, so the pattern
// is symmetric either way round from the centre.
//
// `rot` is where that centre currently sits, in pixels around the ring. Because
// the pixels form a circle the pattern has no ends to fall off -- the point
// opposite the centre is the seam, and both sides arrive at the same hue there,
// so it stays continuous as the whole thing turns.
void renderStrip(uint16_t hue, uint16_t spread, float rot) {
  const float HALF = NUM_PIXELS / 2.0f;
  for (int i = 0; i < NUM_PIXELS; i++) {
    // Distance from the centre the short way round, 0 .. HALF.
    float around = fmodf((float)i - rot + NUM_PIXELS, (float)NUM_PIXELS);
    float d      = (around > HALF) ? (NUM_PIXELS - around) : around;

    uint16_t h = hue + (uint16_t)(spread * d / HALF);
    pixels.setPixelColor(i, pixels.gamma32(pixels.ColorHSV(h, 255, 255)));
  }
  pixels.show();
}

// Slow hue rotation for standalone mode. The offset is integrated rather than
// derived from millis() so the colours do not jump when the link drops or comes
// back partway through a revolution.
void renderStandalone(float dt) {
  static float hueOffset = 0.0f;

  hueOffset += 65536.0f * dt / STANDALONE_ROT_PERIOD_S;
  while (hueOffset >= 65536.0f) hueOffset -= 65536.0f;

  // Standalone gets both: the pattern turns like everywhere else, and the hue
  // underneath it drifts as well.
  renderStrip((uint16_t)hueOffset, STANDALONE_SPREAD, ringPhase);
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

    // A sine between AUTO_FLOOR_HZ and the peak knob 2 asks for -- the shape
    // from the bench test, rather than the old (1-cos)/2 that swept down to a
    // standstill and made the motor stop and restart every cycle.
    float peak = autoAmpHz;
    if (peak < AUTO_FLOOR_HZ) peak = AUTO_FLOOR_HZ;
    float center = (peak + AUTO_FLOOR_HZ) * 0.5f;
    float amp    = (peak - AUTO_FLOOR_HZ) * 0.5f;

    setMotor(0, (int32_t)(center + amp * sinf(autoPhase)));
    setMotor(1, (int32_t)(center + amp * sinf(autoPhase + AUTO_PHASE_LAG)));
  } else {
    setMotor(0, speedTarget[0]);
    setMotor(1, speedTarget[1]);
  }
}

void setup() {
  Serial.begin(115200);

  // Serial here is the native USB CDC (ARDUINO_USB_CDC_ON_BOOT), not a UART.
  // With no host draining the endpoint its ring buffer fills, and every write
  // then blocks for up to 20 * tx_timeout_ms -- seconds, in the worst case.
  // That stalls loop(), and because the hue rotation is integrated from dt, the
  // whole stall lands as one visible jump. Zero means "drop it if nobody is
  // listening", which is the right trade for a piece running on its own.
  Serial.setTxTimeoutMs(0);

  setupMotors();

  pixels.begin();
  pixels.setBrightness(60);
  renderStrip(colorHue, colorSpread, ringPhase);

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

  unsigned long now = millis();
  if (now - lastUpdate < UPDATE_PERIOD_MS) return;
  float dt = (now - lastUpdate) / 1000.0f;
  lastUpdate = now;

  // Anything that stalls the loop would otherwise be replayed as a single large
  // step in the hue and auto-sweep phase. Capping dt turns a stall into a brief
  // slow-down instead of a jump; at the normal 20 ms tick this never fires.
  if (dt > 0.1f) dt = 0.1f;

  // Advanced here rather than in either branch, so the pattern keeps turning at
  // the same rate across a link drop or recovery.
  ringPhase += NUM_PIXELS * dt / RING_ROT_PERIOD_S;
  while (ringPhase >= NUM_PIXELS) ringPhase -= NUM_PIXELS;

  if (rxPending) {
    MoirePacket pkt;
    noInterrupts();
    memcpy(&pkt, (const void *)&rxPacket, sizeof(pkt));
    rxPending = false;
    interrupts();
    applyPacket(pkt);

    if (!linked) {
      linked = true;
      Serial.println("Link up.");
    }
  }

  // Testing `linked` as well as the timeout is what makes standalone mode start
  // at boot: lastRxMs is still 0 then, so the timeout on its own would leave the
  // disc idle for the first LINK_TIMEOUT_MS.
  if (!linked || now - lastRxMs > LINK_TIMEOUT_MS) {
    if (linked) {
      linked = false;
      Serial.println("Link lost - running standalone.");
    }
    // Motor 2 runs the opposite way here, and only here. The controller modes
    // above are untouched by this.
    setMotor(0, STANDALONE_STEP_HZ);
    setMotor(1, -STANDALONE_STEP_HZ);
    renderStandalone(dt);
  } else {
    updateMotors(dt);
    renderStrip(colorHue, colorSpread, ringPhase);
  }

  // --- Serial readout (throttled so it doesn't flood the monitor) ---
  // `if (Serial)` is false until a host opens the port, so with nothing attached
  // the readout is skipped outright rather than formatted and thrown away.
  static unsigned long lastPrint = 0;
  if (Serial && now - lastPrint >= 250) {
    lastPrint = now;
    if (!linked) {
      Serial.print("STANDALONE\tM1/M2: ");
      Serial.print(STANDALONE_STEP_HZ);
      Serial.println(" steps/s\thue rotating - no controller");
    } else {
      Serial.print("MODE: ");
      Serial.print(activeMode == MODE_SPEED   ? "SPEED"
                   : activeMode == MODE_COLOR ? "COLOR"
                                              : "AUTO");
      Serial.print("\tM1: ");
      Serial.print(motorCmd[0]);
      Serial.print("\tM2: ");
      Serial.print(motorCmd[1]);
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
