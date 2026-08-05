#pragma once

// Shared wire format for the Moire disc remote link.
//
// This file is duplicated byte-for-byte in both projects:
//   04-controller/include/moire_link.h   (XIAO ESP32-C3, sender)
//   04-Moire disc/include/moire_link.h   (XIAO ESP32-C6, receiver)
// If you edit one, copy it to the other or the two ends stop agreeing.

#include <stdint.h>

// Both boards park on this channel; neither joins an AP, so it has to be
// pinned explicitly on each end or ESP-NOW never hears anything.
static const uint8_t  MOIRE_WIFI_CHANNEL   = 1;

static const uint32_t MOIRE_MAGIC          = 0x4D4F4952UL;  // "MOIR"
static const uint8_t  MOIRE_PROTO_VERSION  = 1;
static const uint32_t MOIRE_SEND_PERIOD_MS = 20;            // 50 Hz

// The three controller states, cycled by the button and shown on its LED.
enum MoireMode : uint8_t {
  MODE_SPEED = 0,  // knobs drive the two motors directly, bidirectional
  MODE_COLOR = 1,  // knobs drive the LED strip hue and hue spread
  MODE_AUTO  = 2,  // knobs drive a sine speed sweep on both motors
  MODE_COUNT = 3,
};

// The controller sends raw (filtered) knob readings and lets the disc do all
// the mapping, so the packet stays mode-agnostic and the disc owns its own
// step-rate limits.
struct __attribute__((packed)) MoirePacket {
  uint32_t magic;
  uint8_t  version;
  uint8_t  mode;      // MoireMode
  uint8_t  button;    // 1 while held; spare, nothing acts on it yet
  uint8_t  reserved;
  uint16_t knob1;     // 0..4095
  uint16_t knob2;     // 0..4095
  uint32_t seq;       // increments every send, for loss reporting
};

static_assert(sizeof(MoirePacket) == 16, "MoirePacket size must match on both ends");

static const uint16_t MOIRE_KNOB_MAX = 4095;
