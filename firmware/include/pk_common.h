// Shared definitions for nodes and tags.
#pragma once
#include <Arduino.h>
#include "mbedtls/md.h"

// Demo pairing key shared by the child tag and its guardian. In deployment each
// family gets a fresh random key at the registration kiosk; only the guardian
// device (and the reunification desk, if the guardian discloses it) holds it.
static const uint8_t PAIR_KEY[16] = {0x3a, 0x91, 0x5c, 0x07, 0xe2, 0x4f, 0xb8, 0x16,
                                     0xd4, 0x6b, 0x20, 0xaf, 0x75, 0xc9, 0x0e, 0x58};

// ID rotation period. Demo: 60 s so judges can watch it change.
// Deployment: 10-15 min, rotating the MAC at the same instant (GAEN practice).
static const uint32_t ROTATE_S = 60;
static const uint32_t SLOT_SEARCH = 2048;   // slots searched when re-acquiring (~34 h)

static const uint16_t COMPANY_ID = 0xFFFF;  // Bluetooth SIG test ID; replace before deployment
static const uint8_t DOMAIN_TAG_ID = 1, DOMAIN_TAG_ADDR = 2, DOMAIN_GUARDIAN_ID = 3;

static const int LED_PIN = 2;      // on-board LED on most ESP32 dev kits
static const int BUTTON_PIN = 0;   // BOOT button: guardian "with me" / tag demo LOST toggle

// Manufacturer-specific advertising payload (after the 2-byte company ID)
//   [0..1] 'P','K' (tag) or 'P','G' (guardian)
//   [2]    version
//   [3]    flags (bit0 = LOST, bit1 = possible SEPARATION)
//   [4..11] 8-byte rotating ID
//   [12..19] only when a flag is set: up to two 4-byte hashes of learned
//            companion (family-device) addresses, so nodes can look for the family
// 'P','C' = command beacon sent by a node: ID = target tag, flags = command
static const uint8_t FLAG_LOST = 0x01, FLAG_SEPARATED = 0x02;

// Salt for hashing device addresses on nodes and tags (rotate per event in deployment)
static const uint8_t ADDR_SALT[8] = {0x51, 0x0d, 0x9e, 0x22, 0x6c, 0xa7, 0x13, 0xf0};

inline void addr_hash4(const uint8_t addr[6], uint8_t out[4]) {
  uint8_t buf[14], full[32];
  memcpy(buf, ADDR_SALT, 8);
  memcpy(buf + 8, addr, 6);
  mbedtls_md(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), buf, sizeof(buf), full);
  memcpy(out, full, 4);
}

inline void hmac8(uint8_t domain, uint32_t slot, uint8_t out[8]) {
  uint8_t msg[5] = {domain, (uint8_t)(slot >> 24), (uint8_t)(slot >> 16), (uint8_t)(slot >> 8), (uint8_t)slot};
  uint8_t full[32];
  mbedtls_md_context_t ctx;
  mbedtls_md_init(&ctx);
  mbedtls_md_setup(&ctx, mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), 1);
  mbedtls_md_hmac_starts(&ctx, PAIR_KEY, sizeof(PAIR_KEY));
  mbedtls_md_hmac_update(&ctx, msg, sizeof(msg));
  mbedtls_md_hmac_finish(&ctx, full);
  mbedtls_md_free(&ctx);
  memcpy(out, full, 8);
}

inline uint32_t local_slot() { return millis() / 1000 / ROTATE_S; }

// Tracks the slot offset of a paired peer whose clock we do not share:
// the first match is found by searching SLOT_SEARCH slots, afterwards only
// the expected slot +/- 1 is checked.
struct SlotTracker {
  bool locked = false;
  int32_t offset = 0;  // peer_slot - local_slot
  uint8_t misses[16][8];  // IDs already searched without a match (other families' tags)
  int nMiss = 0, missIdx = 0;

  bool match(uint8_t domain, const uint8_t id[8]) {
    uint8_t cand[8];
    int32_t base = (int32_t)local_slot();
    if (locked) {
      for (int d = -1; d <= 1; d++) {
        int32_t s = base + offset + d;
        if (s < 0) continue;
        hmac8(domain, s, cand);
        if (memcmp(cand, id, 8) == 0) { offset += d; return true; }
      }
    }
    for (int i = 0; i < nMiss; i++)
      if (memcmp(misses[i], id, 8) == 0) return false;   // skip the expensive search
    for (uint32_t s = 0; s < SLOT_SEARCH; s++) {
      hmac8(domain, s, cand);
      if (memcmp(cand, id, 8) == 0) { locked = true; offset = (int32_t)s - base; return true; }
    }
    memcpy(misses[missIdx], id, 8);
    missIdx = (missIdx + 1) % 16;
    if (nMiss < 16) nMiss++;
    return false;
  }
};

// ESP-NOW messages between nodes (the crowd-alert message, RouteMsg, is in pk_qs.h)
struct __attribute__((packed)) SightingMsg {
  uint8_t magic;   // 'S'
  uint8_t node;    // node that heard the tag
  uint8_t id[8];   // opaque rotating tag ID (only the guardian key can link it)
  int8_t rssi;
  uint8_t flags;
};

struct __attribute__((packed)) RhoMsg {  // gateway relays simulated sensor readings
  uint8_t magic;   // 'R'
  uint8_t node;
  float rho;
};

// Last-seen search: the desk computes the case's tag IDs for the last 60 min and
// asks every node for matches in its local sighting buffer (LoRa in deployment).
struct __attribute__((packed)) SearchMsg {
  uint8_t magic;   // 'F'
  uint8_t n;
  uint8_t ids[8][8];
};

struct __attribute__((packed)) HitMsg {
  uint8_t magic;   // 'H'
  uint8_t node;
  uint8_t id[8];
  uint16_t age_s;  // how long ago the node heard it
  int8_t rssi;
};

// Family-device watch: hashes of companion addresses carried by a flagged tag
struct __attribute__((packed)) WatchMsg {
  uint8_t magic;   // 'W'
  uint8_t node;
  uint8_t n;
  uint8_t hashes[2][4];
};

struct __attribute__((packed)) FamilyMsg {
  uint8_t magic;   // 'M'
  uint8_t node;    // node that heard a watched family device
  uint8_t hash[4];
  uint16_t age_s;
  int8_t rssi;
};

// Operator released the fail-safe hold for a silent node after checking it on site
struct __attribute__((packed)) ClearDownMsg {
  uint8_t magic;   // 'D'
  uint16_t node;   // the silent node; 0xFFFF = all
};

// Desk command relayed to nodes: advertise a 'PC' beacon for a tag ID (LOST on/off)
struct __attribute__((packed)) CmdMsg {
  uint8_t magic;   // 'C'
  uint8_t id[8];
  uint8_t cmd;     // FLAG_LOST to set, 0 to clear
};
