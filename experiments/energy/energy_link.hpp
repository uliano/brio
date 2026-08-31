// energy_link.hpp - the wire contract of the energy experiment: what
// crosses the two board-to-board wires, in one file both firmwares
// include so the two halves cannot drift apart (the usart_link/
// spi_link/twi_link precedent, here crossing ARCHITECTURES for the
// first time).
//
// Both wires are ANALOG and the contract speaks ABSOLUTE VOLTS on
// internal references at both ends - never logic levels, never
// ratiometric: the DUT's supply is 3.0..4.2 V while the meter runs at
// ~5 V, and the C21's digital VIH (0.7 x VDD, DS60001479M ch. 45) puts
// a 3.3 V witness out of spec on any digital input. So:
//
//   stimulus  meter DAC (INTREF 1.024 V) -> DUT ADC pin, 0..1 V,
//             detected by the DUT against ITS internal 2.048 V
//             reference (a VDD-referred threshold would slide with the
//             capacitor discharge);
//   witness   DUT pin (0/VDD push-pull) -> meter AC comparator against
//             the internal 1.024 V bandgap, hysteresis on, so the same
//             edge decodes at every DUT supply in the window.
//
// TWO AUDIENCES. The witness vocabulary below is shared by both sides.
// The seeded schedule is included by the METER ONLY: the DUT must stay
// blind to the future - its detection is the experiment, not a replay.
// run.py reimplements xorshift32 in Python (the uart_stress precedent)
// and every run verifies the two generators against each other via the
// golden vector the meter prints at start.
//
// Status: v0 - witness vocabulary and level contract. The schedule
// section arrives with the meter's stimulus letter.

#pragma once

#include <stdint.h>

namespace energy {

// ---- level contract (absolute mV) ------------------------------------------

// Stimulus levels out of the meter's DAC (its INTREF at 1.024 V; codes
// are dac_code(mv, 1024) on the meter side). The DUT detects against
// its own 2.048 V ADC reference: threshold halfway between the two
// levels so both margins survive every supply in the 3.0..4.2 V window.
inline constexpr uint16_t stimulus_quiet_mv = 200;
inline constexpr uint16_t stimulus_burst_mv = 800;
inline constexpr uint16_t detect_threshold_mv = 500;

// The witness is received by the meter's AC against the 1.024 V
// bandgap; the DUT drives rail-to-rail at its own supply. Margins at
// the worst corner (DUT at 3.0 V): high ~2 V, low ~1 V.
inline constexpr uint16_t witness_threshold_mv = 1024;

// ---- witness vocabulary -----------------------------------------------------

// One toggle = one burst processed. Signatures are counted fast pulses
// (a pulse = high for signature_half_us, low for signature_half_us),
// separated from data toggles and from each other by at least
// signature_gap_us of silence. Counts are odd and distinct from 1 so a
// lone data toggle can never alias a signature.
// Sized for a millisecond-resolution decoder: a signature's 2N edges
// land inside ~N x 0.4 ms, and any two witness events (signature or
// data toggle) are separated by at least signature_gap_us of silence -
// so "edges closer than 2 ms" groups a signature and a group of ONE
// edge is a data toggle.
inline constexpr uint8_t sig_park_enter = 3;  // entering PD parking
inline constexpr uint8_t sig_park_leave = 5;  // leaving PD parking
inline constexpr uint16_t signature_half_us = 200;
inline constexpr uint16_t signature_gap_us = 5000;

// ---- the schedule generator -------------------------------------------------

// xorshift32 (Marsaglia), the same generator tools/uart_stress.py
// established as the firmware/Python shared PRNG shape. Seed must be
// nonzero. The burst-schedule derivation on top of this (arrival times,
// M, W per point) is the meter's and run.py's alone - deliberately NOT
// available to the DUT half (see the header comment).
inline constexpr uint32_t xorshift32(uint32_t x) {
    x ^= x << 13;
    x ^= x >> 17;
    x ^= x << 5;
    return x;
}

}  // namespace energy
