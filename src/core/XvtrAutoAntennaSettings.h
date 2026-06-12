#pragma once

#include <QString>

namespace AetherSDR {

// Client-side per-transverter antenna auto-switch configuration (#3531).
// The radio does not persist per-XVTR antenna mappings, so they live in
// AppSettings as one nested JSON blob under the "XvtrAutoAntenna" root key
// (Principle V), keyed by the xvtr status-object index:
//
//   {"12": {"rx": "XVTA", "tx": "XVTA"}, "4": {"rx": "XVTB"}}
//
// An empty port means "leave the radio's band-stack antenna untouched",
// which is the default for every transverter.

struct XvtrAutoAntennaPorts {
    QString rx;
    QString tx;

    bool isConfigured() const { return !rx.isEmpty() || !tx.isEmpty(); }
};

XvtrAutoAntennaPorts loadXvtrAutoAntennaPorts(int xvtrIndex);
void saveXvtrAutoAntennaPorts(int xvtrIndex, const XvtrAutoAntennaPorts& ports);

} // namespace AetherSDR
