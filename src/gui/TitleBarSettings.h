#pragma once

#include "core/AppSettings.h"

#include <QJsonDocument>
#include <QJsonObject>
#include <QString>

namespace AetherSDR {

// Persistence helper for title-bar UI toggles (#3408 PanLock is the first;
// future title-bar toggles land here as additional fields).
//
// Stored as a nested JSON blob under AppSettings["TitleBar"], per the
// nested-JSON-per-feature convention (constitution Principle V).  The legacy
// flat key "PanLockEnabled" is migrated into this blob on first read so
// existing users keep their behavior.
class TitleBarSettings {
public:
    static bool panLockEnabled()
    {
        return readObj().value("panLockEnabled").toString("False") == "True";
    }

    static void setPanLockEnabled(bool on)
    {
        QJsonObject o = readObj();
        o["panLockEnabled"] = on ? QStringLiteral("True") : QStringLiteral("False");
        write(o);
    }

    // One-shot migration from the legacy "PanLockEnabled" flat key.  Run at app
    // startup (or TitleBar construction) before any caller reads the new blob.
    // Safe to call repeatedly: returns immediately if the new blob already
    // exists.
    static void migrateLegacy()
    {
        auto& s = AppSettings::instance();
        if (s.contains("TitleBar")) return;
        const bool legacyPanLock =
            s.value("PanLockEnabled", "False").toString() == "True";
        QJsonObject o;
        o["panLockEnabled"] =
            legacyPanLock ? QStringLiteral("True") : QStringLiteral("False");
        write(o);
        // Leave the legacy flat key in place — harmless after migration, and a
        // future cleanup PR can drop it once we're confident no other reader
        // still touches it.
    }

private:
    static QJsonObject readObj()
    {
        const QString json =
            AppSettings::instance().value("TitleBar", QString{}).toString();
        if (json.isEmpty()) return {};
        return QJsonDocument::fromJson(json.toUtf8()).object();
    }
    static void write(const QJsonObject& o)
    {
        auto& s = AppSettings::instance();
        s.setValue("TitleBar",
                   QString::fromUtf8(
                       QJsonDocument(o).toJson(QJsonDocument::Compact)));
        s.save();
    }
};

} // namespace AetherSDR
