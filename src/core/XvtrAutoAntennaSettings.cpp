#include "XvtrAutoAntennaSettings.h"

#include "AppSettings.h"

#include <QJsonDocument>
#include <QJsonObject>

namespace AetherSDR {

namespace {

constexpr const char* kSettingsKey = "XvtrAutoAntenna";

QJsonObject loadRoot()
{
    const QString raw =
        AppSettings::instance().value(kSettingsKey, "").toString();
    if (raw.isEmpty()) {
        return {};
    }
    const QJsonDocument doc = QJsonDocument::fromJson(raw.toUtf8());
    return doc.isObject() ? doc.object() : QJsonObject{};
}

} // namespace

XvtrAutoAntennaPorts loadXvtrAutoAntennaPorts(int xvtrIndex)
{
    const QJsonObject entry =
        loadRoot().value(QString::number(xvtrIndex)).toObject();
    return {entry.value("rx").toString(), entry.value("tx").toString()};
}

void saveXvtrAutoAntennaPorts(int xvtrIndex, const XvtrAutoAntennaPorts& ports)
{
    QJsonObject root = loadRoot();
    const QString key = QString::number(xvtrIndex);
    if (!ports.isConfigured()) {
        root.remove(key);
    } else {
        QJsonObject entry;
        if (!ports.rx.isEmpty()) {
            entry.insert("rx", ports.rx);
        }
        if (!ports.tx.isEmpty()) {
            entry.insert("tx", ports.tx);
        }
        root.insert(key, entry);
    }

    auto& s = AppSettings::instance();
    if (root.isEmpty()) {
        s.remove(kSettingsKey);
    } else {
        s.setValue(kSettingsKey, QString::fromUtf8(
            QJsonDocument(root).toJson(QJsonDocument::Compact)));
    }
    s.save();
}

} // namespace AetherSDR
