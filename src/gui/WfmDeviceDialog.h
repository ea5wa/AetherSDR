#pragma once

#include <QDialog>
#include <QStringList>

class QListWidget;
class QCheckBox;
class QLabel;

namespace AetherSDR {

// Dialog shown on the first WFM activation (or when no device is saved).
// Enumerates available waveOut audio devices and lets the user pick one.
// The selection can be persisted via the "Remember this choice" checkbox
// so the dialog does not appear again on subsequent activations.
class WfmDeviceDialog : public QDialog {
    Q_OBJECT

public:
    explicit WfmDeviceDialog(QWidget* parent = nullptr);

    // Full device name chosen by the user, or empty if cancelled.
    QString selectedDevice() const;

    // True when the user ticked "Remember this choice".
    bool rememberChoice() const;

private:
    static QStringList enumerateDevices();

    QListWidget* m_list{nullptr};
    QCheckBox*   m_remember{nullptr};
};

} // namespace AetherSDR
