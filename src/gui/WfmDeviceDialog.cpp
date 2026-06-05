#include "WfmDeviceDialog.h"

#include <QCheckBox>
#include <QDialogButtonBox>
#include <QLabel>
#include <QListWidget>
#include <QPushButton>
#include <QVBoxLayout>

#ifdef Q_OS_WIN
#define WIN32_LEAN_AND_MEAN
#define NOMINMAX
#include <windows.h>
#include <mmsystem.h>
#endif

namespace AetherSDR {

// Keywords that identify virtual audio cable (VAC) devices.
// A device is shown if its name contains ANY of these (case-insensitive).
static const QStringList kVacKeywords = {
    "cable",        // VB-Audio Virtual Cable, VB-Audio Hi-Fi Cable, Virtual Audio Cable
    "virtual",      // Virtual Audio Cable (Muzychenko), various others
    "vb-audio",     // VB-Audio branding
    "voicemeeter",  // VB-Audio Voicemeeter family
    "dante",        // Audinate Dante Virtual Soundcard
    "loopback",     // Loopback (Rogue Amoeba), others
    "jack",         // JACK Audio Connection Kit
    "blackhole",    // BlackHole (macOS)
    "soundflower",  // Soundflower (macOS)
};

static bool isVacDevice(const QString& name)
{
    const QString lower = name.toLower();
    for (const QString& kw : kVacKeywords) {
        if (lower.contains(kw))
            return true;
    }
    return false;
}

WfmDeviceDialog::WfmDeviceDialog(QWidget* parent)
    : QDialog(parent)
{
    setWindowTitle(tr("WFM Audio Output Device"));
    setMinimumWidth(400);

    auto* vb = new QVBoxLayout(this);
    vb->setSpacing(10);
    vb->setContentsMargins(16, 16, 16, 12);

    auto* lbl = new QLabel(
        tr("Select the virtual audio cable (VAC) to receive the WFM demodulated audio.\n"
           "Install VB-Audio Virtual Cable or similar if no device appears below."),
        this);
    lbl->setWordWrap(true);
    vb->addWidget(lbl);

    m_list = new QListWidget(this);
    m_list->setAlternatingRowColors(true);

    const QStringList devices = enumerateDevices();
    for (const QString& name : devices)
        m_list->addItem(name);

    if (m_list->count() == 0) {
        auto* noDevLbl = new QLabel(
            tr("<i>No virtual audio cable found.<br>"
               "Install <b>VB-Audio Virtual Cable</b> and restart AetherSDR.</i>"),
            this);
        noDevLbl->setWordWrap(true);
        vb->addWidget(noDevLbl);
    } else {
        m_list->setCurrentRow(0);
    }

    vb->addWidget(m_list);

    m_remember = new QCheckBox(tr("Remember this choice"), this);
    m_remember->setChecked(true);
    vb->addWidget(m_remember);

    auto* btns = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);
    btns->button(QDialogButtonBox::Ok)->setEnabled(m_list->count() > 0);
    connect(btns, &QDialogButtonBox::accepted, this, &QDialog::accept);
    connect(btns, &QDialogButtonBox::rejected, this, &QDialog::reject);
    connect(m_list, &QListWidget::itemSelectionChanged, this, [btns, this]() {
        btns->button(QDialogButtonBox::Ok)->setEnabled(m_list->currentItem() != nullptr);
    });
    vb->addWidget(btns);

    // Double-click confirms
    connect(m_list, &QListWidget::itemDoubleClicked, this, &QDialog::accept);
}

QString WfmDeviceDialog::selectedDevice() const
{
    auto* item = m_list->currentItem();
    return item ? item->text() : QString();
}

bool WfmDeviceDialog::rememberChoice() const
{
    return m_remember->isChecked();
}

// static
QStringList WfmDeviceDialog::enumerateDevices()
{
    QStringList devices;

#ifdef Q_OS_WIN
    const UINT count = waveOutGetNumDevs();
    for (UINT i = 0; i < count; ++i) {
        WAVEOUTCAPSW caps{};
        if (waveOutGetDevCapsW(i, &caps, sizeof(caps)) == MMSYSERR_NOERROR) {
            const QString name = QString::fromWCharArray(caps.szPname);
            if (isVacDevice(name))
                devices << name;
        }
    }
#endif

    return devices;
}

} // namespace AetherSDR
