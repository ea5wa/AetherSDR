#pragma once

#include <QObject>
#include <QByteArray>
#include <QString>
#include <QMutex>
#include <windows.h>
#include <mmsystem.h>

namespace AetherSDR {

// Reads PCM audio from a Windows waveIn (recording) device using a
// dedicated capture thread.  Intended use: SmartSDR DAX IQ devices
// (e.g. "DAX IQ RX 1") which deliver stereo audio where L=I and R=Q.
class WaveInReader : public QObject {
    Q_OBJECT

public:
    explicit WaveInReader(QObject* parent = nullptr);
    ~WaveInReader() override;

    bool open(const QString& deviceNameFragment, int sampleRate = 48000,
              int channels = 2, int bitsPerSample = 16);
    void close();
    bool isOpen() const { return m_hWaveIn != nullptr; }

    QString deviceName() const { return m_deviceName; }

    void captureLoop();  // called by capture thread — do not call directly

signals:
    // Stereo Int16 PCM: for IQ streams, L channel = I, R channel = Q.
    void pcmReady(const QByteArray& pcm);

private:
    static constexpr int NUM_BUFS    = 8;
    static constexpr int BUF_SAMPLES = 512;

    static DWORD WINAPI captureThreadProc(LPVOID param);

    HWAVEIN    m_hWaveIn{nullptr};
    HANDLE     m_captureEvent{nullptr};
    HANDLE     m_captureThread{nullptr};
    volatile bool m_running{false};

    WAVEHDR    m_headers[NUM_BUFS]{};
    QByteArray m_buffers[NUM_BUFS];
    int        m_bytesPerBuf{0};
    QString    m_deviceName;
};

} // namespace AetherSDR
