#pragma once

#include <QObject>
#include <QByteArray>
#include <QString>
#include <QMutex>
#include <windows.h>
#include <mmsystem.h>

namespace AetherSDR {

// Writes PCM audio to a Windows waveOut device using a dedicated refill thread.
// When each buffer finishes playing, the thread immediately re-queues it with
// real audio (from the ring) or silence — the queue is NEVER empty.
//
// sampleRate   : 48000
// channels     : 2 (stereo)
// bitsPerSample: 16 (Int16)
class WaveOutWriter : public QObject {
    Q_OBJECT

public:
    explicit WaveOutWriter(QObject* parent = nullptr);
    ~WaveOutWriter() override;

    bool open(const QString& deviceNameFragment, int sampleRate = 48000,
              int channels = 2, int bitsPerSample = 16);
    void close();
    bool isOpen() const { return m_hWaveOut != nullptr; }

    // Push PCM bytes from the IQ processing thread (thread-safe).
    void write(const QByteArray& pcm);

    QString deviceName() const { return m_deviceName; }

    // Called by the refill thread — do not call directly.
    void refillDoneBuffers();

private:
    static constexpr int NUM_BUFS    = 8;    // rotating buffers
    static constexpr int BUF_SAMPLES = 512;  // 10 ms at 48 kHz

    static DWORD WINAPI refillThreadProc(LPVOID param);

    HWAVEOUT   m_hWaveOut{nullptr};
    HANDLE     m_refillEvent{nullptr};
    HANDLE     m_refillThread{nullptr};
    volatile bool m_running{false};

    WAVEHDR    m_headers[NUM_BUFS]{};
    QByteArray m_buffers[NUM_BUFS];
    int        m_bytesPerBuf{0};

    QByteArray m_pending;
    QMutex     m_mutex;
    QString    m_deviceName;
};

} // namespace AetherSDR
