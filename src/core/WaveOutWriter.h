#pragma once
#include <QAudioDevice>
#include <QAudioFormat>
#include <QAudioSink>
#include <QByteArray>
#include <QIODevice>
#include <QObject>
#include <QString>

namespace AetherSDR {

// Cross-platform audio output wrapper backed by QAudioSink (Qt6 Multimedia).
// Accepts Int16 stereo PCM at a fixed sample rate and writes it to the
// selected output device — works on Windows (WASAPI), macOS (CoreAudio),
// and Linux (PipeWire / PulseAudio / ALSA).
class WaveOutWriter : public QObject
{
    Q_OBJECT
public:
    explicit WaveOutWriter(QObject* parent = nullptr);
    ~WaveOutWriter() override;

    // Open the device whose description contains |deviceId| (the
    // QAudioDevice::id() string stored in WfmSettings).  Returns true on
    // success.  |sampleRate| is the output rate in Hz (typically 48000).
    bool open(const QString& deviceId, int sampleRate, int channelCount = 2);

    void close();

    // Write Int16 interleaved PCM samples.  Thread-safe: may be called from
    // any thread; internally posts to the Qt event loop.
    void write(const QByteArray& pcm);

    bool isOpen() const { return m_sink != nullptr; }
    QString deviceName() const { return m_deviceName; }

private:
    QAudioSink*  m_sink{nullptr};
    QIODevice*   m_io{nullptr};
    QString      m_deviceName;
};

} // namespace AetherSDR
