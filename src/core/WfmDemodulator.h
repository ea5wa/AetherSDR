#pragma once
#include <QObject>
#include <QVector>
#include <QByteArray>
#include <algorithm>

namespace AetherSDR {

class DaxIqModel;
class WaveOutWriter;

// Software FM demodulator for satellite work (G3RUH 9600 baud, hs-soundmodem).
// Uses a phase-difference (atan2) discriminator.
//
// IQ source     : VITA-49 DaxIqModel::iqSamplesReady — cross-platform,
//                 no Win32 dependency.
// IF bandwidth  : ±20 kHz slice filter
// Audio output  : QAudioSink → any system output device (VAC, BlackHole,
//                 PipeWire null-sink, etc.) at 48 kHz stereo Int16.
// Gain          : G3RUH ±4.8 kHz deviation → ~60 % full scale
class WfmDemodulator : public QObject
{
    Q_OBJECT
public:
    static constexpr int   DAX_CHANNEL = 1;
    static constexpr int   IQ_RATE     = 48000;
    static constexpr int   AUDIO_RATE  = 48000;
    static constexpr int   FILTER_HZ   = 20000;
    static constexpr float GAIN        = 1.0f;

    explicit WfmDemodulator(QObject* parent = nullptr);
    ~WfmDemodulator() override;

    // deviceId    : QAudioDevice::id() string stored in WfmSettings.
    // panId       : panadapter ID string ("0x40000000")
    // freqOffsetHz: slice_frequency_hz − panadapter_center_hz
    //               Applied as a complex frequency shift so the IQ is
    //               centered at the satellite frequency before FM discrimination.
    void start(DaxIqModel* daxIq, const QString& deviceId,
               const QString& panId = QString(), float freqOffsetHz = 0.0f);
    void stop();

    bool isActive() const { return m_active; }

    // Volume 0–100 (maps linearly to 0.0–1.0 amplitude scale).
    void setVolume(int pct) { m_volume = std::clamp(pct / 100.0f, 0.0f, 1.0f); }

signals:
    void commandReady(const QString& cmd);

public slots:
    // Called from DaxIqModel VITA-49 path
    void onIqSamples(int channel, QVector<float> iqInterleaved, int sampleRate);
    void onStreamChanged(int channel);

private:
    void processSamples(const QVector<float>& iqInterleaved);

    DaxIqModel*    m_daxIq{nullptr};
    WaveOutWriter* m_waveOut{nullptr};
    bool           m_active{false};
    float          m_volume{1.0f};

    // Frequency-correction oscillator (centers IQ at slice frequency)
    float m_corrCos{1.0f};
    float m_corrSin{0.0f};
    float m_corrCosStep{1.0f};
    float m_corrSinStep{0.0f};
    float m_prevI{0.0f};
    float m_prevQ{0.0f};

    QString m_panId;
    bool    m_panSent{false};
};

} // namespace AetherSDR
