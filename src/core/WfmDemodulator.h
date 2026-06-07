#pragma once
#include <QAudioSource>
#include <QIODevice>
#include <QObject>
#include <QVector>
#include <QByteArray>
#include <algorithm>

namespace AetherSDR {

class DaxIqModel;
class WaveOutWriter;

// Receives raw PCM from QAudioSource and forwards it to WfmDemodulator.
class DaxIqCaptureDevice : public QIODevice
{
    Q_OBJECT
public:
    explicit DaxIqCaptureDevice(QObject* parent = nullptr) : QIODevice(parent) {}
    bool isSequential() const override { return true; }
    qint64 readData(char*, qint64) override { return 0; }
    qint64 writeData(const char* data, qint64 len) override {
        emit pcmReady(QByteArray(data, static_cast<int>(len)));
        return len;
    }
signals:
    void pcmReady(const QByteArray& pcm);
};

// Software FM demodulator for satellite work (G3RUH 9600 baud, hs-soundmodem).
//
// IQ source (primary, Windows):
//   QAudioSource reading from the SmartSDR DAX IQ capture device
//   ("DAX IQ RX 1"). DAX delivers IQ as stereo PCM: L=I, R=Q.
//
// IQ source (fallback, Linux/macOS):
//   VITA-49 DaxIqModel::iqSamplesReady signal.
//
// Audio output: QAudioSink → HiFi Cable / VAC at 48 kHz stereo Float32.
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

    void start(DaxIqModel* daxIq, const QString& deviceId,
               const QString& panId = QString(), float freqOffsetHz = 0.0f);
    void stop();

    bool isActive() const { return m_active; }

    void setVolume(int pct) { m_volume = std::clamp(pct / 100.0f, 0.0f, 1.0f); }

signals:
    void commandReady(const QString& cmd);

public slots:
    // VITA-49 fallback path
    void onIqSamples(int channel, QVector<float> iqInterleaved, int sampleRate);
    void onStreamChanged(int channel);

private slots:
    // DAX audio capture path (primary on Windows)
    void onDaxIqPcm(const QByteArray& pcm);

private:
    void processIqFloat(const QVector<float>& iqInterleaved);
    void processIqInt16(const QByteArray& pcm);

    DaxIqModel*         m_daxIq{nullptr};
    WaveOutWriter*      m_waveOut{nullptr};
    QAudioSource*       m_iqSource{nullptr};
    DaxIqCaptureDevice* m_iqDevice{nullptr};
    bool                m_usingDaxCapture{false};
    bool                m_active{false};
    float               m_volume{1.0f};

    // Frequency-correction oscillator
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
