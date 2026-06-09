#pragma once
#include <QAudioSource>
#include <QIODevice>
#include <QObject>
#include <QVector>
#include <QByteArray>
#include <algorithm>
#include <array>

namespace AetherSDR {

class DaxIqModel;
class WaveOutWriter;

// ---------------------------------------------------------------------------
// DaxIqCaptureDevice — QIODevice sink that forwards DAX PCM blocks
// ---------------------------------------------------------------------------
class DaxIqCaptureDevice : public QIODevice
{
    Q_OBJECT
public:
    explicit DaxIqCaptureDevice(QObject* parent = nullptr) : QIODevice(parent) {}
    bool   isSequential() const override { return true; }
    qint64 readData(char*, qint64)       override { return 0; }
    qint64 writeData(const char* data, qint64 len) override {
        emit pcmReady(QByteArray(data, static_cast<int>(len)));
        return len;
    }
signals:
    void pcmReady(const QByteArray& pcm);
};

// ---------------------------------------------------------------------------
// WfmDemodulator
//
// Signal chain
//   1. IQ (Int16 stereo from DAX, or float32 from VITA-49)
//   2. Frequency-correction phasor (Doppler)
//   3. Phase-difference FM discriminator: +atan2(I·Qp − Q·Ip, I·Ip + Q·Qp) / π
//      — atan2 is amplitude-invariant; no IQ normalisation needed
//   4. FIR low-pass, order 96 (97 taps), fc = 20 kHz @ 48 kHz, Hamming window
//      — symmetric (h[n] = h[N−1−n]), linear phase, sum ≈ 1, no DC notch
//   5. Volume scale → Float32 stereo → QAudioSink (HiFi Cable / VAC)
// ---------------------------------------------------------------------------
class WfmDemodulator : public QObject
{
    Q_OBJECT
public:
    static constexpr int   DAX_CHANNEL = 1;
    static constexpr int   IQ_RATE     = 48000;
    static constexpr int   AUDIO_RATE  = 48000;
    // FIR low-pass design parameters
    //   fs       = 48 000 Hz
    //   fc       = 20 000 Hz  (midpoint of transition band 18–22 kHz)
    //   order    = 94  →  95 taps  (Type-I symmetric, h[n]=h[N-1-n])
    //   window   = Hamming  (stopband ≥ 53 dB, stopband edge ≈ 22 kHz)
    static constexpr int   LP_CUTOFF   = 20000;  // FIR fc, Hz
    static constexpr int   FILTER_HZ   = 20000;  // alias used by MainWindow
    static constexpr float GAIN        = 3.0f;   // G3RUH peak ≈ ±0.2×3 = ±0.6 → no clip

    static constexpr int kFirOrder = 94;          // even order → odd taps → Type-I
    static constexpr int kFirTaps  = kFirOrder + 1;  // 95

    explicit WfmDemodulator(QObject* parent = nullptr);
    ~WfmDemodulator() override;

    void start(DaxIqModel* daxIq, const QString& deviceId,
               const QString& panId = QString(), float freqOffsetHz = 0.0f);
    void stop();

    bool isActive() const { return m_active; }
    void setVolume(int pct) { m_volume = std::clamp(pct / 100.0f, 0.0f, 1.0f); }

    // Real-time Doppler correction: offsetHz = sliceHz − panCenterHz
    void setFreqOffsetHz(float offsetHz) {
        if (!m_active) return;
        const float step = -2.0f * static_cast<float>(M_PI) * offsetHz / m_actualIqRate;
        m_corrCosStep = std::cos(step);
        m_corrSinStep = std::sin(step);
    }

signals:
    void commandReady(const QString& cmd);

public slots:
    void onIqSamples(int channel, QVector<float> iqInterleaved, int sampleRate);
    void onStreamChanged(int channel);

private slots:
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

    // Doppler correction oscillator
    float m_corrCos{1.0f};
    float m_corrSin{0.0f};
    float m_corrCosStep{1.0f};
    float m_corrSinStep{0.0f};

    // FM discriminator state
    float m_prevI{0.0f};
    float m_prevQ{0.0f};

    // FM noise parabola compensator — two cascaded 1st-order IIR LPF
    // H(f)² ∝ 1/f² above fc ≈ 1.2 kHz → cancels f² FM noise rise
    // α = exp(−2π × 1200/48000) ≈ 0.85
    float m_deemph1{0.0f};
    float m_deemph2{0.0f};

    // FIR delay line (circular buffer)
    std::array<float, kFirTaps> m_firBuf{};
    int m_firIdx{0};

    QString m_panId;
    bool    m_panSent{false};
    int     m_actualIqRate{IQ_RATE};
};

} // namespace AetherSDR
