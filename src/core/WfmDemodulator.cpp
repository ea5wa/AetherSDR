#include "core/WfmDemodulator.h"
#include "core/WaveOutWriter.h"
#include "core/LogManager.h"
#include "models/DaxIqModel.h"

#include <QMediaDevices>
#include <cmath>
#include <algorithm>

namespace AetherSDR {

// ---------------------------------------------------------------------------
// FIR low-pass coefficients  — buildFirLP()
//
// Specification
//   fs            = 48 000 Hz
//   fc            = 20 000 Hz  (midpoint of 18 kHz passband / 22 kHz stopband)
//   order         = 94  →  95 taps  (kFirTaps = kFirOrder + 1)
//   window        = Hamming  → stopband attenuation ≥ 53 dB, stopband edge ≈ 22 kHz
//   phase         = linear (Type-I: odd taps, symmetric h[n] = h[N-1-n])
//   DC gain       = 1.0  (Σ h[n] normalised → no DC notch, no high-pass artifact)
//
// Formula
//   t    = n − M/2        (time index centred at M/2, where M = kFirOrder = 94)
//   sinc = sin(π·2·fc·t) / (π·2·fc·t),   sinc(0) = 1
//   win  = 0.54 − 0.46·cos(2π·n / M)     (Hamming window)
//   h[n] = sinc · win
//   normalise so Σ h[n] = 1
//
// Symmetry check: t(n) = −t(N−1−n) → sinc and win are both symmetric around n=M/2
//   ⟹ h[n] = h[N−1−n]  ✓
// ---------------------------------------------------------------------------
static std::array<float, WfmDemodulator::kFirTaps> buildFirLP()
{
    constexpr int   M  = WfmDemodulator::kFirOrder;   // 94
    constexpr float fc = static_cast<float>(WfmDemodulator::LP_CUTOFF)
                       / static_cast<float>(WfmDemodulator::IQ_RATE); // 20000/48000
    const float pi = static_cast<float>(M_PI);

    std::array<float, WfmDemodulator::kFirTaps> h{};
    float sum = 0.0f;

    for (int n = 0; n <= M; ++n) {
        const float t = static_cast<float>(n) - static_cast<float>(M) * 0.5f;

        // Ideal low-pass impulse response (sinc)
        float sinc;
        if (std::abs(t) < 1e-7f) {
            sinc = 1.0f;
        } else {
            const float x = 2.0f * fc * pi * t;   // π · 2fc · t
            sinc = std::sin(x) / x;
        }

        // Hamming window: w[n] = 0.54 − 0.46·cos(2π·n/M)
        const float win = 0.54f
                        - 0.46f * std::cos(2.0f * pi * static_cast<float>(n)
                                                      / static_cast<float>(M));

        h[n] = sinc * win;
        sum += h[n];
    }

    // Normalise to unity DC gain: Σ h[n] = 1  →  no DC notch
    for (float& v : h) v /= sum;

    return h;
}

// Computed once at program start; const so it lives in .rodata
static const std::array<float, WfmDemodulator::kFirTaps> kFirLP = buildFirLP();

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static QAudioDevice findCaptureDevice(const QStringList& hints)
{
    const auto inputs = QMediaDevices::audioInputs();
    for (const QAudioDevice& dev : inputs) {
        const QString desc = dev.description().toLower();
        for (const QString& h : hints)
            if (desc.contains(h.toLower())) return dev;
    }
    return {};
}

// ---------------------------------------------------------------------------
// WfmDemodulator — lifecycle
// ---------------------------------------------------------------------------

WfmDemodulator::WfmDemodulator(QObject* parent) : QObject(parent) {}

WfmDemodulator::~WfmDemodulator() { stop(); }

void WfmDemodulator::start(DaxIqModel* daxIq, const QString& deviceId,
                           const QString& panId, float freqOffsetHz)
{
    if (m_active) stop();

    m_daxIq           = daxIq;
    m_prevI           = 0.0f;
    m_prevQ           = 0.0f;
    m_corrCos         = 1.0f;
    m_corrSin         = 0.0f;
    m_panId           = panId;
    m_panSent         = false;
    m_usingDaxCapture = false;
    m_firBuf.fill(0.0f);
    m_firIdx  = 0;
    m_deemph1 = 0.0f;
    m_deemph2 = 0.0f;
    m_eqX1 = m_eqX2 = 0.0f;
    m_eqY1 = m_eqY2 = 0.0f;

    qCDebug(lcAudio) << "WfmDemodulator::start device=" << deviceId
                     << "freqOffset=" << freqOffsetHz;

    // --- Primary path: SmartSDR DAX IQ capture device (Windows) ---
    const QStringList daxHints = { "dax iq rx 1", "dax iq 1",
                                   "dax reserved iq rx 1", "dax iq" };
    QAudioDevice capDev = findCaptureDevice(daxHints);

    if (!capDev.isNull()) {
        QAudioFormat fmt;
        fmt.setSampleRate(IQ_RATE);
        fmt.setChannelCount(2);
        fmt.setSampleFormat(QAudioFormat::Int16);
        if (!capDev.isFormatSupported(fmt))
            fmt = capDev.preferredFormat();

        const int rate = fmt.sampleRate();
        m_actualIqRate = rate;

        const float step = -2.0f * static_cast<float>(M_PI) * freqOffsetHz / rate;
        m_corrCosStep = std::cos(step);
        m_corrSinStep = std::sin(step);

        qCInfo(lcAudio) << "WfmDemodulator: DAX device=" << capDev.description()
                        << "rate=" << rate
                        << "ch=" << fmt.channelCount()
                        << "fmt=" << fmt.sampleFormat();

        m_waveOut = new WaveOutWriter(this);
        if (!m_waveOut->open(deviceId, rate, 2)) {
            qCWarning(lcAudio) << "WfmDemodulator: cannot open audio output" << deviceId;
            delete m_waveOut; m_waveOut = nullptr;
            return;
        }

        m_iqDevice = new DaxIqCaptureDevice(this);
        m_iqDevice->open(QIODevice::WriteOnly);
        connect(m_iqDevice, &DaxIqCaptureDevice::pcmReady,
                this,       &WfmDemodulator::onDaxIqPcm);

        m_iqSource = new QAudioSource(capDev, fmt, this);
        m_iqSource->start(m_iqDevice);

        if (m_iqSource->error() == QAudio::NoError) {
            m_usingDaxCapture = true;
            qCInfo(lcAudio) << "WfmDemodulator: DAX capture active at" << rate << "Hz";
        } else {
            qCWarning(lcAudio) << "WfmDemodulator: DAX capture failed, error="
                               << m_iqSource->error() << "→ fallback VITA-49";
            m_iqSource->stop();
            delete m_iqSource;  m_iqSource = nullptr;
            delete m_iqDevice;  m_iqDevice = nullptr;
        }
    } else {
        qCDebug(lcAudio) << "WfmDemodulator: DAX device not found";
    }

    // --- Fallback: VITA-49 DaxIqModel (Linux/macOS, or no DAX) ---
    if (!m_usingDaxCapture) {
        const float step = -2.0f * static_cast<float>(M_PI) * freqOffsetHz / IQ_RATE;
        m_corrCosStep  = std::cos(step);
        m_corrSinStep  = std::sin(step);
        m_actualIqRate = IQ_RATE;

        m_waveOut = new WaveOutWriter(this);
        if (!m_waveOut->open(deviceId, AUDIO_RATE, 2)) {
            qCWarning(lcAudio) << "WfmDemodulator: cannot open audio output" << deviceId;
            delete m_waveOut; m_waveOut = nullptr;
            return;
        }
        connect(m_daxIq, &DaxIqModel::iqSamplesReady, this, &WfmDemodulator::onIqSamples);
        connect(m_daxIq, &DaxIqModel::streamChanged,  this, &WfmDemodulator::onStreamChanged);
        m_daxIq->createStream(DAX_CHANNEL);
        qCDebug(lcAudio) << "WfmDemodulator: VITA-49 fallback, panId=" << m_panId;
    }

    m_active = true;
}

void WfmDemodulator::stop()
{
    if (!m_active) return;
    m_active = false;

    if (m_iqSource) { m_iqSource->stop(); delete m_iqSource; m_iqSource = nullptr; }
    if (m_iqDevice) { m_iqDevice->close(); delete m_iqDevice; m_iqDevice = nullptr; }
    if (m_daxIq) {
        if (!m_usingDaxCapture) m_daxIq->removeStream(DAX_CHANNEL);
        disconnect(m_daxIq, nullptr, this, nullptr);
        m_daxIq = nullptr;
    }
    if (m_waveOut) { m_waveOut->close(); delete m_waveOut; m_waveOut = nullptr; }
}

// ---------------------------------------------------------------------------
// Slot handlers
// ---------------------------------------------------------------------------

void WfmDemodulator::onStreamChanged(int channel)
{
    const auto& s = m_daxIq->stream(DAX_CHANNEL);
    if (channel != DAX_CHANNEL || m_panSent || m_panId.isEmpty()) return;
    if (!s.exists || s.streamId == 0) return;
    const QString cmd = QString("stream set 0x%1 pan=%2")
                        .arg(s.streamId, 0, 16).arg(m_panId);
    qCDebug(lcAudio) << "WfmDemodulator: sending" << cmd;
    emit commandReady(cmd);
    m_panSent = true;
}

void WfmDemodulator::onIqSamples(int channel, QVector<float> iq, int /*rate*/)
{
    if (channel != DAX_CHANNEL || !m_active || !m_waveOut) return;
    processIqFloat(iq);
}

void WfmDemodulator::onDaxIqPcm(const QByteArray& pcm)
{
    if (!m_active || !m_waveOut) return;
    processIqInt16(pcm);
}

// ---------------------------------------------------------------------------
// processIqInt16 — DAX stereo Int16 → float, then processIqFloat
// ---------------------------------------------------------------------------

void WfmDemodulator::processIqInt16(const QByteArray& pcm)
{
    const int n = pcm.size() / (2 * sizeof(qint16));
    if (n <= 0) return;
    const auto* raw = reinterpret_cast<const qint16*>(pcm.constData());
    QVector<float> iq(n * 2);
    for (int i = 0; i < n; ++i) {
        iq[2*i]   = raw[2*i]   / 32768.0f;   // L = I
        iq[2*i+1] = raw[2*i+1] / 32768.0f;   // R = Q
    }
    processIqFloat(iq);
}

// ---------------------------------------------------------------------------
// processIqFloat — core DSP
//
//  Step 1 – Doppler phasor rotation
//  Step 2 – Phase-difference FM discriminator (+atan2)
//            atan2 is amplitude-invariant: no IQ normalisation needed.
//            Output ∈ [−π, π]; divide by π → [−1, 1] (GAIN ≤ 1 = no clip)
//  Step 3 – FIR low-pass, 97 taps, fc = 20 kHz, Hamming, linear phase
//            Symmetric coefficients, sum = 1, NO DC notch, NO de-emphasis
//  Step 4 – Volume scale, write Float32 stereo to ring buffer
// ---------------------------------------------------------------------------

void WfmDemodulator::processIqFloat(const QVector<float>& iq)
{
    const int numSamples = iq.size() / 2;
    if (numSamples <= 0) return;

    QByteArray pcm(numSamples * 2 * sizeof(float), Qt::Uninitialized);
    auto* out = reinterpret_cast<float*>(pcm.data());

    float prevI = m_prevI;
    float prevQ = m_prevQ;

    // Signal flow (per sample):
    //   [1] IQ  →  FM discriminator (atan2)
    //   [2]     →  FIR low-pass 95-tap Hamming, fc=20kHz, h[n]=h[N-1-n]
    //   [3]     →  normalisation (soft clamp, optional)
    //   [4]     →  ring buffer → QIODevice → QAudioSink

    const float pi   = static_cast<float>(M_PI);
    const float norm = GAIN / pi;   // atan2 ∈ [−π,π] / π → [−1,1]; ×GAIN ≤ 1 ⟹ no clip

    for (int i = 0; i < numSamples; ++i) {
        float I = iq[2*i];
        float Q = iq[2*i+1];

        // --- Doppler phasor correction ---
        {
            const float Ic = I * m_corrCos - Q * m_corrSin;
            const float Qc = I * m_corrSin + Q * m_corrCos;
            I = Ic; Q = Qc;
            const float nc = m_corrCos * m_corrCosStep - m_corrSin * m_corrSinStep;
            const float ns = m_corrCos * m_corrSinStep + m_corrSin * m_corrCosStep;
            m_corrCos = nc;
            m_corrSin = ns;
        }

        // [1] FM discriminator — atan2, amplitude-invariant
        //   cross = Im(conj(prev)·curr) = I·Qprev − Q·Iprev
        //   dot   = Re(conj(prev)·curr) = I·Iprev + Q·Qprev
        //   disc  = atan2(cross, dot) / π  ∈ [−1, 1]
        const float cross = I * prevQ - Q * prevI;
        const float dot   = I * prevI + Q * prevQ;
        const float disc  = std::atan2(cross, dot) * norm;

        prevI = I;
        prevQ = Q;

        // [2] FIR low-pass — 95 taps, Hamming, fc=20kHz, h[n]=h[N-1-n], Σh=1
        //   Circular delay line; newest sample at m_firIdx, then convolve.
        m_firBuf[m_firIdx] = disc;
        float lp = 0.0f;
        for (int k = 0; k < kFirTaps; ++k)
            lp += kFirLP[k] * m_firBuf[(m_firIdx - k + kFirTaps) % kFirTaps];
        m_firIdx = (m_firIdx + 1) % kFirTaps;

        // [3] FM noise parabola compensator — 2-pole IIR LPF
        //   Each pole: y = α·y + β·x  (1st-order LPF, α=0.85, fc≈1.2 kHz)
        //   Two in cascade: combined response ∝ 1/f² above fc
        //   f² (FM noise) × 1/f² (compensator) → flat/falling spectrum
        //   α = exp(−2π×1200/48000) ≈ 0.85
        static constexpr float kA = 0.80f;   // fc ≈ 1.5 kHz — gentler slope than 0.85
        static constexpr float kB = 1.0f - kA;
        m_deemph1 = kA * m_deemph1 + kB * lp;
        m_deemph2 = kA * m_deemph2 + kB * m_deemph1;

        // [4] Biquad peaking EQ @ 11.75 kHz  (+9 dB, Q=2.0)  — Direct Form I
        //   Fills the external notch at 10–13.5 kHz (radio/DAX artefact).
        //   Narrow peak (BW ≈ fc/Q ≈ 5.9 kHz → covers 8.8–14.7 kHz) so the
        //   rest of the spectrum is untouched, unlike a broad Q=0.7 bell.
        //   Audio EQ Cookbook, peaking EQ, fc=11750, fs=48000:
        //     w0=2π·11750/48000=1.5376  cos=0.03319  sin=0.99945
        //     A=10^(9/40)=1.6788  α=sin/(2Q)=0.24986
        //   Coefficients normalised by a0 (b1=a1, symmetric peaking):
        //     b0=1.23561  b1=−0.05778  b2=0.50529
        //     a1=−0.05778 a2=0.74092
        //   DC gain = 1.0 ✓  Nyquist gain = 1.0 ✓  peak @11.75 kHz = +9 dB ✓
        float comp = m_deemph2;
        if (m_eqEnabled) {
            static constexpr float b0 =  1.23561f;
            static constexpr float b1 = -0.05778f;
            static constexpr float b2 =  0.50529f;
            static constexpr float a1 = -0.05778f;
            static constexpr float a2 =  0.74092f;
            const float y = b0 * comp + b1 * m_eqX1 + b2 * m_eqX2
                                      - a1 * m_eqY1 - a2 * m_eqY2;
            m_eqX2 = m_eqX1;  m_eqX1 = comp;
            m_eqY2 = m_eqY1;  m_eqY1 = y;
            comp = y;
        }

        // [5] Soft clamp + volume
        const float s = std::clamp(comp, -1.0f, 1.0f) * m_volume;

        // [4] Stereo Float32 → ring buffer → QIODevice → QAudioSink
        out[2*i]   = s;
        out[2*i+1] = s;
    }

    m_prevI = prevI;
    m_prevQ = prevQ;

    // Renormalise phasor every block to prevent drift
    const float pn = std::sqrt(m_corrCos * m_corrCos + m_corrSin * m_corrSin);
    if (pn > 1e-9f) { m_corrCos /= pn; m_corrSin /= pn; }

    // Diagnostic log ~every 2 s
    static int s_blk = 0;
    if (++s_blk % 100 == 0) {
        float iqRms = 0.0f, audMax = 0.0f;
        for (int i = 0; i < numSamples; ++i) {
            const float rI = iq[2*i], rQ = iq[2*i+1];
            iqRms += rI*rI + rQ*rQ;
            if (std::abs(out[2*i]) > audMax) audMax = std::abs(out[2*i]);
        }
        iqRms = std::sqrt(iqRms / numSamples);
        qCDebug(lcAudio) << "WfmDemodulator blk#" << s_blk
                         << "IQ_rms=" << iqRms
                         << "audio_max=" << audMax
                         << "path=" << (m_usingDaxCapture ? "dax-capture" : "vita49");
    }

    m_waveOut->write(pcm);
}

} // namespace AetherSDR
