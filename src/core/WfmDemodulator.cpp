#include "core/WfmDemodulator.h"
#include "core/WaveOutWriter.h"
#include "core/LogManager.h"
#include "models/DaxIqModel.h"

#include <cmath>
#include <algorithm>

namespace AetherSDR {

WfmDemodulator::WfmDemodulator(QObject* parent)
    : QObject(parent)
{}

WfmDemodulator::~WfmDemodulator()
{
    stop();
}

void WfmDemodulator::start(DaxIqModel* daxIq, const QString& deviceId,
                           const QString& panId, float freqOffsetHz)
{
    if (m_active) stop();

    m_daxIq   = daxIq;
    m_prevI   = 0.0f;
    m_prevQ   = 0.0f;
    m_corrCos = 1.0f;
    m_corrSin = 0.0f;
    m_panId   = panId;
    m_panSent = false;

    // Pre-compute the per-sample rotation for frequency correction.
    const float step = -2.0f * static_cast<float>(M_PI) * freqOffsetHz / IQ_RATE;
    m_corrCosStep = std::cos(step);
    m_corrSinStep = std::sin(step);

    qCDebug(lcAudio) << "WfmDemodulator::start deviceId=" << deviceId
                     << "freqOffsetHz=" << freqOffsetHz;

    m_waveOut = new WaveOutWriter(this);
    if (!m_waveOut->open(deviceId, AUDIO_RATE, 2)) {
        qCDebug(lcAudio) << "WfmDemodulator: failed to open audio device:" << deviceId;
        delete m_waveOut;
        m_waveOut = nullptr;
        return;
    }

    // Wire VITA-49 DaxIqModel path (cross-platform).
    connect(m_daxIq, &DaxIqModel::iqSamplesReady,
            this, &WfmDemodulator::onIqSamples);
    connect(m_daxIq, &DaxIqModel::streamChanged,
            this, &WfmDemodulator::onStreamChanged);
    m_daxIq->createStream(DAX_CHANNEL);

    m_active = true;
}

void WfmDemodulator::stop()
{
    if (!m_active) return;
    m_active = false;

    if (m_daxIq) {
        m_daxIq->removeStream(DAX_CHANNEL);
        disconnect(m_daxIq, nullptr, this, nullptr);
        m_daxIq = nullptr;
    }
    if (m_waveOut) {
        m_waveOut->close();
        delete m_waveOut;
        m_waveOut = nullptr;
    }
}

void WfmDemodulator::onStreamChanged(int channel)
{
    const auto& s = m_daxIq->stream(DAX_CHANNEL);
    qCDebug(lcAudio) << "WfmDemodulator::onStreamChanged ch=" << channel
                     << "panSent=" << m_panSent << "panId=" << m_panId
                     << "exists=" << s.exists << "active=" << s.active
                     << "streamId=0x" + QString::number(s.streamId, 16);
    if (channel != DAX_CHANNEL || m_panSent || m_panId.isEmpty()) return;
    if (!s.exists || s.streamId == 0) return;
    const QString cmd = QString("stream set 0x%1 pan=%2")
                        .arg(s.streamId, 0, 16).arg(m_panId);
    qCDebug(lcAudio) << "WfmDemodulator: sending" << cmd;
    emit commandReady(cmd);
    m_panSent = true;
}

void WfmDemodulator::onIqSamples(int channel, QVector<float> iqInterleaved, int /*sampleRate*/)
{
    if (channel != DAX_CHANNEL || !m_active || !m_waveOut) return;
    processSamples(iqInterleaved);
}

void WfmDemodulator::processSamples(const QVector<float>& iqInterleaved)
{
    const int numSamples = iqInterleaved.size() / 2;
    if (numSamples <= 0) return;

    QByteArray pcm(numSamples * 2 * sizeof(qint16), Qt::Uninitialized);
    auto* out = reinterpret_cast<qint16*>(pcm.data());

    float prevI = m_prevI;
    float prevQ = m_prevQ;

    for (int i = 0; i < numSamples; ++i) {
        float I = iqInterleaved[2 * i];
        float Q = iqInterleaved[2 * i + 1];

        // Frequency correction: rotate IQ by the running phasor.
        {
            const float Ic = I * m_corrCos - Q * m_corrSin;
            const float Qc = I * m_corrSin + Q * m_corrCos;
            I = Ic; Q = Qc;
            const float newCos = m_corrCos * m_corrCosStep - m_corrSin * m_corrSinStep;
            const float newSin = m_corrCos * m_corrSinStep + m_corrSin * m_corrCosStep;
            m_corrCos = newCos;
            m_corrSin = newSin;
        }

        // Normalize to unit circle (carrier lock)
        const float amp = std::sqrt(I * I + Q * Q);
        if (amp > 1e-9f) { I /= amp; Q /= amp; }
        else              { I = prevI; Q = prevQ; }

        // Phase-difference FM discriminator
        const float cross = I * prevQ - Q * prevI;
        const float dot   = I * prevI + Q * prevQ;
        float audio = std::atan2(cross, dot) * (GAIN / static_cast<float>(M_PI));
        audio = std::max(-1.0f, std::min(1.0f, audio));

        prevI = I;
        prevQ = Q;

        const qint16 s16 = static_cast<qint16>(audio * 32767.0f * m_volume);
        out[i * 2]     = s16;
        out[i * 2 + 1] = s16;
    }

    m_prevI = prevI;
    m_prevQ = prevQ;

    // Renormalize frequency-correction phasor every block to prevent drift.
    {
        const float norm = std::sqrt(m_corrCos * m_corrCos + m_corrSin * m_corrSin);
        if (norm > 1e-9f) { m_corrCos /= norm; m_corrSin /= norm; }
    }

    // Periodic signal-level diagnostic (every 100 blocks ≈ every ~2 s at 48 kHz/512)
    static int s_blk = 0;
    if (++s_blk % 100 == 0) {
        float iqRms = 0, audioRms = 0, audioMax = 0;
        for (int i = 0; i < numSamples; ++i) {
            const float rI = iqInterleaved[2*i], rQ = iqInterleaved[2*i+1];
            iqRms += rI*rI + rQ*rQ;
            const float a = std::abs(out[i*2] / 32767.0f);
            audioRms += a*a;
            if (a > audioMax) audioMax = a;
        }
        iqRms    = std::sqrt(iqRms / numSamples);
        audioRms = std::sqrt(audioRms / numSamples);
        qCDebug(lcAudio) << "WfmDemodulator blk#" << s_blk
                         << "IQ_rms=" << iqRms
                         << "audio_rms=" << audioRms
                         << "audio_max=" << audioMax;
    }

    m_waveOut->write(pcm);
}

} // namespace AetherSDR
