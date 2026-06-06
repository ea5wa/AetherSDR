#include "WfmDemodulator.h"
#include "WaveOutWriter.h"
#include "WaveInReader.h"
#include "models/DaxIqModel.h"

#include <QTextStream>
#include <algorithm>
#include <cmath>


namespace AetherSDR {

WfmDemodulator::WfmDemodulator(QObject* parent)
    : QObject(parent)
{}

WfmDemodulator::~WfmDemodulator()
{
    stop();
}

void WfmDemodulator::start(DaxIqModel* daxIq, const QString& audioDevice,
                           const QString& panId, float freqOffsetHz)
{
    if (m_active) stop();

    m_daxIq       = daxIq;
    m_prevI       = 0.0f;
    m_prevQ       = 0.0f;
    m_corrCos     = 1.0f;
    m_corrSin     = 0.0f;
    m_panId       = panId;
    m_panSent     = false;
    m_usingWaveIn = false;

    // Pre-compute the per-sample rotation for frequency correction.
    // The shift is NEGATIVE to move the signal down to DC.
    const float step = -2.0f * static_cast<float>(M_PI) * freqOffsetHz / IQ_RATE;
    m_corrCosStep = std::cos(step);
    m_corrSinStep = std::sin(step);
    qDebug().noquote() << QString("WfmDemodulator::start device='%1' freqOffsetHz=%2")
           .arg(audioDevice).arg(freqOffsetHz, 0, 'f', 1);

    m_waveOut = new WaveOutWriter(this);
    if (!m_waveOut->open(audioDevice, AUDIO_RATE, 2, 16)) {
        qDebug().noquote() << "WfmDemodulator: failed to open audio device: " + audioDevice;
        delete m_waveOut;
        m_waveOut = nullptr;
        return;
    }

    // Try SmartSDR DAX waveIn IQ device first (primary path on Windows with DAX).
    // SmartSDR DAX delivers IQ as stereo audio: L=I, R=Q at the configured IQ rate.
    m_waveIn = new WaveInReader(this);
    bool daxOk = m_waveIn->open("DAX IQ RX 1", IQ_RATE, 2, 16);
    if (!daxOk) daxOk = m_waveIn->open("DAX RESERVED IQ RX 1", IQ_RATE, 2, 16);
    if (!daxOk) daxOk = m_waveIn->open("DAX IQ", IQ_RATE, 2, 16);

    if (daxOk) {
        m_usingWaveIn = true;
        connect(m_waveIn, &WaveInReader::pcmReady,
                this, &WfmDemodulator::onDaxIqPcm);
        qDebug().noquote() << QString("WfmDemodulator: using waveIn path '%1'").arg(m_waveIn->deviceName());
    } else {
        // Fall back to VITA-49 DaxIqModel path
        delete m_waveIn;
        m_waveIn = nullptr;
        connect(m_daxIq, &DaxIqModel::iqSamplesReady, this, &WfmDemodulator::onIqSamples);
        connect(m_daxIq, &DaxIqModel::streamChanged,   this, &WfmDemodulator::onStreamChanged);
        qDebug().noquote() << QString("WfmDemodulator::start panId='%1' (VITA-49 fallback)").arg(m_panId);
        m_daxIq->createStream(DAX_CHANNEL);
    }

    m_active = true;
}

void WfmDemodulator::stop()
{
    if (!m_active) return;
    m_active = false;

    if (m_waveIn) {
        m_waveIn->close();
        delete m_waveIn;
        m_waveIn = nullptr;
    }
    if (m_daxIq) {
        if (!m_usingWaveIn)
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
    qDebug().noquote() << QString("onStreamChanged: ch=%1 panSent=%2 panId='%3' exists=%4 active=%5 streamId=0x%6")
           .arg(channel).arg(m_panSent).arg(m_panId)
           .arg(s.exists).arg(s.active)
           .arg(s.streamId, 0, 16);
    if (channel != DAX_CHANNEL || m_panSent || m_panId.isEmpty()) return;
    if (!s.exists || s.streamId == 0) return;
    const QString cmd = QString("stream set 0x%1 pan=%2")
                        .arg(s.streamId, 0, 16).arg(m_panId);
    qDebug().noquote() << QString("onStreamChanged: sending '%1'").arg(cmd);
    emit commandReady(cmd);
    m_panSent = true;
}

// Primary IQ path: SmartSDR DAX waveIn (stereo Int16, L=I, R=Q)
void WfmDemodulator::onDaxIqPcm(const QByteArray& pcm)
{
    if (!m_active || !m_waveOut) return;
    const int numSamples = pcm.size() / (2 * sizeof(qint16));
    if (numSamples <= 0) return;

    const auto* src = reinterpret_cast<const qint16*>(pcm.constData());
    QVector<float> iq(numSamples * 2);
    for (int i = 0; i < numSamples; ++i) {
        iq[2*i]   = src[2*i]   / 32768.0f;   // I = left channel
        iq[2*i+1] = src[2*i+1] / 32768.0f;   // Q = right channel
    }
    processSamples(iq);
}

// Fallback IQ path: VITA-49 DaxIqModel
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

        // Frequency correction: rotate IQ by the running phasor to center
        // the signal at DC (removes the panadapter→slice frequency offset).
        {
            const float Ic = I * m_corrCos - Q * m_corrSin;
            const float Qc = I * m_corrSin + Q * m_corrCos;
            I = Ic;  Q = Qc;
            // Advance phasor (numerically stable incremental rotation)
            const float newCos = m_corrCos * m_corrCosStep - m_corrSin * m_corrSinStep;
            const float newSin = m_corrCos * m_corrSinStep + m_corrSin * m_corrCosStep;
            // Renormalize every 4096 samples to prevent drift
            m_corrCos = newCos;
            m_corrSin = newSin;
        }

        const float amp = std::sqrt(I * I + Q * Q);
        if (amp > 1e-9f) { I /= amp; Q /= amp; }
        else              { I = prevI; Q = prevQ; }

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

    // Renormalize frequency-correction phasor every block to prevent drift
    {
        const float norm = std::sqrt(m_corrCos * m_corrCos + m_corrSin * m_corrSin);
        if (norm > 1e-9f) { m_corrCos /= norm; m_corrSin /= norm; }
    }

    static int s_blk = 0;
    ++s_blk;
    if (s_blk % 100 == 0) {
        float iqRms = 0, audioRms = 0, audioMax = 0;
        for (int i = 0; i < numSamples; ++i) {
            const float rI = iqInterleaved[2*i], rQ = iqInterleaved[2*i+1];
            iqRms += rI*rI + rQ*rQ;
            const float a = std::abs(reinterpret_cast<const qint16*>(pcm.constData())[i*2] / 32767.0f);
            audioRms += a*a;
            if (a > audioMax) audioMax = a;
        }
        iqRms    = std::sqrt(iqRms / numSamples);
        audioRms = std::sqrt(audioRms / numSamples);
        qDebug().noquote() << QString("blk#%1 IQ_rms=%2 audio_rms=%3 audio_max=%4 path=%5")
               .arg(s_blk).arg(iqRms,0,'f',6)
               .arg(audioRms,0,'f',4).arg(audioMax,0,'f',4)
               .arg(m_usingWaveIn ? "waveIn" : "vita49");
    }

    m_waveOut->write(pcm);
}

} // namespace AetherSDR
