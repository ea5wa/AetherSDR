#include "core/WfmDemodulator.h"
#include "core/WaveOutWriter.h"
#include "core/LogManager.h"
#include "models/DaxIqModel.h"

#include <QMediaDevices>
#include <cmath>
#include <algorithm>

namespace AetherSDR {

// Try to find a QAudioDevice whose description contains any of the given
// substrings (case-insensitive).  Returns a null device if not found.
static QAudioDevice findCaptureDevice(const QStringList& hints)
{
    const auto inputs = QMediaDevices::audioInputs();
    for (const QAudioDevice& dev : inputs) {
        const QString desc = dev.description().toLower();
        for (const QString& h : hints) {
            if (desc.contains(h.toLower()))
                return dev;
        }
    }
    return {};
}

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
    m_usingDaxCapture = false;

    const float step = -2.0f * static_cast<float>(M_PI) * freqOffsetHz / IQ_RATE;
    m_corrCosStep = std::cos(step);
    m_corrSinStep = std::sin(step);

    qCDebug(lcAudio) << "WfmDemodulator::start deviceId=" << deviceId
                     << "freqOffsetHz=" << freqOffsetHz;

    // Open output device first.
    m_waveOut = new WaveOutWriter(this);
    if (!m_waveOut->open(deviceId, AUDIO_RATE, 2)) {
        qCDebug(lcAudio) << "WfmDemodulator: failed to open audio output:" << deviceId;
        delete m_waveOut;
        m_waveOut = nullptr;
        return;
    }

    // Primary path: read IQ from SmartSDR DAX audio capture device.
    // DAX delivers IQ as stereo PCM: L=I, R=Q at IQ_RATE.
    // Device names vary by DAX version; try several substrings.
    const QStringList daxHints = {
        "dax iq rx 1", "dax iq 1", "dax reserved iq rx 1", "dax iq"
    };
    QAudioDevice capDev = findCaptureDevice(daxHints);

    if (!capDev.isNull()) {
        QAudioFormat fmt;
        fmt.setSampleRate(IQ_RATE);
        fmt.setChannelCount(2);
        fmt.setSampleFormat(QAudioFormat::Int16);
        if (!capDev.isFormatSupported(fmt))
            fmt = capDev.preferredFormat();

        m_iqDevice = new DaxIqCaptureDevice(this);
        m_iqDevice->open(QIODevice::WriteOnly);
        connect(m_iqDevice, &DaxIqCaptureDevice::pcmReady,
                this, &WfmDemodulator::onDaxIqPcm);

        m_iqSource = new QAudioSource(capDev, fmt, this);
        m_iqSource->start(m_iqDevice);

        if (m_iqSource->error() == QAudio::NoError) {
            m_usingDaxCapture = true;
            qCDebug(lcAudio) << "WfmDemodulator: DAX capture path:"
                             << capDev.description()
                             << "rate=" << fmt.sampleRate()
                             << "fmt=" << fmt.sampleFormat();
        } else {
            qCDebug(lcAudio) << "WfmDemodulator: DAX capture failed, error="
                             << m_iqSource->error() << "— falling back to VITA-49";
            m_iqSource->stop();
            delete m_iqSource;  m_iqSource = nullptr;
            delete m_iqDevice;  m_iqDevice = nullptr;
        }
    } else {
        qCDebug(lcAudio) << "WfmDemodulator: no DAX IQ capture device found";
    }

    // Fallback: VITA-49 DaxIqModel path (Linux / macOS, or if DAX not running).
    if (!m_usingDaxCapture) {
        connect(m_daxIq, &DaxIqModel::iqSamplesReady,
                this, &WfmDemodulator::onIqSamples);
        connect(m_daxIq, &DaxIqModel::streamChanged,
                this, &WfmDemodulator::onStreamChanged);
        m_daxIq->createStream(DAX_CHANNEL);
        qCDebug(lcAudio) << "WfmDemodulator: using VITA-49 fallback, panId=" << m_panId;
    }

    m_active = true;
}

void WfmDemodulator::stop()
{
    if (!m_active) return;
    m_active = false;

    if (m_iqSource) {
        m_iqSource->stop();
        delete m_iqSource;
        m_iqSource = nullptr;
    }
    if (m_iqDevice) {
        m_iqDevice->close();
        delete m_iqDevice;
        m_iqDevice = nullptr;
    }
    if (m_daxIq) {
        if (!m_usingDaxCapture)
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
                     << "exists=" << s.exists << "streamId=0x" + QString::number(s.streamId, 16);
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
    processIqFloat(iqInterleaved);
}

// DAX audio capture: stereo Int16 PCM, L=I, R=Q.
void WfmDemodulator::onDaxIqPcm(const QByteArray& pcm)
{
    if (!m_active || !m_waveOut) return;
    processIqInt16(pcm);
}

void WfmDemodulator::processIqInt16(const QByteArray& pcm)
{
    const int numSamples = pcm.size() / (2 * sizeof(qint16));
    if (numSamples <= 0) return;

    const auto* raw = reinterpret_cast<const qint16*>(pcm.constData());
    QVector<float> iq(numSamples * 2);
    for (int i = 0; i < numSamples; ++i) {
        iq[2 * i]     = raw[2 * i]     / 32768.0f;  // L = I
        iq[2 * i + 1] = raw[2 * i + 1] / 32768.0f;  // R = Q
    }
    processIqFloat(iq);
}

void WfmDemodulator::processIqFloat(const QVector<float>& iqInterleaved)
{
    const int numSamples = iqInterleaved.size() / 2;
    if (numSamples <= 0) return;

    // Float32 stereo output — universally supported by Qt audio backends.
    QByteArray pcm(numSamples * 2 * sizeof(float), Qt::Uninitialized);
    auto* out = reinterpret_cast<float*>(pcm.data());

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

        // Phase-difference FM discriminator.
        // Negated to correct spectral inversion introduced by the DAX IQ
        // channel order (SmartSDR DAX delivers L=I, R=Q with upper-sideband
        // mixing, which inverts the baseband spectrum).
        const float cross = I * prevQ - Q * prevI;
        const float dot   = I * prevI + Q * prevQ;
        float audio = -std::atan2(cross, dot) * (GAIN / static_cast<float>(M_PI));
        audio = std::clamp(audio, -1.0f, 1.0f);

        prevI = I;
        prevQ = Q;

        const float s = audio * m_volume;
        out[i * 2]     = s;
        out[i * 2 + 1] = s;
    }

    m_prevI = prevI;
    m_prevQ = prevQ;

    // Renormalize frequency-correction phasor every block to prevent drift.
    const float norm = std::sqrt(m_corrCos * m_corrCos + m_corrSin * m_corrSin);
    if (norm > 1e-9f) { m_corrCos /= norm; m_corrSin /= norm; }

    // Periodic diagnostic (~every 2 s)
    static int s_blk = 0;
    if (++s_blk % 100 == 0) {
        float iqRms = 0, audioMax = 0;
        for (int i = 0; i < numSamples; ++i) {
            const float rI = iqInterleaved[2*i], rQ = iqInterleaved[2*i+1];
            iqRms += rI*rI + rQ*rQ;
            if (std::abs(out[i*2]) > audioMax) audioMax = std::abs(out[i*2]);
        }
        iqRms = std::sqrt(iqRms / numSamples);
        qCDebug(lcAudio) << "WfmDemodulator blk#" << s_blk
                         << "IQ_rms=" << iqRms << "audio_max=" << audioMax
                         << "path=" << (m_usingDaxCapture ? "dax-capture" : "vita49");
    }

    m_waveOut->write(pcm);
}

} // namespace AetherSDR
