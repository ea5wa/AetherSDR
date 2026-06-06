#include "core/WaveOutWriter.h"
#include "core/LogManager.h"
#include <QMediaDevices>

namespace AetherSDR {

WaveOutWriter::WaveOutWriter(QObject* parent)
    : QObject(parent)
{}

WaveOutWriter::~WaveOutWriter()
{
    close();
}

bool WaveOutWriter::open(const QString& deviceId, int sampleRate, int channelCount)
{
    close();

    // Find the requested device by its persistent ID string.
    QAudioDevice found;
    const auto outputs = QMediaDevices::audioOutputs();
    qCDebug(lcAudio) << "WaveOutWriter::open looking for" << deviceId
                     << "among" << outputs.size() << "devices";
    for (const QAudioDevice& dev : outputs) {
        if (dev.id() == deviceId.toUtf8()) {
            found = dev;
            break;
        }
    }
    if (found.isNull()) {
        qCDebug(lcAudio) << "WaveOutWriter::open: device not found —" << deviceId;
        return false;
    }

    QAudioFormat fmt;
    fmt.setSampleRate(sampleRate);
    fmt.setChannelCount(channelCount);
    fmt.setSampleFormat(QAudioFormat::Int16);

    if (!found.isFormatSupported(fmt)) {
        qCDebug(lcAudio) << "WaveOutWriter::open: Int16 format not supported by"
                         << found.description() << "— trying default format";
        fmt = found.preferredFormat();
    }

    m_sink = new QAudioSink(found, fmt, this);
    m_io   = m_sink->start();

    if (!m_io || m_sink->error() != QAudio::NoError) {
        qCDebug(lcAudio) << "WaveOutWriter::open: QAudioSink::start() failed, error="
                         << m_sink->error();
        delete m_sink;
        m_sink = nullptr;
        m_io   = nullptr;
        return false;
    }

    m_deviceName = found.description();
    qCDebug(lcAudio) << "WaveOutWriter opened:" << m_deviceName
                     << "rate=" << fmt.sampleRate()
                     << "ch="   << fmt.channelCount()
                     << "fmt="  << fmt.sampleFormat();
    return true;
}

void WaveOutWriter::close()
{
    if (m_sink) {
        m_sink->stop();
        delete m_sink;
        m_sink = nullptr;
        m_io   = nullptr;
        qCDebug(lcAudio) << "WaveOutWriter closed";
    }
    m_deviceName.clear();
}

void WaveOutWriter::write(const QByteArray& pcm)
{
    if (!m_io || pcm.isEmpty())
        return;
    m_io->write(pcm);
}

} // namespace AetherSDR
