#include "WaveInReader.h"
#include <QFile>
#include <QTextStream>
#include <cstring>

static void wfmLog(const QString& msg)
{
    QFile f("C:/Users/reigc/wfm_debug.txt");
    if (f.open(QIODevice::Append | QIODevice::Text))
        QTextStream(&f) << msg << "\n";
}

namespace AetherSDR {

WaveInReader::WaveInReader(QObject* parent)
    : QObject(parent)
{}

WaveInReader::~WaveInReader()
{
    close();
}

bool WaveInReader::open(const QString& deviceNameFragment, int sampleRate,
                         int channels, int bitsPerSample)
{
    close();

    const UINT numDevs = waveInGetNumDevs();
    UINT devId = WAVE_MAPPER;
    {
        QString allDevs;
        for (UINT i = 0; i < numDevs; ++i) {
            WAVEINCAPSW caps{};
            if (waveInGetDevCapsW(i, &caps, sizeof(caps)) == MMSYSERR_NOERROR) {
                const QString name = QString::fromWCharArray(caps.szPname);
                allDevs += QString("  [%1] %2\n").arg(i).arg(name);
                if (m_deviceName.isEmpty()
                        && name.contains(deviceNameFragment, Qt::CaseInsensitive)) {
                    devId = i;
                    m_deviceName = name;
                }
            }
        }
        wfmLog(QString("WaveInReader::open looking for '%1' among %2 waveIn devices:\n%3")
               .arg(deviceNameFragment).arg(numDevs).arg(allDevs.trimmed()));
    }

    if (!deviceNameFragment.isEmpty() && m_deviceName.isEmpty()) {
        wfmLog(QString("WaveInReader::open: '%1' not found — aborting").arg(deviceNameFragment));
        return false;
    }
    if (m_deviceName.isEmpty())
        m_deviceName = "(default)";

    WAVEFORMATEX wfx{};
    wfx.wFormatTag      = WAVE_FORMAT_PCM;
    wfx.nChannels       = static_cast<WORD>(channels);
    wfx.nSamplesPerSec  = static_cast<DWORD>(sampleRate);
    wfx.wBitsPerSample  = static_cast<WORD>(bitsPerSample);
    wfx.nBlockAlign     = wfx.nChannels * (wfx.wBitsPerSample / 8);
    wfx.nAvgBytesPerSec = wfx.nSamplesPerSec * wfx.nBlockAlign;

    m_captureEvent = CreateEvent(NULL, FALSE, FALSE, NULL);

    const MMRESULT res = waveInOpen(&m_hWaveIn, devId, &wfx,
                                    reinterpret_cast<DWORD_PTR>(m_captureEvent),
                                    0, CALLBACK_EVENT);
    if (res != MMSYSERR_NOERROR) {
        wfmLog(QString("waveInOpen failed: %1").arg(res));
        CloseHandle(m_captureEvent);
        m_captureEvent = nullptr;
        m_hWaveIn      = nullptr;
        return false;
    }

    m_bytesPerBuf = BUF_SAMPLES * channels * (bitsPerSample / 8);
    for (int i = 0; i < NUM_BUFS; ++i) {
        m_buffers[i].resize(m_bytesPerBuf, 0);
        auto& hdr = m_headers[i];
        std::memset(&hdr, 0, sizeof(hdr));
        hdr.lpData         = m_buffers[i].data();
        hdr.dwBufferLength = static_cast<DWORD>(m_bytesPerBuf);
        waveInPrepareHeader(m_hWaveIn, &hdr, sizeof(hdr));
        waveInAddBuffer(m_hWaveIn, &hdr, sizeof(hdr));
    }

    m_running       = true;
    m_captureThread = CreateThread(NULL, 0, captureThreadProc, this, 0, NULL);
    SetThreadPriority(m_captureThread, THREAD_PRIORITY_ABOVE_NORMAL);

    waveInStart(m_hWaveIn);

    wfmLog(QString("WaveInReader opened: device='%1' rate=%2 ch=%3 bits=%4")
           .arg(m_deviceName).arg(sampleRate).arg(channels).arg(bitsPerSample));
    return true;
}

void WaveInReader::close()
{
    if (!m_hWaveIn) return;

    m_running = false;
    if (m_captureEvent) SetEvent(m_captureEvent);
    if (m_captureThread) {
        WaitForSingleObject(m_captureThread, 2000);
        CloseHandle(m_captureThread);
        m_captureThread = nullptr;
    }

    waveInStop(m_hWaveIn);
    waveInReset(m_hWaveIn);
    for (int i = 0; i < NUM_BUFS; ++i)
        waveInUnprepareHeader(m_hWaveIn, &m_headers[i], sizeof(m_headers[i]));
    waveInClose(m_hWaveIn);
    m_hWaveIn = nullptr;

    if (m_captureEvent) {
        CloseHandle(m_captureEvent);
        m_captureEvent = nullptr;
    }
    wfmLog("WaveInReader closed");
}

// static
DWORD WINAPI WaveInReader::captureThreadProc(LPVOID param)
{
    WaveInReader* r = reinterpret_cast<WaveInReader*>(param);
    r->captureLoop();
    return 0;
}

void WaveInReader::captureLoop()
{
    while (m_running) {
        WaitForSingleObject(m_captureEvent, 50);
        if (!m_running) break;

        for (int i = 0; i < NUM_BUFS; ++i) {
            auto& hdr = m_headers[i];
            if (!(hdr.dwFlags & WHDR_DONE)) continue;

            QByteArray pcm(m_buffers[i].constData(), m_bytesPerBuf);
            emit pcmReady(pcm);

            hdr.dwFlags &= ~WHDR_DONE;
            waveInAddBuffer(m_hWaveIn, &hdr, sizeof(hdr));
        }
    }
}

} // namespace AetherSDR
