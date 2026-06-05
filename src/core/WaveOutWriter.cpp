#include "WaveOutWriter.h"
#include <QFile>
#include <QTextStream>
#include <algorithm>
#include <cstring>

static void wfmLog(const QString& msg)
{
    QFile f("C:/Users/reigc/wfm_debug.txt");
    if (f.open(QIODevice::Append | QIODevice::Text))
        QTextStream(&f) << msg << "\n";
}

namespace AetherSDR {

WaveOutWriter::WaveOutWriter(QObject* parent)
    : QObject(parent)
{}

WaveOutWriter::~WaveOutWriter()
{
    close();
}

bool WaveOutWriter::open(const QString& deviceNameFragment, int sampleRate,
                          int channels, int bitsPerSample)
{
    close();

    // Find matching waveOut device
    const UINT numDevs = waveOutGetNumDevs();
    UINT       devId   = WAVE_MAPPER;
    {
        QString allDevs;
        for (UINT i = 0; i < numDevs; ++i) {
            WAVEOUTCAPSW caps{};
            if (waveOutGetDevCapsW(i, &caps, sizeof(caps)) == MMSYSERR_NOERROR) {
                const QString name = QString::fromWCharArray(caps.szPname);
                allDevs += QString("  [%1] %2\n").arg(i).arg(name);
                if (m_deviceName.isEmpty()
                        && name.contains(deviceNameFragment, Qt::CaseInsensitive)) {
                    devId = i;
                    m_deviceName = name;
                }
            }
        }
        wfmLog(QString("WaveOutWriter::open looking for '%1' among %2 devices:\n%3")
               .arg(deviceNameFragment).arg(numDevs).arg(allDevs.trimmed()));
    }
    // Fail explicitly if the named device wasn't found — don't fall back to
    // WAVE_MAPPER (which would silently route audio to the default speakers).
    if (!deviceNameFragment.isEmpty() && m_deviceName.isEmpty()) {
        wfmLog(QString("WaveOutWriter::open: '%1' not found — aborting").arg(deviceNameFragment));
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

    // Create event — waveOut signals it every time a buffer completes (WOM_DONE)
    m_refillEvent = CreateEvent(NULL, FALSE, FALSE, NULL);

    const MMRESULT res = waveOutOpen(&m_hWaveOut, devId, &wfx,
                                     reinterpret_cast<DWORD_PTR>(m_refillEvent),
                                     0, CALLBACK_EVENT);
    if (res != MMSYSERR_NOERROR) {
        wfmLog(QString("waveOutOpen failed: %1").arg(res));
        CloseHandle(m_refillEvent);
        m_refillEvent = nullptr;
        m_hWaveOut    = nullptr;
        return false;
    }

    // Prepare buffers
    m_bytesPerBuf = BUF_SAMPLES * channels * (bitsPerSample / 8);
    for (int i = 0; i < NUM_BUFS; ++i) {
        m_buffers[i].resize(m_bytesPerBuf, 0);
        auto& hdr = m_headers[i];
        std::memset(&hdr, 0, sizeof(hdr));
        hdr.lpData         = m_buffers[i].data();
        hdr.dwBufferLength = static_cast<DWORD>(m_bytesPerBuf);
        waveOutPrepareHeader(m_hWaveOut, &hdr, sizeof(hdr));
        // Pre-queue silence so driver is always playing something
        waveOutWrite(m_hWaveOut, &hdr, sizeof(hdr));
    }

    // Start refill thread
    m_running      = true;
    m_refillThread = CreateThread(NULL, 0, refillThreadProc, this, 0, NULL);
    SetThreadPriority(m_refillThread, THREAD_PRIORITY_ABOVE_NORMAL);

    wfmLog(QString("WaveOutWriter opened: device='%1' rate=%2 ch=%3 bits=%4 bufBytes=%5")
           .arg(m_deviceName).arg(sampleRate).arg(channels).arg(bitsPerSample).arg(m_bytesPerBuf));
    return true;
}

void WaveOutWriter::close()
{
    if (!m_hWaveOut) return;

    m_running = false;
    if (m_refillEvent) SetEvent(m_refillEvent);  // wake thread so it can exit
    if (m_refillThread) {
        WaitForSingleObject(m_refillThread, 2000);
        CloseHandle(m_refillThread);
        m_refillThread = nullptr;
    }

    waveOutReset(m_hWaveOut);
    for (int i = 0; i < NUM_BUFS; ++i)
        waveOutUnprepareHeader(m_hWaveOut, &m_headers[i], sizeof(m_headers[i]));
    waveOutClose(m_hWaveOut);
    m_hWaveOut = nullptr;

    if (m_refillEvent) {
        CloseHandle(m_refillEvent);
        m_refillEvent = nullptr;
    }

    m_pending.clear();
    wfmLog("WaveOutWriter closed");
}

void WaveOutWriter::write(const QByteArray& pcm)
{
    QMutexLocker lock(&m_mutex);
    m_pending.append(pcm);
}

// static
DWORD WINAPI WaveOutWriter::refillThreadProc(LPVOID param)
{
    WaveOutWriter* w = reinterpret_cast<WaveOutWriter*>(param);
    while (w->m_running) {
        // Wait for WOM_DONE event (or close signal)
        WaitForSingleObject(w->m_refillEvent, 50);
        if (w->m_running)
            w->refillDoneBuffers();
    }
    return 0;
}

void WaveOutWriter::refillDoneBuffers()
{
    for (int i = 0; i < NUM_BUFS; ++i) {
        auto& hdr = m_headers[i];
        if (!(hdr.dwFlags & WHDR_DONE)) continue;

        {
            QMutexLocker lock(&m_mutex);
            if (m_pending.size() >= m_bytesPerBuf) {
                std::memcpy(m_buffers[i].data(), m_pending.constData(), m_bytesPerBuf);
                m_pending.remove(0, m_bytesPerBuf);
            } else {
                // No real audio ready — queue silence to keep driver busy
                std::memset(m_buffers[i].data(), 0, m_bytesPerBuf);
            }
        }

        hdr.dwFlags &= ~WHDR_DONE;
        waveOutWrite(m_hWaveOut, &hdr, sizeof(hdr));
    }
}

} // namespace AetherSDR
