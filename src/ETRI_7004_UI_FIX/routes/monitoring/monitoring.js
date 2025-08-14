const express = require('express');
const path = require('path');
const rtsp = require('rtsp-ffmpeg');
const router = express.Router();
const rtspStreams = {};

// 특정 RTSP 스트림 시작 함수
function startMonitoringRTSPStream(io, cctvId, rtspUrl) {
  if (rtspStreams[cctvId]) {
    console.log(`RTSP stream already running for CCTV ${cctvId}`);
    return;
  }

  console.log(`Attempting to start RTSP stream for ${cctvId} with URL: ${rtspUrl}`);
  
  // Check if this is a port 1935 stream (likely RTMP-over-RTSP)
  const isPort1935 = rtspUrl.includes(':1935');
  console.log(`Is port 1935 stream: ${isPort1935}`);
  
  if (isPort1935) {
    console.log(`Special handling for port 1935 stream: ${cctvId}`);
  }

  // Different configuration for port 1935 streams (likely RTMP-over-RTSP)
  let streamConfig = {
    input: rtspUrl,
    resolution: '640x480',
    quality: 5,
    rate: 30
  };

  if (isPort1935) {
    // Special arguments for port 1935 RTMP-like streams
    streamConfig.arguments = [
      '-rtsp_transport', 'tcp',
      '-fflags', 'nobuffer',
      '-flags', 'low_delay',
      '-strict', 'experimental',
      '-analyzeduration', '1000000',
      '-probesize', '1000000',
      '-max_delay', '0',
      '-reorder_queue_size', '0',
      '-buffer_size', '1024000'
    ];
  } else {
    // Standard RTSP arguments
    streamConfig.arguments = [
      '-fflags', 'nobuffer',
      '-flags', 'low_delay',
      '-strict', 'experimental',
      '-analyzeduration', '0',
      '-probesize', '32'
    ];
  }

  const stream = new rtsp.FFMpeg(streamConfig);

  stream.on('data', (frame) => {
    io.emit(`${cctvId}_videoFrame`, frame);
  });

  stream.on('error', (err) => {
    console.error(`RTSP stream error for CCTV ${cctvId}:`, err);
    setTimeout(() => startMonitoringRTSPStream(io, cctvId, rtspUrl), 5000);
  });

  stream.on('end', () => {
    console.log(`RTSP stream ended for CCTV ${cctvId}`);
    setTimeout(() => startMonitoringRTSPStream(io, cctvId, rtspUrl), 5000);
  });

  rtspStreams[cctvId] = stream;
  console.log(`RTSP stream started for CCTV ${cctvId}`);
}

// 모든 RTSP 스트림 시작 API
router.post('/api/startAllMonitoringRTSPStreams', (req, res) => {
  const cctvConfigs = req.app.locals.config.monitoring;

  if (!cctvConfigs || !Array.isArray(cctvConfigs)) {
    return res.status(400).send('Invalid CCTV configuration');
  }

  const io = req.app.locals.io;

  cctvConfigs.forEach(({ id, rtsp }) => {
    // 🚫 빈 RTSP 주소 방지
    if (!rtsp || typeof rtsp !== 'string' || rtsp.trim() === '') {
      console.warn(`Skipping RTSP stream for ${id} due to empty or invalid URL`);
      return;
    }

    startMonitoringRTSPStream(io, id, rtsp);
  });

  res.status(200).send('RTSP streams started for all valid CCTVs');
});

// 모니터링 페이지 제공
router.get('/', (req, res) => {
  res.sendFile(path.join(__dirname, '../../public', 'monitoring', 'monitoring.html'));
});

module.exports = router;
