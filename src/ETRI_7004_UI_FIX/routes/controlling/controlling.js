const express = require('express');
const path = require('path');
const { CameraServer } = require('./modules/onvifJoystick');
const rtsp = require('rtsp-ffmpeg');
const router = express.Router();
let cameraServer;
const rtspStreams = {};

/**
 * 제어 페이지를 제공하는 라우터
 */
router.get('/', (req, res) => {
    res.sendFile(path.join(__dirname, '../../public', 'controlling', 'controlling.html'));
});

/**
 * 특정 로봇의 RTSP 스트림을 시작하는 API
 */
router.post('/api/startRTSPStream', (req, res) => {
    const robotName = req.body.robotName;
    const rtspUrl = getRTSPUrl(req, robotName);

    if (!rtspUrl) {
        return res.status(404).json({ error: 'RTSP URL not found for the selected robot' });
    }

    if (rtspStreams[robotName]) {
        return res.status(200).send('RTSP stream already running for this robot');
    }

    startRTSPStream(req, robotName, rtspUrl);
    res.status(200).send('RTSP stream started');

    initializeCameraServer(req, robotName);
});

/**
 * 주어진 로봇의 RTSP URL을 가져오는 함수
 * @param {Object} req - 요청 객체
 * @param {string} robotName - 로봇 이름
 * @returns {string} - RTSP URL
 */
function getRTSPUrl(req, robotName) {
    return req.app.locals.config.controlling.find(robot => robot[robotName])?.[robotName]?.rtsp?.RTSP_URL;
}

/**
 * 주어진 로봇의 RTSP 스트림을 시작하는 함수
 * @param {Object} req - 요청 객체
 * @param {string} robotName - 로봇 이름
 * @param {string} rtspUrl - RTSP URL
 */
function startRTSPStream(req, robotName, rtspUrl) {
    const stream = new rtsp.FFMpeg({
        input: rtspUrl,
        resolution: '640x480',
        quality: 5,
        rate: 15,
        arguments: [
            '-fflags', 'nobuffer',
            '-flags', 'low_delay',
            '-strict', 'experimental',
            '-analyzeduration', '0',
            '-probesize', '32'
          ]
    });
    
    stream.on('data', (frame) => {
        req.app.locals.io.emit(`${robotName}_videoFrame`, frame);
    });

    stream.on('error', (err) => {
        console.error(`RTSP stream error for robot ${robotName}:`, err);
        setTimeout(() => startRTSPStream(req, robotName, rtspUrl), 5000);
    });

    stream.on('end', () => {
        console.log(`RTSP stream ended for robot ${robotName}`);
        setTimeout(() => startRTSPStream(req, robotName, rtspUrl), 5000);
    });

    rtspStreams[robotName] = stream;
    console.log(`RTSP stream started for robot ${robotName}`);
}

/**
 * 카메라 서버를 초기화하는 함수
 * @param {Object} req - 요청 객체
 * @param {string} robotName - 로봇 이름
 */

function initializeCameraServer(req, robotName) {
    if (!cameraServer) {
        const cameraConfig = req.app.locals.config.controlling.find(robot => robot[robotName])?.[robotName]?.rtsp;
        const io = req.app.locals.io;
        // console.log(io)
        cameraServer = new CameraServer(cameraConfig, io);
        cameraServer.initialize();

        io.emit("load-finished")
        
    }
}

module.exports = router;
