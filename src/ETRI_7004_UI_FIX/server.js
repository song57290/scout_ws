const express = require('express');
const http = require('http');
const socketIo = require('socket.io');
const fs = require('fs').promises;
const { createProxyMiddleware } = require('http-proxy-middleware');  // Flask 프록시 추가
const app = express();
const PORT = 3000;
let clients = 0;
let config = {};
let io;

// ✅ Flask YOLO 감지 서버 주소 (IP 기반으로 지정)
const FLASK_SERVER_URL = 'http://192.168.0.24:5000';

/**
 * config.json 파일을 비동기적으로 읽어와서 JSON 객체로 반환합니다.
 * @returns {Promise<Object>} - config.json 파일의 내용을 파싱한 객체
 */
async function loadConfig() {
    try {
        const data = await fs.readFile('config.json', 'utf8');
        return JSON.parse(data);
    } catch (err) {
        console.error('config.json 파일을 읽는 중 오류가 발생했습니다:', err);
        return {};
    }
}

/**
 * 주어진 설정 데이터를 config.json 파일에 비동기적으로 저장합니다.
 * @param {Object} configData - 저장할 설정 데이터 객체
 * @throws {Error} - 파일 저장 중 발생한 오류
 */
async function saveConfig(configData) {
    try {
        await fs.writeFile('config.json', JSON.stringify(configData, null, 2), 'utf8');
        console.log('설정이 성공적으로 저장되었습니다.');
    } catch (err) {
        console.error('config.json 파일 저장 중 오류 발생:', err);
        throw err;
    }
}

// ✅ 서버 및 socket.io 설정
const server = http.createServer(app);
io = socketIo(server);
app.use(express.json());
app.get('/', (req, res) => {
    res.redirect('/controlling');
});

/**
 * 설정 정보를 가져오는 API 엔드포인트
 * 클라이언트에게 현재 설정 정보를 JSON 형식으로 반환합니다.
 */
app.get('/api/getConfig', async (req, res) => {
    try {
        const config = app.locals.config;
        if (config) {
            res.json({ config });
        } else {
            res.status(404).json({ error: 'config not found' });
        }
    } catch (error) {
        console.error('Error fetching config:', error);
        res.status(500).json({ error: 'Internal server error' });
    }
});

/**
 * 설정 정보를 저장하는 API 엔드포인트
 * 클라이언트로부터 받은 설정 데이터를 config.json 파일에 저장하고, 모든 클라이언트에 업데이트를 알립니다.
 */
app.post("/api/saveConfig", async (req, res) => {
    const configData = req.body;
    console.log(configData);

    try {
        await saveConfig(configData);
        app.locals.config = configData;
        io.emit('configUpdate', configData);
        res.status(200).send("설정이 성공적으로 저장되었습니다.");
    } catch (err) {
        res.status(500).send("설정 저장 중 오류가 발생했습니다.");
    }
});

/**
 * config.json 파일의 변경을 감지하고, 변경 시 설정을 다시 로드하여 모든 클라이언트에 업데이트를 알립니다.
 */
fs.watch('config.json', async (eventType) => {
    if (eventType === 'change') {
        try {
            console.log('config.json 파일이 변경되었습니다. 새로 고침 중...');
            config = await loadConfig();
            app.locals.config = config;
            io.emit('configUpdate', config);
        } catch (err) {
            console.error('config.json 파일 읽기 오류:', err);
        }
    }
});

/**
 * 📌 Flask YOLO 감지 서버 프록시 설정 (새로운 기능 추가)
 */
app.use('/detection/video_feed', createProxyMiddleware({
    target: FLASK_SERVER_URL,  // Flask YOLO 서버 주소
    changeOrigin: true,
    logLevel: 'debug',  // 디버깅 로그 활성화
    pathRewrite: (path, req) => {
        console.log(`🔄 프록시 요청 변환: ${path} → /video_feed`);
        return '/video_feed';  // 무조건 /video_feed로 변경
    },
    onProxyReq: (proxyReq, req, res) => {
        console.log(`🔄 프록시 요청: ${req.url} → ${FLASK_SERVER_URL}/video_feed`);
    },
    onError: (err, req, res) => {
        console.error(`❌ 프록시 에러 발생: ${err.message}`);
        res.status(500).send('Flask 서버 연결 실패');
    }
}));

/**
 * 📌 기존 GUI 페이지 라우팅 유지
 */
async function startServer() {
    try {
        config = await loadConfig();
        app.use(express.static('public'));
        app.locals.config = config;
        app.locals.io = io;

        // 기존 라우트 유지
        app.use('/monitoring', require('./routes/monitoring/monitoring.js'));
        app.use('/controlling', require('./routes/controlling/controlling.js'));
        app.use('/setting', require('./routes/setting/setting.js'));

        // 📌 Detection 메뉴 추가
        app.use('/detection', require('./routes/detection/detection.js'));

        server.listen(PORT, () => {
            console.log(`🚀 서버 실행 중: http://localhost:${PORT}`);
            console.log(`🔍 YOLO 감지 스트리밍: http://localhost:${PORT}/detection/video_feed`);
        });
    } catch (error) {
        console.error('❌ 서버 실행 중 오류 발생:', error);
    }
}

startServer();
