/*
const express = require('express');
const router = express.Router();

// Detection 
router.get('/', (req, res) => {
  res.sendFile('detection.html', { root: './public/detection' });
});

module.exports = router;

*/

const express = require('express');
const router = express.Router();

// YOLO 영상 페이지 렌더링
router.get('/', (req, res) => {
    res.sendFile('detection.html', { root: 'public/detection' });
});

// Flask YOLO 스트리밍 연동
router.get('/video_feed', (req, res) => {
    res.redirect('http://localhost:5000/video_feed'); // Flask 서버로 리디렉트
});

module.exports = router;
