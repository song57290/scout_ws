const express = require('express');
const path = require('path');
const router = express.Router();

/**
 * GET / - 설정 페이지를 반환합니다.
 */
router.get('/', (req, res) => {
    res.sendFile(path.join(__dirname, '../../public', 'setting', 'setting.html'));
});

module.exports = router;
