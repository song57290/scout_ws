/**
 * 서버로부터 설정을 가져옵니다.
 * @returns {Promise<Object>} 설정 객체를 반환합니다.
 */
async function getConfig() {
    const response = await fetch("/api/getConfig");
    const data = await response.json();
    return data.config || {};
}