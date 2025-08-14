/**
 * 브라우저의 전체화면 모드를 토글하는 함수
 * 현재 전체화면이 아닐 경우 전체화면으로 전환하고,
 * 전체화면일 경우 일반 화면으로 복귀합니다.
 * 브라우저 호환성을 위해 vendor prefix를 포함합니다.
 */
function toggleFullscreen() {
    if (!document.fullscreenElement) {
        if (document.documentElement.requestFullscreen) {
            document.documentElement.requestFullscreen();
        } else if (document.documentElement.webkitRequestFullscreen) {
            document.documentElement.webkitRequestFullscreen();
        } else if (document.documentElement.msRequestFullscreen) {
            document.documentElement.msRequestFullscreen();
        }
        document.getElementById("fullscreenButton").textContent = "전체화면 종료";
    } else {
        if (document.exitFullscreen) {
            document.exitFullscreen();
        } else if (document.webkitExitFullscreen) {
            document.webkitExitFullscreen();
        } else if (document.msExitFullscreen) {
            document.msExitFullscreen();
        }
        document.getElementById("fullscreenButton").textContent = "전체화면";
    }
}