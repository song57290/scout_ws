let isRosCameraActive = true;

/**
 * 카메라 뷰를 전환합니다.
 */
function toggleCameraView() {
    const rosContainer = document.getElementById("camera-container-ros");
    const rtspContainer = document.getElementById("camera-container-rtsp");
    const toggleButton = document.getElementById("toggleCameraButton");

    if (isRosCameraActive) {
        showRTSPCameraView(rosContainer, rtspContainer, toggleButton);
    } else {
        showROSCameraView(rosContainer, rtspContainer, toggleButton);
    }

    isRosCameraActive = !isRosCameraActive;
}

/**
 * RTSP 카메라 뷰를 표시합니다.
 * @param {HTMLElement} rosContainer - ROS 카메라 컨테이너
 * @param {HTMLElement} rtspContainer - RTSP 카메라 컨테이너
 * @param {HTMLElement} toggleButton - 전환 버튼
 */
function showRTSPCameraView(rosContainer, rtspContainer, toggleButton) {
    rosContainer.classList.add("mini");
    rosContainer.classList.remove("fullscreen");
    rtspContainer.classList.add("fullscreen");
    rtspContainer.classList.remove("mini");
    toggleButton.textContent = "ROS 카메라 확장";
}

/**
 * ROS 카메라 뷰를 표시합니다.
 * @param {HTMLElement} rosContainer - ROS 카메라 컨테이너
 * @param {HTMLElement} rtspContainer - RTSP 카메라 컨테이너
 * @param {HTMLElement} toggleButton - 전환 버튼
 */
function showROSCameraView(rosContainer, rtspContainer, toggleButton) {
    rosContainer.classList.add("fullscreen");
    rosContainer.classList.remove("mini");
    rtspContainer.classList.add("mini");
    rtspContainer.classList.remove("fullscreen");
    toggleButton.textContent = "RTSP 카메라 확장";
}