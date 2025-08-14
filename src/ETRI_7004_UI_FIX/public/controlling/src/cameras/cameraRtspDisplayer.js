/**
 * CameraRtspDisplayer 클래스는 RTSP 스트림을 HTML 캔버스 요소에 표시합니다.
 */
class CameraRtspDisplayer {
    /**
     * @param {string} rtspImageElementId - RTSP 이미지를 표시할 HTML 캔버스 요소의 ID
     */
    constructor(rtspImageElementId) {
        this.socket = io();
        this.canvas = document.getElementById(rtspImageElementId);
        this.ctx = this.canvas.getContext('2d');
        this.selectedRobot = localStorage.getItem('selectedRobot');
        this.initializeSocket();
    }

    /**
     * 소켓 연결을 초기화하고 비디오 프레임을 수신합니다.
     */
    initializeSocket() {
        this.socket.on(`${this.selectedRobot}_videoFrame`, (arrayBuffer) => {
            this.handleVideoFrame(arrayBuffer);
        });
    }

    /**
     * 수신한 비디오 프레임을 처리하여 캔버스에 표시합니다.
     * @param {ArrayBuffer} arrayBuffer - 수신한 비디오 프레임 데이터
     */
    handleVideoFrame(arrayBuffer) {
        const uint8Array = new Uint8Array(arrayBuffer);
        const blob = new Blob([uint8Array], { type: 'image/jpeg' });
        const url = URL.createObjectURL(blob);

        const img = new Image();
        img.onload = () => {
            this.updateCanvas(img);
        };

        img.src = url;
    }

    /**
     * 캔버스를 업데이트하여 이미지를 그립니다.
     * @param {HTMLImageElement} img - 캔버스에 그릴 이미지
     */
    updateCanvas(img) {
        this.canvas.width = img.width;
        this.canvas.height = img.height;
        this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
        this.ctx.drawImage(img, 0, 0);

        try {
            const imageData = this.ctx.getImageData(0, 0, img.width, img.height);
        } catch (error) {
            console.error("ImageData 추출 오류:", error);
        }
    }
}
