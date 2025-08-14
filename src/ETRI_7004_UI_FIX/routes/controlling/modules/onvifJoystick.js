const { Cam } = require('onvif');

/**
 * CameraServer 클래스는 ONVIF 프로토콜을 사용하여 카메라와의 연결을 관리하고,
 * 클라이언트로부터의 명령을 처리하여 카메라를 제어합니다.
 */
class CameraServer {
    /**
     * @param {Object} cameraConfig - 카메라 설정 객체
     * @param {Object} io - 소켓 IO 인스턴스
     */
    constructor(cameraConfig, io) {
        this.cameraHost = cameraConfig.onvif_host;
        this.username = cameraConfig.onvif_username;
        this.password = cameraConfig.onvif_password;
        this.port = cameraConfig.onvif_port;
        this.io = io;
        this.camera = null;
    }

    /**
     * 카메라에 연결을 시도합니다.
     * @param {Function} callback - 연결 결과를 처리할 콜백 함수
     */
    connectCamera(callback) {
        this.camera = new Cam({
            hostname: this.cameraHost,
            username: this.username,
            password: this.password,
            port: this.port
        }, (err) => {
            if (err) {
                console.log('Connection Failed for ' + this.cameraHost);
                return callback(err);
            }
            console.log('CONNECTED TO CAMERA');
            callback(null);
        });
    }

    /**
     * 클라이언트로부터의 카메라 이동 명령을 처리합니다.
     */
    handleCameraMovement() {
        this.io.on('connection', (socket) => {
            console.log('Client connected');

            socket.on('moveCamera', (position) => {
                /**const { x, y, zoom } = position;
		**/
                let { x, y, zoom } = position;
                x=-x;
                y=-y;

                if (Math.abs(x) > 1 || Math.abs(y) > 1 || Math.abs(zoom) > 1) {
                    console.error('Invalid move values:', x, y, zoom);
                    return;
                }

                console.log("카메라 이동 명령:", x, y, zoom);
                this.camera.relativeMove({ x, y, zoom }, (err) => {
                    if (err) {
                        console.error('Camera move error:', err);
                    } else {
                        console.log(`Camera moved to x: ${x}, y: ${y}, zoom: ${zoom}`);
                    }
                });
            });

            socket.on('zeroPointCamera', () => {
                console.log("카메라 초기화 명령 수신");
                this.camera.absoluteMove({ x: 0, y: 0, zoom: 0 }, (err) => {
                    if (err) {
                        console.error('Camera move error:', err);
                    } else {
                        console.log(`Camera moved to x: 0, y: 0 zoom: 0`);
                    }
                });
            });
        });
    }

    /**
     * 서버의 상태를 확인하고 로그를 출력합니다.
     */
    checkServerStatus() {
        if (this.camera) {
            console.log('Server is open and connected to the camera.');
        } else {
            console.log('Server is not connected to the camera.');
        }
    }

    /**
     * 카메라 서버를 초기화하고 연결을 설정합니다.
     */
    initialize() {
        this.connectCamera((err) => {
            if (err) {
                console.log(err);
            } else {
                this.checkServerStatus();
                this.handleCameraMovement();
            }
        });
    }
}

module.exports = { CameraServer };
