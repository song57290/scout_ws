/**
 * JoystickController 클래스는 게임패드를 사용하여 로봇의 이동과 카메라 제어를 수행합니다.
 * @param {Object} ros - ROS 연결 객체
 * @param {string} cmdVel - 로봇의 속도 명령을 발행할 ROS 토픽 이름
 * @param {Object} onvifCamera - ONVIF 카메라 객체
 */
class JoystickController {
    constructor(ros, cmdVel, onvifCamera) {
        this.ros = ros;
        this.cmdVel = cmdVel;
        this.onvifCamera = onvifCamera;
        this.maxLinearSpeed = 1/2;
        this.maxAngularSpeed = 3.14/4;
        this.preCameraMove = { pan: 0, tilt: 0 };
        this.init();
        this.socket = io();
        this.connectToCamera();
        this.lastSentTime = 0;
        this.messageInterval = 100;
        this.totalZoom = 0;
        // PTZ 버튼 이벤트 리스너 등록
        this.initPTZButtons();
    }

    /**
     * ONVIF 카메라에 연결을 시도합니다.
     */
    connectToCamera() {
        this.socket.emit('connection');
        if (this.onvifCamera) {
            this.onvifCamera.connect((err) => {
                if (err) {
                    console.error('ONVIF 카메라 연결 실패:', err);
                } else {
                    console.log('ONVIF 카메라 연결 성공');
                }
            });
        }
    }

    /**
     * 게임패드 연결 및 해제 이벤트를 초기화합니다.
     */
    init() {
        window.addEventListener("gamepadconnected", (event) => {
            console.log("Gamepad connected:", event.gamepad);
            this.connectToCamera();
            requestAnimationFrame(this.updateGamepadStatus.bind(this));
        });

        window.addEventListener("gamepaddisconnected", (event) => {
            console.log("Gamepad disconnected:", event.gamepad);
        });
    }

    /**
     * 게임패드 상태를 업데이트하고 로봇 및 카메라 제어 명령을 발행합니다.
     */
    updateGamepadStatus() {
        let twist;
        const gamepads = navigator.getGamepads();
        if (!gamepads) return;

        const gamepad = gamepads[0];
        if (gamepad.buttons[3].pressed) {
            this.zoomControll("zoomIn");
            console.log("카메라 줌 인 명령");
        }
        if (gamepad.buttons[1].pressed) {
            this.zoomControll("zoomOut");
            console.log("카메라 줌 아웃 명령");
        }
        if (gamepad) {
            this.connectToCamera();
            const currentTime = Date.now();
            if (currentTime - this.lastSentTime < this.messageInterval) {
                requestAnimationFrame(this.updateGamepadStatus.bind(this));
                return;
            }
            let vx = parseFloat((gamepad.axes[0] * this.maxAngularSpeed));
            let vy = parseFloat((-gamepad.axes[1] * this.maxLinearSpeed));

            if (gamepad.buttons[4].pressed) {
                console.log("왼쪽 조이스틱 버튼이 눌린 상태에서 조이스틱 이동");
                twist = new ROSLIB.Message({
                    linear: { x: vy, y: -vx, z: 0 },
                    angular: { x: 0, y: 0, z: 0 },
                });
            } else if (gamepad.buttons[6].pressed) {
                twist = new ROSLIB.Message({
                    linear: { x: 0, y: 0, z: 0 },
                    angular: { x: 0, y: 0, z: 0 },
                });
            } else {
                twist = new ROSLIB.Message({
                    linear: { x: vy, y: 0, z: 0 },
                    angular: { x: 0, y: 0, z: -vx },
                });
            }
            const cmdVelPub = new ROSLIB.Topic({
                ros: this.ros,
                name: this.cmdVel,
                messageType: "geometry_msgs/Twist",
            });
            cmdVelPub.publish(twist);

            const rightStickX = gamepad.axes[2];
            const rightStickY = gamepad.axes[3];

            if (gamepad.buttons[5].pressed) {
                this.socket.emit('zeroPointCamera');
                console.log("카메라 초기화");
            }

            const magnitude = Math.sqrt(rightStickX * rightStickX + rightStickY * rightStickY);
            const speedFactor = magnitude > 1 ? 1 : magnitude;

            if (speedFactor > 0.1) {
                let panIncrement = 0;
                let tiltIncrement = 0;

                if (Math.abs(rightStickX) > Math.abs(rightStickY)) {
                    panIncrement = rightStickX * speedFactor;
                } else {
                    tiltIncrement = -rightStickY * speedFactor;
                }

                console.log("Pan Increment:", panIncrement, "Tilt Increment:", tiltIncrement);

                const relativeMove = {
                    x: panIncrement / 9,
                    y: tiltIncrement/9,
                    zoom: 0
                };

                console.log("카메라 상대 이동 명령:", relativeMove);
                this.socket.emit('moveCamera', relativeMove);
            }

            this.lastSentTime = currentTime;
        }

        requestAnimationFrame(this.updateGamepadStatus.bind(this));
    }

    /**
     * PTZ 제어 버튼 이벤트 리스너 등록
     */
    initPTZButtons() {
        const btnUp = document.getElementById('ptz-up');
        const btnDown = document.getElementById('ptz-down');
        const btnLeft = document.getElementById('ptz-left');
        const btnRight = document.getElementById('ptz-right');
        const btnZoomIn = document.getElementById('ptz-zoom-in');
        const btnZoomOut = document.getElementById('ptz-zoom-out');
        const btnHome = document.getElementById('ptz-home');
        if (btnUp) btnUp.onclick = () => this.moveCamera({x: 0, y: 0.1, zoom: 0});
        if (btnDown) btnDown.onclick = () => this.moveCamera({x: 0, y: -0.1, zoom: 0});
        if (btnLeft) btnLeft.onclick = () => this.moveCamera({x: -0.1, y: 0, zoom: 0});
        if (btnRight) btnRight.onclick = () => this.moveCamera({x: 0.1, y: 0, zoom: 0});
        if (btnZoomIn) btnZoomIn.onclick = () => this.moveCamera({x: 0, y: 0, zoom: 0.1});
        if (btnZoomOut) btnZoomOut.onclick = () => this.moveCamera({x: 0, y: 0, zoom: -0.1});
        if (btnHome) btnHome.onclick = () => this.zeroPointCamera();
    }

    /**
     * PTZ 카메라 이동 명령 (화살표/줌 버튼용)
     */
    moveCamera(relativeMove) {
        this.socket.emit('moveCamera', relativeMove);
    }

    /**
     * PTZ 카메라 홈(초기화) 명령
     */
    zeroPointCamera() {
        this.socket.emit('zeroPointCamera');
    }
}
