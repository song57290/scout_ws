/**
 * SetClient 클래스는 로봇의 설정을 관리하고 ROS 연결을 처리합니다.
 * 로봇의 이름, IP 주소, 센서 및 디스플레이 설정을 초기화하고,
 * ROS 서버와의 연결을 설정 및 종료하는 기능을 제공합니다.
 */
class SetClient {
    /**
     * SetClient의 인스턴스를 생성합니다.
     * 로컬 스토리지에서 선택된 로봇의 이름을 가져오고,
     * 로봇의 설정을 초기화합니다.
     */
    constructor() {
        this.robotName = localStorage.getItem('selectedRobot');
        if (!this.robotName) {
            alert('로봇을 선택해 주세요.');
            return;
        }
        this.config;
        this.ip;
        this.lidar2d;
        this.lidar3d;
        this.cmdVel;
        this.robotPose;
        this.rtsp;
        this.ros;
        this.batteryTopic;
        this.map;
        this.io = io()
    }

    /**
     * RTSP 스트림을 시작합니다.
     * 선택된 로봇의 이름을 서버에 전송하여 스트림을 요청합니다.
     */
    async startRTSPStream() {
        try {
            const response = await fetch('api/startRTSPStream', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({ robotName: this.robotName }),
            });

            if (response.ok) {
                console.log('RTSP stream started successfully.');
            } else {
                const errorData = await response.json();
                console.error('Error starting RTSP stream:', errorData.error);
            }
        } catch (error) {
            console.error('Error starting RTSP stream:', error);
        }
    }

    /**
     * 다양한 디스플레이 요소를 설정합니다.
     * 카메라, LiDAR, 지도 및 배터리 상태를 표시하는 객체를 초기화합니다.
     */
    setDisplayer() {
        const imageElementId = 'camera-image';
        new CameraDisplayer(this.ros, this.camera, imageElementId);

        const rtspImageElementId = 'camera-image-rtsp';
        new CameraRtspDisplayer(rtspImageElementId);

        const canvasId = 'lidar-canvas';
        new LidarDisplayer(this.ros, this.lidar2d, canvasId);

        const mapCanvasId = 'map-canvas';
        const Map = new MapVisualizer(this.ros, this.robotPose, mapCanvasId, this.map);
        const batteryId = 'robot-state-terminal-battery';
        new RobotStatesDisplayer(this.ros, this.batteryTopic, batteryId);
    }

    /**
     * 로봇의 설정을 선택합니다.
     * 주어진 설정 객체에서 선택된 로봇의 정보를 가져와 초기화합니다.
     * @param {Array} config - 로봇 설정 배열
     */
    selectRobot(config) {
        const selectedObject = config.find(obj => Object.keys(obj)[0] === this.robotName)[this.robotName];
        this.ip = selectedObject.ip;
        this.lidar2d = selectedObject.lidar2d;
        this.lidar3d = selectedObject.lidar3d;
        this.cmdVel = selectedObject.cmdVel;
        this.camera = selectedObject.camera;
        this.robotPose = selectedObject.robotPose;
        this.rtsp = selectedObject.rtsp;
        this.map = selectedObject.map;
        this.batteryTopic = selectedObject.battery;
    }

    /**
     * ROS 연결을 설정합니다.
     * ROS 서버와의 웹소켓 연결을 설정하고, 연결 상태를 콘솔에 출력합니다.
     */
    async ROSSet() {
        this.ros = new ROSLIB.Ros({
            url: `ws://${this.ip}:9090`
        });

        this.ros.on('connection', () => {
            console.log('Connected to ROS!');
        });

        this.ros.on('error', (error) => {
            console.error('Error connecting to ROS:', error);
        });

        this.ros.on('close', () => {
            console.log('Disconnected from ROS');
        });
    }

    /**
     * SetClient를 초기화합니다.
     * 로봇 설정을 가져오고, ROS 연결을 설정하며, 디스플레이와 조이스틱 컨트롤러를 초기화합니다.
     */
    async init() {
        if (!this.robotName) return;
        try {
            const response = await fetch('/api/getConfig')
            const data = await response.json();
            console.log(data);
            if (data) {
                this.config = data.config.controlling;
                this.selectRobot(this.config);

                await this.ROSSet();

                this.setDisplayer();

                await this.startRTSPStream();

                this.joystickController = new JoystickController(this.ros, this.cmdVel);
                
                this.io.on("load-finished",()=>{
                    showCustomAlert("시스템 가동 준비가 완료되었습니다. 확인을 눌러 새로고침 해주세요")

                })

            } else {
                console.error('system init failed');
            }
        } catch (error) {
            console.error('system init errored:', error);
        }
    }

    /**
     * ROS 연결을 종료합니다.
     * 현재 ROS 연결이 존재하면 종료하고, 콘솔에 종료 메시지를 출력합니다.
     */
    terminate() {
        if (this.ros) {
            this.ros.close();
            console.log('ROS connection terminated.');
        } else {
            console.log('No ROS connection to terminate.');
        }
    }
}

const client = new SetClient();
client.init();
