/**
 * MapVisualizer 클래스는 ROS를 통해 수신된 위치 데이터를 기반으로
 * HTML 캔버스에 지도를 시각화하고 로봇의 위치를 표시합니다.
 */
class MapVisualizer {
    /**
     * MapVisualizer의 인스턴스를 생성합니다.
     * @param {Object} ros - ROS 연결 객체
     * @param {string} amclPoseTopic - 로봇의 위치 데이터가 발행되는 ROS 토픽 이름
     * @param {string} mapCanvasId - 지도를 표시할 HTML 캔버스 요소의 ID
     * @param {Object} mapConfig - 지도 설정 객체 (이미지 경로, 해상도, 크기, 방향 포함)
     */
    constructor(ros, amclPoseTopic, mapCanvasId, mapConfig) {
        this.canvas = document.getElementById(mapCanvasId);
        this.ctx = this.canvas.getContext("2d");
        this.mapImage = new Image();
        this.mapImage.src = mapConfig.imagePath;
        this.robotIcon = new Image();
        this.robotIcon.src = './maps/location.png';
        this.ros = ros;

        this.amclPoseListener = new ROSLIB.Topic({
            ros: this.ros,
            name: amclPoseTopic,
            messageType: 'geometry_msgs/PoseWithCovarianceStamped'
        });

        this.zoomLevel = 2;
        this.rotationAngle = 0;
        this.offsetX = 0;
        this.offsetY = 0;
        this.isDragging = false;
        this.startX = 0;
        this.startY = 0;
        this.robotX = 0;
        this.robotY = 0;

        this.mapResolution = mapConfig.resolution;
        this.realMapWidth = mapConfig.width * this.mapResolution;
        this.realMapHeight = mapConfig.height * this.mapResolution;

        this.mapOrientation = { x: Number(mapConfig.orientation.x), y: Number(mapConfig.orientation.y), z: 0, w: 0 };
        this.correctionValue = { x: this.realMapWidth / 2 + this.mapOrientation.x, y: -(this.realMapHeight / 2 + this.mapOrientation.y), z: 0, w: 0 };
        this.initializeEventListeners();
        this.amclPoseListener.subscribe(this.handleAMCLPose.bind(this));
        this.setupROSConnection();
    }

    /**
     * ROS 연결을 설정하고 연결 상태를 콘솔에 출력합니다.
     */
    setupROSConnection() {
        this.ros.on('connection', () => console.log('Connected to ROS server'));
        this.ros.on('error', (error) => console.error('Error connecting to ROS server:', error));
        this.ros.on('close', () => console.log('Connection to ROS server closed'));
    }

    /**
     * AMCL 포즈 메시지를 처리하여 로봇의 위치와 방향을 업데이트합니다.
     * @param {Object} message - ROS로부터 수신된 포즈 메시지
     */
    handleAMCLPose(message) {
        this.robotX = message.pose.pose.position.x - this.correctionValue.x;
        this.robotY = -(message.pose.pose.position.y) - this.correctionValue.y;

        const { x, y, z, w } = message.pose.pose.orientation;
        let angle = -this.quaternionToEuler(x, y, z, w) * (180 / Math.PI);

        this.rotationAngle = ((angle % 360) + 360) % 360;
        console.log(this.rotationAngle);

        this.offsetX = -(this.robotX * this.zoomLevel / this.mapResolution);
        this.offsetY = -(this.robotY * this.zoomLevel / this.mapResolution);
        this.updateMinimap();
    }

    /**
     * 쿼터니언을 오일러 각도로 변환합니다.
     * @param {number} x - 쿼터니언 x 값
     * @param {number} y - 쿼터니언 y 값
     * @param {number} z - 쿼터니언 z 값
     * @param {number} w - 쿼터니언 w 값
     * @returns {number} 오일러 각도 (라디안)
     */
    quaternionToEuler(x, y, z, w) {
        // yaw (z-axis rotation) 계산
        const siny_cosp = 2 * (w * z + x * y);
        const cosy_cosp = 1 - 2 * (y * y + z * z);
        return Math.atan2(siny_cosp, cosy_cosp);
    }

    /**
     * 캔버스에 대한 이벤트 리스너를 초기화합니다.
     */
    initializeEventListeners() {
        this.canvas.addEventListener('mousedown', (event) => {
            this.isDragging = true;
            this.startX = event.clientX - this.offsetX;
            this.startY = event.clientY - this.offsetY;
        });

        this.canvas.addEventListener('mousemove', (event) => {
            if (this.isDragging) {
                this.offsetX = event.clientX - this.startX;
                this.offsetY = event.clientY - this.startY;
                this.updateMinimap();
            }
        });

        this.canvas.addEventListener('mouseup', () => {
            this.isDragging = false;
        });

        this.canvas.addEventListener('mouseout', () => {
            this.isDragging = false;
        });

        this.canvas.addEventListener('wheel', (event) => {
            event.preventDefault();
            const zoomFactor = event.deltaY > 0 ? 0.9 : 1.1;
            this.zoomLevel *= zoomFactor;
            this.zoomLevel = Math.min(Math.max(this.zoomLevel, 0.1), 10);
            this.updateMinimap();
        });
     
    }

    /**
     * 배경 이미지를 캔버스에 그립니다.
     */
    drawBackgroundImage() {
        const scaleFactor = this.zoomLevel;
        const scaledWidth = this.mapImage.width * scaleFactor;
        const scaledHeight = this.mapImage.height * scaleFactor;
        const x = (this.canvas.width - scaledWidth) / 2 + this.offsetX;
        const y = (this.canvas.height - scaledHeight) / 2 + this.offsetY;

        this.ctx.clearRect(0, 0, this.canvas.width, this.canvas.height);
        this.ctx.drawImage(this.mapImage, x, y, scaledWidth, scaledHeight);
    }

    /**
     * 로봇의 방향을 나타내는 아이콘을 캔버스에 그립니다.
     */
    drawDirectionIndicator() {
        const centerX = (this.canvas.width / 2) + this.offsetX + (this.robotX * this.zoomLevel / this.mapResolution);
        const centerY = (this.canvas.height / 2) + this.offsetY + (this.robotY * this.zoomLevel / this.mapResolution);

        if (this.robotIcon.complete) {
            this.ctx.save();
            this.ctx.translate(centerX, centerY);
            this.ctx.rotate(((this.rotationAngle * Math.PI) / 180));
            
            const robotWidth = 6 * this.zoomLevel;  
            const robotHeight = 4.4 * this.zoomLevel;
            this.ctx.beginPath();
            this.ctx.strokeStyle = 'blue';
            this.ctx.lineWidth = 1 * this.zoomLevel;
            this.ctx.rect(-robotWidth/2, -robotHeight/2, robotWidth, robotHeight);
            this.ctx.stroke();

                   
            const arrowLength = 7 * this.zoomLevel;
            this.ctx.lineWidth = 2 * this.zoomLevel;
            
            this.ctx.beginPath();
            this.ctx.strokeStyle = 'red';
            this.ctx.moveTo(0, 0);
            this.ctx.lineTo(arrowLength, 0);
            this.ctx.stroke();
            
            this.ctx.beginPath();
            this.ctx.strokeStyle = 'green';
            this.ctx.moveTo(0, 0);
            this.ctx.lineTo(0, -arrowLength);
            this.ctx.stroke();

            this.ctx.restore();
        } else {
            console.log('Robot icon not loaded yet');
        }
    }

    /**
     * 미니맵을 업데이트하여 현재 상태를 반영합니다.
     */
    updateMinimap() {
        this.drawBackgroundImage();
        this.drawDirectionIndicator();
    }

    /**
     * MapVisualizer를 초기화하고 캔버스 크기를 설정합니다.
     */
    initialize() {
        this.canvas.width = 800;
        this.canvas.height = 800;

        this.mapImage.onload = () => {
            this.updateMinimap();
        };
    }
}
