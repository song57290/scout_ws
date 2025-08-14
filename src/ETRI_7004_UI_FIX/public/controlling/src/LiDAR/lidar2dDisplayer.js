class LidarDisplayer {
    constructor(ros, topicName, canvasId) {
        this.lidarCanvas = document.getElementById(canvasId);
        this.lidarCtx = this.lidarCanvas.getContext('2d');
        this.topicName = topicName;
        this.messageType = "sensor_msgs/LaserScan";
        this.ros = ros;

        this.lidarTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: this.topicName,
            messageType: this.messageType,
        });

        this.subscribeToLidar();
    }

    getColorByDistance(distance) {
        const minDistance = 0;
        const maxDistance = 5;
        const ratio = (distance - minDistance) / (maxDistance - minDistance);

        const red = Math.max(0, Math.min(255, (1 - ratio) * 255));
        const green = Math.max(0, Math.min(255, ratio * 255));
        const blue = Math.max(0, Math.min(255, ratio * 200));

        return `rgb(${red}, ${green}, ${blue})`;
    }

    drawVehicle() {
        const vehicleWidth = 35;
        const vehicleHeight = 50;
        const centerX = this.lidarCanvas.width / 2;
        const centerY = this.lidarCanvas.height / 1.1;
        
        this.lidarCtx.fillStyle = "rgba(255, 0, 0, 0.5)";
        this.lidarCtx.fillRect(
            centerX - vehicleWidth / 2,
            centerY - vehicleHeight / 2,
            vehicleWidth,
            vehicleHeight
        );
    }

    subscribeToLidar() {
        this.lidarTopic.subscribe((message) => {
            const ranges = message.ranges;
            const angleMin = message.angle_min;
            const angleIncrement = message.angle_increment;

            this.lidarCtx.clearRect(0, 0, this.lidarCanvas.width, this.lidarCanvas.height);
            this.drawVehicle();

            ranges.forEach((distance, index) => {
                if (distance < 4) {
                    const angle = angleMin + index * angleIncrement + Math.PI * 1.5;

                    const x = this.lidarCanvas.width / 2 - distance * Math.cos(angle) * 70;
                    const y = this.lidarCanvas.height / 1.1 + distance * Math.sin(angle) * 70;

                    const color = this.getColorByDistance(distance);

                    this.lidarCtx.beginPath();
                    this.lidarCtx.arc(x, y, 2, 0, 2 * Math.PI);
                    this.lidarCtx.fillStyle = color;
                    this.lidarCtx.fill();
                }
            });
        });
    }
}

