/**
 * CameraDisplayer 클래스는 ROS 카메라 토픽을 구독하여 카메라 이미지를 HTML 이미지 요소에 표시합니다.
 */
class CameraDisplayer {
    /**
     * @param {Object} ros - ROS 연결 객체
     * @param {string} topicName - 구독할 ROS 카메라 토픽 이름
     * @param {string} imageElementId - 이미지를 표시할 HTML 이미지 요소의 ID
     */
    constructor(ros, topicName, imageElementId) {
        this.ros = ros;
        this.topicName = topicName;
        this.imageElement = document.getElementById(imageElementId);
        this.messageType = "sensor_msgs/CompressedImage";

        this.cameraImageTopic = new ROSLIB.Topic({
            ros: this.ros,
            name: this.topicName,
            messageType: this.messageType
        });

        this.startSubscription();
    }

    /**
     * ROS 토픽 구독을 시작합니다.
     */
    startSubscription() {
        this.cameraImageTopic.subscribe((message) => {
            this.updateImage(message.data);
        });
    }

    /**
     * ROS 토픽 구독을 중지합니다.
     */
    stopSubscription() {
        this.cameraImageTopic.unsubscribe();
    }

    /**
     * 수신한 이미지 데이터를 HTML 이미지 요소에 업데이트합니다.
     * @param {string} base64Data - base64로 인코딩된 이미지 데이터
     */
    updateImage(base64Data) {
        const base64Image = `data:image/jpeg;base64,${base64Data}`;
        this.imageElement.src = base64Image;
    }
}

