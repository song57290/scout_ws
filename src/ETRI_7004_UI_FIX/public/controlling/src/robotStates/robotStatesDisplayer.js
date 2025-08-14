/**
 * RobotStatesDisplayer 클래스는 ROS를 통해 수신된 배터리 상태 데이터를
 * HTML 요소에 시각적으로 표시합니다.
 */
class RobotStatesDisplayer {

    /**
     * RobotStatesDisplayer의 인스턴스를 생성합니다.
     * @param {Object} ros - ROS 연결 객체
     * @param {string} batteryTopic - 배터리 상태 데이터가 발행되는 ROS 토픽 이름
     * @param {string} batteryId - 배터리 상태를 표시할 HTML 요소의 ID
     */
    constructor(ros, batteryTopic, batteryId) {
        console.log(batteryId);
        this.batteryStatusElement = document.getElementById("robot-state-terminal-battery");
        this.ros = ros;

        this.batteryListener = new ROSLIB.Topic({
            ros: this.ros,
            name: batteryTopic,
            messageType: 'std_msgs/String'
        });

        this.batteryListener.subscribe(this.updateBatteryStatus.bind(this));
    } 

    /**
     * 배터리 상태를 업데이트하고 HTML 요소의 스타일을 변경합니다.
     * @param {Object} status - ROS로부터 수신된 배터리 상태 메시지
     */
    updateBatteryStatus(status) {
        console.log(status.data);
        const batteryPercentage = Number(status.data);
        this.batteryStatusElement.style.width = `${batteryPercentage}%`;

        if (batteryPercentage > 75) {
            this.batteryStatusElement.style.backgroundColor = 'lightgreen';
            this.batteryStatusElement.style.color = 'black';
        } else if (batteryPercentage > 50) {
            this.batteryStatusElement.style.backgroundColor = 'yellow';
            this.batteryStatusElement.style.color = 'black';
        } else if (batteryPercentage > 25) {
            this.batteryStatusElement.style.backgroundColor = 'orange';
            this.batteryStatusElement.style.color = 'white';
        } else {
            this.batteryStatusElement.style.backgroundColor = 'red';
            this.batteryStatusElement.style.color = 'white';
        }

        this.batteryStatusElement.textContent = `${batteryPercentage}%`;
    }

}