class SetClient {
    constructor() {
        this.robotName = localStorage.getItem('selectedRobot');
        if (!this.robotName) {
            alert('로봇을 선택해 주세요.');
            return;
        }
    }

    TurnOnRtsp(){
        const rtspCamerasDisplay = new RtspCamerasDisplay();
        rtspCamerasDisplay.startAllStreams();
    }

    async init() {
        await this.TurnOnRtsp();
    }
}
const client = new SetClient();
client.init();