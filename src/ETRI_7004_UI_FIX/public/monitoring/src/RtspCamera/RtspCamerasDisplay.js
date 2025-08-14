class RtspCamerasDisplay {
  constructor() {
    this.sockets = [];
    this.initializeSockets();
  }

  async initializeSockets() {
    try {
      // 서버에서 CCTV 설정 정보를 가져옴
      const response = await fetch('/api/getConfig');
      if (!response.ok) throw new Error("Failed to fetch configuration");

      const data = await response.json();
      if (!data.config || !Array.isArray(data.config.monitoring)) {
        throw new Error("Invalid configuration data");
      }

      // 서버에서 받은 CCTV 설정을 기반으로 소켓 초기화
      this.sockets = data.config.monitoring.map(({ id }) => ({
        id,
        socket: io(), // 소켓 연결 초기화
      }));

    this.sockets.forEach(({ id, socket }, index) => {
      console.log(`현재 ${index + 1}번째 실행 중: id = ${id}`);

      const imgs = document.getElementsByClassName("cctv-content"); // CCTV ID에 해당하는 img 태그
      const status = document.getElementsByClassName("status"); // 상태 표시 태그
      const ids = document.getElementsByClassName("cctv-title")
      if (!imgs || !status) {
        console.warn(`Missing elements for id: ${id}, index: ${index}`);
        return;
      }

      // 비디오 프레임 데이터를 받아서 화면에 표시
      socket.on(`${id}_videoFrame`, (data) => {
        const base64String = this.arrayBufferToBase64(data);
        imgs[index].src = "data:image/jpeg;base64," + base64String;
        ids[index].innerHTML = id
        status[index].textContent = "Status: Active";
      });

      // 연결 종료 시 상태를 업데이트
      socket.on("disconnect", () => {
        status.textContent = "Status: Offline";
      });
    });

    } catch (error) {
      console.error("Error initializing sockets:", error);
    }
  }

  startAllStreams() {
    fetch('api/startAllMonitoringRTSPStreams', { method: 'POST' })
      .then((response) => response.text())
      .then((data) => console.log(data))
      .catch((error) => console.error("Error starting streams:", error));
  }

  arrayBufferToBase64(buffer) {
    let binary = "";
    const bytes = new Uint8Array(buffer);
    bytes.forEach((byte) => (binary += String.fromCharCode(byte)));
    return window.btoa(binary);
  }
}
