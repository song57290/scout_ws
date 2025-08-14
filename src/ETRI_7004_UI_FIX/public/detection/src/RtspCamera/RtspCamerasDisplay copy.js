class RtspCamerasDisplay {
  constructor() {
    this.sockets = [
      { id: "cctv1", socket: io("") },
      { id: "cctv2", socket: io("") },
      { id: "cctv3", socket: io("") },
      { id: "cctv4", socket: io("") },
    ];
    this.initializeSockets();
  }

  initializeSockets() {
    this.sockets.forEach(({ id, socket }) => {
      const img = document.getElementById(id);
      const status = document.getElementById(`status${id.slice(-1)}`);

      socket.on(`${id}_videoFrame`, (data) => {
        const base64String = this.arrayBufferToBase64(data);
        img.src = "data:image/jpeg;base64," + base64String;
        status.textContent = "Status: Active";
      });

      socket.on("disconnect", () => {
        status.textContent = "Status: Offline";
      });
    });
  }

  startAllStreams() {
    fetch("api/startAllMonitoringRTSPStreams", {
      method: "POST",
    })
      .then((response) => response.text())
      .then((data) => {
        console.log(data);
      })
      .catch((error) => {
        console.error("Error starting streams:", error);
      });
  }

  arrayBufferToBase64(buffer) {
    let binary = "";
    const bytes = new Uint8Array(buffer);
    const len = bytes.byteLength;
    for (let i = 0; i < len; i++) {
      binary += String.fromCharCode(bytes[i]);
    }
    return window.btoa(binary);
  }
}
