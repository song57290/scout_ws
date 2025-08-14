/**
 * JSONFileManager 클래스는 주어진 컨테이너에 JSON 데이터를 렌더링하고 저장하는 기능을 제공합니다.
 */
class JSONFileManager {
  /**
   * JSONFileManager의 인스턴스를 생성합니다.
   * @param {string} containerId - JSON 데이터를 렌더링할 HTML 컨테이너의 ID
   * @param {Object} jsonData - 렌더링할 JSON 데이터
   */
  constructor(containerId, jsonData) {
    this.container = document.getElementById(containerId);
    this.jsonData = jsonData;
  }

  /**
   * 주어진 부모 요소에 중첩된 JSON 데이터를 렌더링합니다.
   * @param {HTMLElement} parentElement - JSON 데이터를 렌더링할 부모 요소
   * @param {Object} data - 렌더링할 JSON 데이터
   * @param {string} [keyPath=""] - 현재 데이터의 키 경로
   */
  renderNestedConfig(parentElement, data, keyPath = "") {
    Object.keys(data).forEach((key) => {
      const value = data[key];
      const div = document.createElement("div");
      div.className = "nested";
      if (key === 'map') {
        div.innerHTML = `<label>2D Grid Map:</label>`;
      } else {
        div.innerHTML = `<label>${key}:</label>`;
      }
      parentElement.appendChild(div);
      if (typeof value === "object" && value !== null) {
        this.renderNestedConfig(
          div,
          value,
          keyPath ? `${keyPath}.${key}` : key
        );
      } else {
        const input = document.createElement("input");
        input.type = "text";
        input.value = value;
        input.dataset.keyPath = keyPath ? `${keyPath}.${key}` : key;
        div.appendChild(input);

        if (key.toLowerCase().includes("password")) {
          const toggleButton = document.createElement("button");
          input.type = "password";2020
          toggleButton.type = "button";
          toggleButton.textContent = "보기";
          toggleButton.className = "toggle-password"; 
          toggleButton.addEventListener("click", () => {
            input.type = input.type === "password" ? "text" : "password";
            toggleButton.textContent = input.type === "password" ? "보기" : "숨기기";
          });
          div.appendChild(toggleButton);
        }
      }
    });
  }

  /**
   * JSON 데이터를 HTML 컨테이너에 렌더링합니다.
   */
  renderJSON() {
    this.container.innerHTML = "";
    const rootDiv = document.createElement("div");
    rootDiv.className = "section";
    this.container.appendChild(rootDiv);
    this.renderNestedConfig(rootDiv, this.jsonData);
  }

  /**
   * HTML 입력 요소의 값을 JSON 데이터에 저장하고 서버에 전송합니다.
   */
  saveJSON() {
    const inputs = this.container.querySelectorAll("input");
    inputs.forEach((input) => {
      const keyPath = input.dataset.keyPath.split(".");
      let obj = this.jsonData;
      keyPath.slice(0, -1).forEach((key) => {
        obj = obj[key];
      });
      obj[keyPath[keyPath.length - 1]] = input.value;
    });
    fetch('/api/saveConfig', {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json'
      },
      body: JSON.stringify(this.jsonData)
    })
      .then(response => {
        if (response.ok) {
          this.showCustomAlert("설정 저장 완료");
          return response.text();
        } else {
          throw new Error('설정 저장 실패');
        }
      })
      .then((data) => console.log(data))
      .catch((err) => console.error(err));
  }

  /**
   * 사용자에게 메시지를 표시하는 커스텀 알림을 보여줍니다.
   * @param {string} message - 사용자에게 표시할 메시지
   */
  showCustomAlert(message) {
    const modal = document.getElementById("customAlert");
    const closeButton = modal.querySelector(".close-button");
    modal.querySelector("p").textContent = message;
    modal.style.display = "block";

    closeButton.onclick = function() {
      modal.style.display = "none";
    };
    window.onclick = function(event) {
      if (event.target == modal) {
        modal.style.display = "none";
      }
    };
  }
}

/**
 * 서버로부터 설정 데이터를 가져옵니다.
 * @returns {Promise<Object>} 설정 데이터를 포함하는 Promise 객체
 */
async function getConfig() {
  const response = await fetch("/api/getConfig");
  const data = await response.json();
  const config = data.config || {};
  return config;
}

/**
 * JSONFileManager를 초기화하고 이벤트 리스너를 설정합니다.
 */
async function initializeFileManager() {
  const config = await getConfig();
  const fileManager = new JSONFileManager("configContainer", config);
  fileManager.renderJSON();

  document.getElementById("saveButton").addEventListener("click", (e) => {
    e.preventDefault();
    fileManager.saveJSON();
  });

  document.addEventListener("keydown", (e) => {
    if (e.ctrlKey && e.key === 's') {
      e.preventDefault();
      fileManager.saveJSON();
    }
  });
}

initializeFileManager();
