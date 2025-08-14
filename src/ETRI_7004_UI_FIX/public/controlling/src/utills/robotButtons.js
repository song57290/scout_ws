/**
 * 로봇 버튼을 추가합니다.
 * @param {Object} config - 설정 객체
 */
function addRobotButtons(config) {
    const ul = document.getElementById('robot-list');
    const selectedRobot = localStorage.getItem('selectedRobot');

    config.controlling.forEach(robot => {
        const robotName = Object.keys(robot)[0];
        const li = createRobotButton(robotName, selectedRobot);
        ul.appendChild(li);
    });
}

/**
 * 로봇 버튼을 생성합니다.
 * @param {string} robotName - 로봇 이름
 * @param {string} selectedRobot - 선택된 로봇 이름
 * @returns {HTMLButtonElement} 생성된 버튼 요소
 */
function createRobotButton(robotName, selectedRobot) {
    const li = document.createElement('button');
    const p = document.createElement('p');
    p.textContent = robotName;
    li.appendChild(p);

    if (robotName === selectedRobot) {
        li.disabled = true;
        li.style.backgroundColor = '#1099cf62';
    }

    li.addEventListener('click', () => handleRobotButtonClick(robotName));
    return li;
}

/**
 * 로봇 버튼 클릭 시 처리합니다.
 * @param {string} robotName - 로봇 이름
 */
function handleRobotButtonClick(robotName) {
    if (client) {
        client.terminate();
    }
    localStorage.setItem('selectedRobot', robotName);
    window.location.reload();
}

getConfig().then(config => {
    addRobotButtons(config);
});