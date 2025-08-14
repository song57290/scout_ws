/**
 * 주어진 스크립트와 스타일 시트를 비동기적으로 로드하고,
 * 모든 리소스가 로드된 후 콜백 함수를 호출합니다.
 * 
 * @param {Function} callback - 모든 리소스가 로드된 후 실행할 콜백 함수
 */
function loadResources(callback) {
    const resources = {
        scripts: [
            "./src/utills/config.js",
            "./src/utills/robotButtons.js",
            "./src/utills/cameraToggle.js",
            "./src/utills/mapToggle.js",
            "./src/utills/sidebar.js",
            "./src/utills/fullScreenToggle.js",
            "./src/utills/toggleExplain.js",
            "./src/utills/customAlert.js",
            "./src/cameras/cameraRosDisplayer.js",
            "./src/cameras/cameraRtspDisplayer.js",
            /** "./src/joysticks/joystickController.js", */
            "./src/joysticks/joystickController_add.js",
            "./src/LiDAR/lidar2dDisplayer.js",
            /** "./src/map/map.js", */
            "./src/map/office_map.js",
            "./src/robotStates/robotStatesDisplayer.js",
            "./src/clientSet/clientSet.js",
        ]
    };

    let loadedResources = 0;
    const totalResources = resources.scripts.length;

    /**
     * 각 리소스가 로드될 때마다 호출되며,
     * 모든 리소스가 로드되면 콜백 함수를 실행합니다.
     */
    function resourceLoaded() {
        loadedResources++;
        if (loadedResources === totalResources) {
            console.log("All scripts loaded successfully.");
            callback();
        }
    }

    resources.scripts.forEach(url => {
        const script = document.createElement('script');
        script.src = url;
        script.onload = resourceLoaded;
        script.onerror = (e) => console.error(`Error loading script: ${e.target.src}`);
        document.body.appendChild(script);
    });
}

/**
 * 모든 리소스가 로드된 후 실행할 작업을 정의합니다.
 */
function doSomethingAfterResourcesLoaded() {
    console.log("모든 리소스가 로드 되었습니다.");
}

loadResources(doSomethingAfterResourcesLoaded);
