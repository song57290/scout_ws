/**
 * 추가 스크립트를 로드합니다.
 * @param {Function} callback - 모든 스크립트가 로드된 후 호출될 콜백 함수
 */
function loadAdditionalScripts(callback) {
    const scriptUrls = [
        "src/components/fileManager.js",
        "src/utills/toggleExplain.js",
        
    ];

    const styleUrls = [
        "style/base/base.css",
        "style/layout/config.css",
        "style/components/content.css",
        "style/components/header.css",
        "style/components/sidebar.css",
        "style/layout/help.css"
    ];

    let loadedScripts = 0;
    const totalScripts = scriptUrls.length + styleUrls.length;

    scriptUrls.forEach(url => {
        const script = document.createElement('script');
        script.src = url;
        script.onload = () => {
            console.log(`${url} loaded successfully.`);
            loadedScripts++;
            if (loadedScripts === totalScripts) {
                console.log("All scripts and styles loaded successfully.");
                callback();
            }
        };
        script.onerror = (e) => console.error(`Error loading script: ${e.target.src}`);
        document.body.appendChild(script);
    });

    styleUrls.forEach(url => {
        const link = document.createElement('link');
        link.rel = 'stylesheet';
        link.href = url;
        link.onload = () => {
            console.log(`${url} loaded successfully.`);
            loadedScripts++;
            if (loadedScripts === totalScripts) {
                console.log("All scripts and styles loaded successfully.");
                callback();
            }
        };
        link.onerror = (e) => console.error(`Error loading style: ${e.target.href}`);
        document.head.appendChild(link);
    });
}

/**
 * 모든 스크립트가 로드된 후 실행할 작업을 수행합니다.
 */
function doSomethingAfterScriptsLoaded() {
    console.log("Now I can do something after all scripts are loaded!");
}

loadAdditionalScripts(doSomethingAfterScriptsLoaded);