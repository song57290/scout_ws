function loadAdditionalResources(callback) {
    const resources = [
        "./src/RtspCamera/RtspCamerasDisplay.js",
        "./src/sidebar/sidebarToggle.js",
        "./src/clientSet/clientSet.js",
    ];

    let currentIndex = 0;

    function loadNextScript() {
        if (currentIndex >= resources.length) {
            console.log("All resources loaded successfully.");
            callback();
            return;
        }

        const script = document.createElement('script');
        script.src = resources[currentIndex];
        script.onload = () => {
            console.log(`Script loaded: ${resources[currentIndex]}`);
            currentIndex++;
            loadNextScript();
        };
        script.onerror = (e) => console.error(`Error loading script: ${e.target.src}`);
        document.body.appendChild(script);
    }

    loadNextScript();
}

function doSomethingAfterResourcesLoaded() {
    console.log("Now I can do something after all resources are loaded!");
}

loadAdditionalResources(doSomethingAfterResourcesLoaded);
