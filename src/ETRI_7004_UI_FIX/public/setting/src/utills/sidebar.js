/**
 * 사이드바와 콘텐츠 영역을 제어하는 요소들을 가져옵니다.
 */
const sidebar = document.getElementById("sidebar");
const content = document.getElementById("content");
const arrow = document.getElementById("arrow");
let sidebarVisible = false;

/**
 * 사이드바의 가시성을 토글하는 함수입니다.
 * 사이드바가 보이면 숨기고, 숨겨져 있으면 보이게 합니다.
 */
function toggleSidebar() {
    sidebarVisible = !sidebarVisible;
    if (sidebarVisible) {
        sidebar.classList.remove("hidden");
        content.classList.remove("expanded");
        toggleButton.innerHTML = '<span class="arrow">&lt;</span>';
    } else {
        sidebar.classList.add("hidden");
        content.classList.add("expanded");
        toggleButton.innerHTML = '<span class="arrow">&gt;</span>';
    }
}

toggleSidebar();