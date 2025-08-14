const sidebar = document.getElementById('sidebar');
const content = document.getElementById('content');
let sidebarVisible = false;

/**
 * 사이드바를 전환합니다.
 */
function toggleSidebar() {
    sidebarVisible = !sidebarVisible;
    updateSidebarDisplay();
    updateToggleButton();
}

/**
 * 사이드바의 표시 상태를 업데이트합니다.
 */
function updateSidebarDisplay() {
    if (sidebarVisible) {
        sidebar.classList.remove('hidden');
        content.classList.remove('expanded');
    } else {
        sidebar.classList.add('hidden');
        content.classList.add('expanded');
    }
}

/**
 * 토글 버튼의 상태를 업데이트합니다.
 */
function updateToggleButton() {
    toggleButton.innerHTML = sidebarVisible 
        ? '<span class="arrow">&lt;</span>'
        : '<span class="arrow">&gt;</span>';
}

toggleSidebar();