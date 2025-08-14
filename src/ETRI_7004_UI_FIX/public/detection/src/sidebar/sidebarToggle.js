const sidebar = document.getElementById('sidebar');
const content = document.getElementById('content');
const arrow = document.getElementById('arrow');
let sidebarVisible = false;

function toggleSidebar() {
    sidebarVisible = !sidebarVisible;
    if (sidebarVisible) {
        sidebar.classList.remove('hidden');
        content.classList.remove('expanded'); 
        toggleButton.innerHTML = '<span class="arrow">&lt;</span>';
    } else {
        sidebar.classList.add('hidden');
        content.classList.add('expanded');
        toggleButton.innerHTML = '<span class="arrow">&gt;</span>';
    }
}


toggleSidebar()