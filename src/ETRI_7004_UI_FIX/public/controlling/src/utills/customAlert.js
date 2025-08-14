/**
 * 사용자에게 메시지를 표시하는 커스텀 알림을 보여줍니다.
 * @param {string} message - 사용자에게 표시할 메시지
 */
function showCustomAlert(message) {
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