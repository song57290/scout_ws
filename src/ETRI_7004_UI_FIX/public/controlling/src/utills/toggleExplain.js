let toggle = 0

function openExplainingPopup() {
    if (toggle%2 ==0){
        document.getElementById("Explaing").style.display = "flex"; 
        
    }else{
        document.getElementById("Explaing").style.display = "none"; 
    }
    toggle++;
}


function closeExplainingPopup() {
    document.getElementById("Explaing").style.display = "none"; 
    toggle++;
}


document.getElementById("close-btn").addEventListener("click", closeExplainingPopup);
