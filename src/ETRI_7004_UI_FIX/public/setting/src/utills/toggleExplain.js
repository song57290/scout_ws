let toggle = 0;

function openExplainingPopup() {
  if (toggle % 2 === 0) {
    document.getElementById("Explaing").style.display = "flex"; 
  } else {
    document.getElementById("Explaing").style.display = "none"; 
  }
  toggle++;
}


function closeExplainingPopup() {
  document.getElementById("Explaing").style.display = "none"; 
  toggle++;
}


document.getElementById("close-btn").addEventListener("click", closeExplainingPopup);


const images = [
  `./image/ex${1}.png`,
  `./image/ex${2}.png`,
  `./image/ex${3}.png`
];

let currentIndex = 0;


const imageElement = document.getElementById("explaning-image");
const nextButton = document.getElementById("next-btn");
const prevButton = document.getElementById("prev-btn");
imageElement.src = images[currentIndex];


prevButton.disabled = true; 


nextButton.addEventListener("click", () => {
  if (currentIndex < images.length - 1) {
    currentIndex++;
    imageElement.src = images[currentIndex];
  }


  updateButtonVisibility();
});


prevButton.addEventListener("click", () => {
  if (currentIndex > 0) {
    currentIndex--;
    imageElement.src = images[currentIndex];
  }
  updateButtonVisibility();
});


function updateButtonVisibility() {
  prevButton.disabled = currentIndex === 0; 
  nextButton.disabled = currentIndex === images.length - 1; 
}
