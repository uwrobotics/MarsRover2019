/* Assumes vis.js has already been loaded */

const reader = new FileReader();
reader.onload = ({ target: { result } }) => {
  try {
    const input = result.trim();
    if (input === '') throw new Error('File is empty');
    processInput(input);
  } catch(err) {
    showError(err);
    console.error(err);
  }
}

const eDropbox = document.querySelector('#dropbox');
const eFileInput = document.querySelector('#fileInput');
const edForm = document.querySelector('#dForm');
const eNFrame = document.querySelector('#nFrame');
const eError = document.querySelector('#error');
const eCanvas = document.querySelector('#canvas');
const edCanvas = document.querySelector('#dCanvas');
const ctx = eCanvas.getContext('2d');

const width = canvas.width;
const height = canvas.height;

const originX = width / 2;
const originY = height / 2;

eDropbox.addEventListener('click', onClick, false);
eDropbox.addEventListener('drop', onDrop, false);
eDropbox.addEventListener('dragenter', stopPropagation, false);
eDropbox.addEventListener('dragover', stopPropagation, false);

function onClick(e) {
  eFileInput.click();
  stopPropagation(e);
}

function onDrop(e) {
  stopPropagation(e);
  handleFiles(e.dataTransfer.files)
}

function stopPropagation(e) {
  e.stopPropagation();
  e.preventDefault();
}

function switchToGraph() {
  edForm.style.display = 'none';
  edCanvas.style.display = 'block';
}

function showError(text) {
  eError.style.visibility = 'visible';
  eError.innerHTML = text;
}

function hideError() {
  eError.style.visibility = 'hidden';
  eError.innerHTML = '';
}

function handleFiles(files) {
  hideError();
  const file = files[0];

  const parts = file.name.split('.');
  const fileType = parts[parts.length - 1];

  if (fileType !== 'csv') {
    showError(new Error('File must be .csv'));
  } else {
    reader.readAsText(file);
  }
}