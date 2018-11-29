// Visualization for the 3R arm IK module

const XY_DELIM = ',';
const POINTS_DELIM = ' ';
const INTERVAL_MS = 1;

function clear() {
  ctx.fillStyle = 'black';
  ctx.fillRect(0, 0, width, height);
}

function plot(x, y, scale, size = 5, color = 'white') {
  const originX = width / 2;
  const originY = height / 2;

  const plotX = originX + x * scale;
  const plotY = originY - y * scale;

  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.arc(plotX, plotY, size, 0, 2 * Math.PI);
  ctx.fill();
}

function visualize(frames, extrema) {
  switchToGraph();
  clear();
  
  const maxDistFromOriginX = Math.max(Math.abs(extrema.minX), Math.abs(extrema.maxX));
  const maxDistFromOriginY = Math.max(Math.abs(extrema.minY), Math.abs(extrema.maxY));
  const maxDistFromOrigin = Math.max(maxDistFromOriginX, maxDistFromOriginY);
  let availableDistFromOrigin = (maxDistFromOriginX > maxDistFromOriginY ? width / 2 : height / 2) * 0.9;

  const scale = availableDistFromOrigin / maxDistFromOrigin;

  let completedOnce = false;
  let i = 0;
  const getNextIndex = () => ++i % frames.length;

  // TODO i got this far, nothing tested lmao

  const plotFrame = () => {
    clear();
    eNFrame.innerHTML = i;

    const frame = frames[i];
    frame.forEach(p => plot(p.x, p.y, scale));

    const dist = (p1, p2) => Math.sqrt(
      (p2.x - p1.x) ** 2 + (p2.y - p1.y) ** 2
    );

    console.log(
      dist(frame[1], frame[0]),
      dist(frame[2], frame[1]),
      dist(frame[3], frame[2]),
      );

    // const toIndex = completedOnce ? frames.length : i;
    // for (let index = 0; index <= toIndex; index++) {
    //   const frame = frames[index];
    //   frame.forEach(p => {
    //     if (index === toIndex) plot(p.x, p,y, scale);
    //     else plot(p.x, p.y, scale, 2, 'gray');
    //   });
    // }
  };

  setInterval(
    () => {
      i = getNextIndex();
      if (i == frames.length - 1) completedOnce = true;
      plotFrame();
    },
    INTERVAL_MS
  );
}

function processInput(csv) {
  const lines = csv.split('\n');

  let extrema = null;
    
  const frames = lines.map((line, iLine) => {
    const points = line.split(POINTS_DELIM);
    return points.map(point => {
      const [xStr, yStr, ...rest] = point.split(XY_DELIM);

      if (rest.length) throw new Error(`Expected x,y coordinates, got more than two values on line ${iLine}`);

      const [x, y] = [parseFloat(xStr), parseFloat(yStr)];

      if (isNaN(x) || isNaN(y)) throw new Error(`Found non-number entry on line ${iLine}`);

      if (!extrema) {
        extrema = {
          minX: x,
          maxX: x,
          minY: y,
          maxY: y,
        };
      }
      else if (x < extrema.minX) extrema.minX = x;
      else if (x > extrema.maxX) extrema.maxX = x;
      else if (y < extrema.minY) extrema.minY = y;
      else if (y > extrema.maxY) extrema.maxY = y;

      return { x, y };
    });
  });

  visualize(frames, extrema);
}

/* SETUP */

const eNFrame = document.querySelector('#nFrame');
const eTerxtArea = document.querySelector('#textarea');
const eError = document.querySelector('#error');
const eCanvas = document.querySelector('#canvas');
const edForm = document.querySelector('#dForm');
const edCanvas = document.querySelector('#dCanvas');
const ctx = eCanvas.getContext('2d');

const width = canvas.width;
const height = canvas.height;

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

function onSubmit(e) {
  e.preventDefault();
  hideError();

  try {
    const input = eTerxtArea.value.trim();
    if (input !== '') processInput(input);
  } catch(err) {
    showError(err);
    console.error(err);
  }
}