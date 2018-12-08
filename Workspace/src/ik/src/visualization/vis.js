// Visualization for the 3R arm IK module

const XY_DELIM = ',';
const POINTS_DELIM = ' ';
const INTERVAL_MS = 1;

function clear() {
  ctx.fillStyle = 'black';
  ctx.fillRect(0, 0, width, height);
}

function plotPoint(x, y, scale, radius = 8, color = 'white') {
  const plotX = originX + x * scale;
  const plotY = originY - y * scale;

  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.arc(plotX, plotY, radius, 0, 2 * Math.PI);
  ctx.fill();
}

function plotLine(x1, y1, x2, y2, scale) {
  const width = 5;
  const color = 'white';

  const plotX1 = originX + x1 * scale;
  const plotY1 = originY - y1 * scale;

  const plotX2 = originX + x2 * scale;
  const plotY2 = originY - y2 * scale;

  ctx.strokeStyle = color;
  ctx.lineWidth = width;
  ctx.beginPath();
  ctx.moveTo(plotX1, plotY1);
  ctx.lineTo(plotX2, plotY2);
  ctx.stroke();
}

function visualize(frames, extrema) {
  switchToGraph();
  clear();
  
  const maxDistFromOriginX = Math.max(Math.abs(extrema.minX), Math.abs(extrema.maxX));
  const maxDistFromOriginY = Math.max(Math.abs(extrema.minY), Math.abs(extrema.maxY));
  const maxDistFromOrigin = Math.max(maxDistFromOriginX, maxDistFromOriginY);
  let availableDistFromOrigin = (maxDistFromOriginX > maxDistFromOriginY ? width / 2 : height / 2) * 0.9;

  const scale = availableDistFromOrigin / maxDistFromOrigin;

  let currentIndex = 0;
  const getNextIndex = () => ++currentIndex % frames.length;

  const plotFrame = frameIndex => {
    clear();
    eNFrame.innerHTML = frameIndex;

    const points = frames[frameIndex];
    
    let prevPoint;
    for (let i = 0; i < points.length; i++) {
      const point = points[i];
      if (prevPoint) plotLine(prevPoint.x, prevPoint.y, point.x, point.y, scale);
      prevPoint = point;
    }

    for (let i = 0; i < points.length; i++) {
      const { x, y } = points[i];
      plotPoint(x, y, scale, 5 * 1.2, 'white');
      plotPoint(x, y, scale, 5, 'tomato');
    }
  };

  setInterval(
    () => {
      currentIndex = getNextIndex();
      plotFrame(currentIndex);
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

const originX = width / 2;
const originY = height / 2;

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