const series = new Map();
const latest = new Map();
const colors = ["#69d2e7", "#f38630", "#a7dbd8", "#e0e4cc", "#fa6900", "#c7f464", "#ff6b6b"];
let packetCount = 0;
let lastPacketMs = 0;

const canvas = document.getElementById("telemetry-canvas");
const ctx = canvas.getContext("2d");
const packetInfo = document.getElementById("packet-info");
const latestValues = document.getElementById("latest-values");
const connectionPill = document.getElementById("connection-pill");
const videoFeed = document.getElementById("video-feed");
const videoPlaceholder = document.getElementById("video-placeholder");

function resizeCanvas() {
  const rect = canvas.getBoundingClientRect();
  const scale = window.devicePixelRatio || 1;
  canvas.width = Math.max(1, Math.floor(rect.width * scale));
  canvas.height = Math.max(1, Math.floor(rect.height * scale));
  ctx.setTransform(scale, 0, 0, scale, 0, 0);
}

function addSample(sample) {
  packetCount += 1;
  lastPacketMs = Date.now();
  latest.set(sample.name, sample);
  if (!series.has(sample.name)) {
    series.set(sample.name, []);
  }
  const points = series.get(sample.name);
  points.push(sample);
  const newest = sample.timestamp_ms;
  while (points.length > 0 && newest - points[0].timestamp_ms > 30000) {
    points.shift();
  }
  while (points.length > 600) {
    points.shift();
  }
}

function selectedSeries() {
  return Array.from(series.keys())
    .filter((name) => {
      const sample = latest.get(name);
      return sample && sample.status === "ok" && name !== "heartbeat";
    })
    .slice(0, 7);
}

function drawPlot() {
  const width = canvas.clientWidth;
  const height = canvas.clientHeight;
  ctx.clearRect(0, 0, width, height);

  ctx.fillStyle = "#0d0f12";
  ctx.fillRect(0, 0, width, height);
  ctx.strokeStyle = "#242932";
  ctx.lineWidth = 1;
  for (let x = 0; x < width; x += 80) {
    ctx.beginPath();
    ctx.moveTo(x, 0);
    ctx.lineTo(x, height);
    ctx.stroke();
  }
  for (let y = 0; y < height; y += 40) {
    ctx.beginPath();
    ctx.moveTo(0, y);
    ctx.lineTo(width, y);
    ctx.stroke();
  }

  const names = selectedSeries();
  if (names.length === 0) {
    ctx.fillStyle = "#6f7885";
    ctx.font = "14px -apple-system, BlinkMacSystemFont, Segoe UI, sans-serif";
    ctx.fillText("waiting for numeric telemetry", 16, 28);
    return;
  }

  const now = Date.now();
  const windowMs = 30000;
  names.forEach((name, index) => {
    const points = series.get(name).filter((p) => p.status === "ok");
    if (points.length < 2) {
      return;
    }
    let min = Math.min(...points.map((p) => p.value));
    let max = Math.max(...points.map((p) => p.value));
    if (min === max) {
      min -= 1;
      max += 1;
    }

    ctx.strokeStyle = colors[index % colors.length];
    ctx.lineWidth = 2;
    ctx.beginPath();
    points.forEach((point, pointIndex) => {
      const x = width - ((now - point.received_ms) / windowMs) * width;
      const y = height - ((point.value - min) / (max - min)) * (height - 22) - 10;
      if (pointIndex === 0) {
        ctx.moveTo(x, y);
      } else {
        ctx.lineTo(x, y);
      }
    });
    ctx.stroke();

    ctx.fillStyle = colors[index % colors.length];
    ctx.font = "12px -apple-system, BlinkMacSystemFont, Segoe UI, sans-serif";
    ctx.fillText(name, 14 + (index % 4) * 210, 18 + Math.floor(index / 4) * 18);
  });
}

function renderLatest() {
  packetInfo.textContent = `${packetCount} packets`;
  const ageMs = lastPacketMs === 0 ? Infinity : Date.now() - lastPacketMs;
  connectionPill.textContent = ageMs < 1500 ? `telemetry: live (${ageMs} ms)` : "telemetry: waiting";
  connectionPill.style.color = ageMs < 1500 ? "#b8f7c1" : "#f5c16c";

  latestValues.innerHTML = "";
  Array.from(latest.keys()).slice(0, 10).forEach((name) => {
    const sample = latest.get(name);
    const chip = document.createElement("span");
    chip.className = "value-chip";
    chip.dataset.status = sample.status;
    chip.textContent = `${name}: ${sample.status === "ok" ? sample.value.toFixed(2) : sample.status} ${sample.unit}`;
    latestValues.appendChild(chip);
  });
}

function animationLoop() {
  drawPlot();
  renderLatest();
  requestAnimationFrame(animationLoop);
}

fetch("/config.json")
  .then((response) => response.json())
  .then((config) => {
    if (config.video_url) {
      videoFeed.src = config.video_url;
      videoFeed.style.display = "block";
      videoPlaceholder.style.display = "none";
    }
  });

const events = new EventSource("/events");
events.onmessage = (event) => {
  addSample(JSON.parse(event.data));
};

window.addEventListener("resize", resizeCanvas);
resizeCanvas();
animationLoop();
