'use strict';

// ── WebSocket status ──────────────────────────────────────────────────────

const sGp    = document.getElementById('s-gp');
const sSpeed = document.getElementById('s-speed');
const sAp    = document.getElementById('s-ap');

function connectWs() {
  const ws = new WebSocket(`ws://${location.host}/ws`);

  ws.onmessage = ({ data }) => {
    const d = JSON.parse(data);
    sGp.textContent    = d.gp_connected ? 'Подключён' : 'Нет связи';
    sSpeed.textContent = (d.speed ?? 0).toFixed(1);
    if (d.autopilot !== undefined) {
      sAp.textContent = d.autopilot ? 'ВКЛ' : 'ВЫКЛ';
      // sync button state if status came from elsewhere
      if (apOn !== d.autopilot) {
        apOn = d.autopilot;
        btnAp.textContent = apOn ? '⏹ Остановить автопилот' : '▶ Включить автопилот';
        btnAp.classList.toggle('active', apOn);
      }
    }
  };

  ws.onclose = () => setTimeout(connectWs, 3000);
}

connectWs();

// ── Camera snapshot polling ───────────────────────────────────────────────

const cam    = document.getElementById('cam');
const noSig  = document.getElementById('no-signal');

let camEndpoint = '/snapshot';
let camTimer    = null;

function pollFrame() {
  const url = camEndpoint + '?t=' + Date.now();
  const img  = new Image();
  img.onload = () => {
    cam.src       = img.src;
    cam.hidden    = false;
    noSig.hidden  = true;
  };
  img.onerror = () => {
    cam.hidden   = true;
    noSig.hidden = false;
  };
  img.src = url;
}

function startCam() {
  if (camTimer) clearInterval(camTimer);
  pollFrame();
  camTimer = setInterval(pollFrame, 100); // 10 fps
}

startCam();

const btnCamRaw  = document.getElementById('btn-cam-raw');
const btnCamMask = document.getElementById('btn-cam-mask');

btnCamRaw.addEventListener('click', () => {
  camEndpoint = '/snapshot';
  btnCamRaw.classList.add('active');
  btnCamMask.classList.remove('active');
});

btnCamMask.addEventListener('click', () => {
  camEndpoint = '/snapshot/mask';
  btnCamMask.classList.add('active');
  btnCamRaw.classList.remove('active');
});

// ── Hold-to-run attachment buttons ───────────────────────────────────────

document.querySelectorAll('.btn.hold').forEach(btn => {
  const post = (v) => fetch(`/cmd/${btn.dataset.dev}/${v}`, { method: 'POST' });

  btn.addEventListener('pointerdown', (e) => {
    e.preventDefault();
    btn.classList.add('pressed');
    post(btn.dataset.on);
  });

  ['pointerup', 'pointercancel', 'pointerleave'].forEach(ev => {
    btn.addEventListener(ev, () => {
      btn.classList.remove('pressed');
      post(0);
    });
  });

  btn.addEventListener('contextmenu', (e) => e.preventDefault());
});

// ── Gamepad toggle ────────────────────────────────────────────────────────

let gpEnabled = true;
const btnGp = document.getElementById('btn-gp');

btnGp.addEventListener('click', () => {
  gpEnabled = !gpEnabled;
  fetch(`/gamepad/enabled/${gpEnabled ? 1 : 0}`, { method: 'POST' });
  btnGp.textContent = gpEnabled ? 'Геймпад ВКЛ' : 'Геймпад ВЫКЛ';
  btnGp.classList.toggle('active', gpEnabled);
});

// ── Emergency stop ────────────────────────────────────────────────────────

document.getElementById('btn-stop').addEventListener('click', () => {
  fetch('/cmd/estop', { method: 'POST' });
});

// ── Separator toggle ──────────────────────────────────────────────────────

let sepOn = false;
const btnSep = document.getElementById('btn-sep');

btnSep.addEventListener('click', () => {
  sepOn = !sepOn;
  fetch(`/cmd/separator/${sepOn ? 2 : 0}`, { method: 'POST' });
  btnSep.textContent = sepOn ? '◉ Выключить' : '◉ Включить';
  btnSep.classList.toggle('active', sepOn);
});

// ── Autopilot toggle ──────────────────────────────────────────────────────

let apOn = false;
const btnAp = document.getElementById('btn-ap');

btnAp.addEventListener('click', () => {
  apOn = !apOn;
  fetch(`/autopilot/enabled/${apOn ? 1 : 0}`, { method: 'POST' });
  btnAp.textContent = apOn ? '⏹ Остановить автопилот' : '▶ Включить автопилот';
  btnAp.classList.toggle('active', apOn);
});
