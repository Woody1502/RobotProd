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

// ── Camera ────────────────────────────────────────────────────────────────

const cam     = document.getElementById('cam');
const noSig   = document.getElementById('no-signal');

cam.onerror = () => { cam.hidden = true;  noSig.hidden = false; };
cam.onload  = () => { cam.hidden = false; noSig.hidden = true;  };

const btnCamRaw  = document.getElementById('btn-cam-raw');
const btnCamMask = document.getElementById('btn-cam-mask');

function switchCam(src, activeBtn, inactiveBtn) {
  cam.src = src;
  activeBtn.classList.add('active');
  inactiveBtn.classList.remove('active');
}

btnCamRaw .addEventListener('click', () => switchCam('/video',      btnCamRaw,  btnCamMask));
btnCamMask.addEventListener('click', () => switchCam('/video/mask', btnCamMask, btnCamRaw));

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
