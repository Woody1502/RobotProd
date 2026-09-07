'use strict';

// ── WebSocket status ──────────────────────────────────────────────────────

const sGp    = document.getElementById('s-gp');
const sSpeed = document.getElementById('s-speed');

function connectWs() {
  const ws = new WebSocket(`ws://${location.host}/ws`);

  ws.onmessage = ({ data }) => {
    const d = JSON.parse(data);
    sGp.textContent    = d.gp_connected ? 'Подключён' : 'Нет связи';
    sSpeed.textContent = (d.speed ?? 0).toFixed(1);
  };

  ws.onclose = () => setTimeout(connectWs, 3000);
}

connectWs();

// ── Camera ────────────────────────────────────────────────────────────────

const cam     = document.getElementById('cam');
const noSig   = document.getElementById('no-signal');

cam.onerror = () => { cam.hidden = true;  noSig.hidden = false; };
cam.onload  = () => { cam.hidden = false; noSig.hidden = true;  };

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
