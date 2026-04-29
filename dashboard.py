#!/usr/bin/env python3
"""
ALAN Ground Station Dashboard
Usage:  python3 dashboard.py <port> [baud]
        python3 dashboard.py /dev/tty.usbmodem1234
Requires: pip install flask pyserial
Then open http://localhost:8080
"""

import re
import sys
import time
import json
import threading
import serial
from flask import Flask, Response, render_template_string, request

app = Flask(__name__)

# shared state, updated by serial thread
state = {"angle": 0, "leds": 0, "seq": 0, "updated": "waiting...", "raw": ""}
listeners   = []
listeners_lock = threading.Lock()
cmd_queue   = []   # commands posted from browser, sent by serial thread

PORT = sys.argv[1] if len(sys.argv) > 1 else None
BAUD = int(sys.argv[2]) if len(sys.argv) > 2 else 115200


def parse_line(line):
    m = re.search(r'STATUS,(-?\d+),(\d+)', line)
    if not m:
        return None
    angle = int(m.group(1))
    leds  = int(m.group(2))
    seq = 0
    ms = re.search(r'seq=(\d+)', line)
    if ms: seq = int(ms.group(1))
    return angle, leds, seq


def push_update():
    data = "data: " + json.dumps(state) + "\n\n"
    with listeners_lock:
        for q in listeners:
            q.append(data)


def serial_thread():
    if not PORT:
        return
    while True:
        try:
            with serial.Serial(PORT, BAUD, timeout=0.1) as ser:
                print(f"[serial] connected to {PORT}")
                while True:
                    # send any queued commands
                    if cmd_queue:
                        ser.write(cmd_queue.pop(0).encode())

                    raw = ser.readline().decode('ascii', errors='ignore').strip()
                    if not raw:
                        continue
                    state['raw'] = raw
                    result = parse_line(raw)
                    if result:
                        state['angle']   = result[0]
                        state['leds']    = result[1]
                        state['seq']     = result[2]
                        state['updated'] = time.strftime('%H:%M:%S')
                    push_update()
        except serial.SerialException as e:
            print(f"[serial] {e} — retrying in 2s")
            time.sleep(2)


@app.route('/')
def index():
    return render_template_string(HTML)


@app.route('/events')
def events():
    q = []
    with listeners_lock:
        listeners.append(q)

    def stream():
        # send current state immediately on connect
        yield "data: " + json.dumps(state) + "\n\n"
        try:
            while True:
                if q:
                    yield q.pop(0)
                else:
                    time.sleep(0.05)
        except GeneratorExit:
            pass
        finally:
            with listeners_lock:
                listeners.remove(q)

    return Response(stream(), mimetype='text/event-stream')


@app.route('/cmd', methods=['POST'])
def cmd():
    key = request.json.get('key', '')
    if key in 'qeadzczygrYRG':
        cmd_queue.append(key)
    return {'ok': True}


HTML = '''<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<title>ALAN Dashboard</title>
<style>
  body { font-family: sans-serif; max-width: 500px; margin: 40px auto; padding: 0 16px; }
  h1 { font-size: 1.2rem; margin-bottom: 24px; }
  h3 { font-size: 0.8rem; color: #666; margin-bottom: 6px; }
  .card { border: 1px solid #ccc; padding: 16px; margin-bottom: 16px; }
  .angle-label { text-align: center; font-weight: bold; margin-top: 6px; }
  .leds { display: flex; gap: 16px; }
  .led { display: flex; flex-direction: column; align-items: center; gap: 4px; font-size: 0.8rem; }
  .led-dot { width: 24px; height: 24px; border-radius: 50%; background: #ddd; border: 1px solid #aaa; }
  .led-dot.on-g { background: green; }
  .led-dot.on-r { background: red; }
  .led-dot.on-y { background: gold; }
  .meta { display: flex; gap: 24px; }
  .meta-item { font-size: 0.9rem; }
  .controls { display: grid; grid-template-columns: repeat(3, 1fr); gap: 6px; margin-bottom: 8px; }
  .led-btns { display: flex; gap: 6px; }
  .led-btns button { flex: 1; }
  button { padding: 8px; cursor: pointer; font-size: 0.85rem; }
  .status-bar { font-size: 0.75rem; color: #666; margin-top: 12px; }
</style>
</head>
<body>
<h1>ALAN GROUND STATION</h1>

<div class="card">
  <div class="label">RUDDER ANGLE</div>
  <svg id="rudder-svg" viewBox="-60 -60 120 120" width="160" height="160" style="display:block;margin:0 auto;">
    <circle cx="0" cy="0" r="50" fill="none" stroke="#ccc" stroke-width="2"/>
    <line x1="0" y1="0" x2="0" y2="-48" stroke="#ccc" stroke-width="1" stroke-dasharray="3,3"/>
    <line id="rudder-needle" x1="0" y1="8" x2="0" y2="-44" stroke="#333" stroke-width="3" stroke-linecap="round"/>
    <circle cx="0" cy="0" r="5" fill="#333"/>
  </svg>
  <div class="angle-label" id="angle-label">+0 deg</div>
</div>

<div class="card">
  <div class="label">LEDs</div>
  <div class="leds">
    <div class="led"><div class="led-dot" id="led-g"></div>GREEN</div>
    <div class="led"><div class="led-dot" id="led-r"></div>RED</div>
    <div class="led"><div class="led-dot" id="led-y"></div>YELLOW</div>
  </div>
</div>

<div class="card">
  <div class="label">LINK</div>
  <div class="meta">
    <div class="meta-item"><span class="label">SEQ </span><span class="meta-val" id="seq">--</span></div>
    <div class="meta-item"><span class="label">UPDATED </span><span class="meta-val" id="updated">--</span></div>
  </div>
</div>

<div class="controls">
  <button class="ctrl-btn" data-key="q">Q  +20 deg</button>
  <div></div>
  <button class="ctrl-btn" data-key="e">E  -20 deg</button>
  <button class="ctrl-btn" data-key="a">A  +10 deg</button>
  <div></div>
  <button class="ctrl-btn" data-key="d">D  -10 deg</button>
  <button class="ctrl-btn" data-key="z">Z  +5 deg</button>
  <div></div>
  <button class="ctrl-btn" data-key="c">C  -5 deg</button>
</div>
<br>
<div class="led-btns">
  <button class="ctrl-btn" data-key="g">G  GREEN LED</button>
  <button class="ctrl-btn" data-key="r">R  RED LED</button>
  <button class="ctrl-btn" data-key="y">Y  YELLOW LED</button>
</div>

<div class="status-bar" id="status">connecting...</div>
<div class="status-bar" id="raw" style="margin-top:6px; color:#3fb950;"></div>

<script>
  function send(key) {
    fetch('/cmd', {
      method: 'POST',
      headers: {'Content-Type': 'application/json'},
      body: JSON.stringify({key})
    });
  }

  document.querySelectorAll('.ctrl-btn').forEach(btn => {
    btn.addEventListener('click', () => send(btn.dataset.key));
  });

  document.addEventListener('keydown', e => {
    if ('qeadzczygrYRG'.includes(e.key)) send(e.key);
  });

  function update(angle, leds, seq, updated) {
    const needle = document.getElementById('rudder-needle');
    needle.setAttribute('transform', 'rotate(' + angle + ')');
    document.getElementById('angle-label').textContent =
      (angle >= 0 ? '+' : '') + angle + ' deg';

    document.getElementById('led-g').className =
      'led-dot' + (leds & 1 ? ' on-g' : '');
    document.getElementById('led-r').className =
      'led-dot' + (leds & 2 ? ' on-r' : '');
    document.getElementById('led-y').className =
      'led-dot' + (leds & 4 ? ' on-y' : '');

    document.getElementById('seq').textContent     = seq;
    document.getElementById('updated').textContent = updated;
  }

  const es = new EventSource('/events');
  es.onmessage = e => {
    const d = JSON.parse(e.data);
    update(d.angle, d.leds, d.seq, d.updated);
    document.getElementById('raw').textContent = d.raw || '';
    document.getElementById('status').textContent = 'live';
  };
  es.onerror = () => {
    document.getElementById('status').textContent = 'disconnected — retrying...';
  };
</script>
</body>
</html>'''


if __name__ == '__main__':
    if not PORT:
        print("Usage: python3 dashboard.py <port> [baud]")
        sys.exit(1)
    t = threading.Thread(target=serial_thread, daemon=True)
    t.start()
    app.run(host='0.0.0.0', port=8080, threaded=True)
