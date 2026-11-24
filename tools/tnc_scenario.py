#!/usr/bin/env python3
"""tnc_scenario.py - drive two real `ultra_tnc --sim-audio` stations over `ota_simulator serve`
the way a messaging client does (command port + data port), and time what the host sees.

Each scenario runs on a fresh channel and fresh TNC processes. Every check is from the host's
side: the lines on the command port, the bytes on the data port, and when they arrived.

  tools/tnc_scenario.py --scenario short --channel awgn --snr-db 10 --out /tmp/tnc_short
  tools/tnc_scenario.py --scenario all --channel good --snr-db 5 --seed 43 --out /tmp/tnc_all

Writes <out>/<scenario>/{summary.txt,events.log,alpha.log,bravo.log,serve.log} and prints one
RESULT line per scenario. Exit code 0 only when every scenario passed.
"""
import argparse
import os
import random
import shutil
import signal
import socket
import subprocess
import sys
import threading
import time

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
BUILD = os.path.join(ROOT, "build")


class Fail(Exception):
    pass


def free_port_pair():
    while True:
        s = socket.socket()
        s.bind(("127.0.0.1", 0))
        port = s.getsockname()[1]
        s.close()
        if port >= 65000:
            continue
        t = socket.socket()
        try:
            t.bind(("127.0.0.1", port + 1))
            t.close()
            return port
        except OSError:
            t.close()


class Host:
    """One client attached to a TNC: command lines and data bytes, each timestamped."""

    def __init__(self, name, port, log, t0):
        self.name = name
        self.log = log
        self.t0 = t0
        self.lines = []  # (t, line)
        self.data = bytearray()
        self.data_events = []  # (t, total_len)
        self.lock = threading.Condition()
        self.closed = False
        self.cmd = self._connect(port)
        self.dat = self._connect(port + 1)
        threading.Thread(target=self._read_cmd, daemon=True).start()
        threading.Thread(target=self._read_data, daemon=True).start()

    def now(self):
        return time.monotonic() - self.t0

    def _connect(self, port):
        deadline = time.monotonic() + 15
        while True:
            try:
                return socket.create_connection(("127.0.0.1", port), timeout=5)
            except OSError:
                if time.monotonic() > deadline:
                    raise Fail(f"{self.name}: cannot connect to port {port}")
                time.sleep(0.2)

    def _read_cmd(self):
        buf = b""
        self.cmd.settimeout(None)
        while True:
            try:
                chunk = self.cmd.recv(4096)
            except OSError:
                chunk = b""
            if not chunk:
                with self.lock:
                    self.closed = True
                    self.lock.notify_all()
                return
            buf += chunk
            while b"\r" in buf:
                line, buf = buf.split(b"\r", 1)
                text = line.decode(errors="replace").strip()
                if not text:
                    continue
                t = self.now()
                with self.lock:
                    self.lines.append((t, text))
                    self.lock.notify_all()
                if text != "IAMALIVE":
                    self.log(f"{t:8.2f} {self.name} <- {text}")

    def _read_data(self):
        self.dat.settimeout(None)
        while True:
            try:
                chunk = self.dat.recv(65536)
            except OSError:
                chunk = b""
            if not chunk:
                return
            t = self.now()
            with self.lock:
                self.data += chunk
                self.data_events.append((t, len(self.data)))
                self.lock.notify_all()
            self.log(f"{t:8.2f} {self.name} <- DATA {len(chunk)} B (total {len(self.data)})")

    def send(self, line):
        self.log(f"{self.now():8.2f} {self.name} -> {line}")
        self.cmd.sendall(line.encode() + b"\r")

    def command(self, line, timeout=10):
        """Send a command and wait for its OK/WRONG reply."""
        start = len(self.lines)
        self.send(line)
        got = self.wait_line(lambda l: l in ("OK", "WRONG") or l.startswith("VERSION")
                             or l.startswith("BUFFER") and line == "BUFFER",
                             timeout, start=start, what=f"reply to {line}")
        return got

    def write(self, payload):
        self.log(f"{self.now():8.2f} {self.name} -> DATA {len(payload)} B")
        self.dat.sendall(payload)

    def wait_line(self, pred, timeout, start=0, what="line"):
        deadline = time.monotonic() + timeout
        with self.lock:
            idx = start
            while True:
                while idx < len(self.lines):
                    t, l = self.lines[idx]
                    idx += 1
                    if pred(l):
                        return t, l
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise Fail(f"{self.name}: timeout ({timeout:.0f} s) waiting for {what}")
                self.lock.wait(min(remaining, 1.0))

    def wait_data(self, total, timeout, what="data"):
        deadline = time.monotonic() + timeout
        with self.lock:
            while len(self.data) < total:
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    raise Fail(f"{self.name}: timeout ({timeout:.0f} s) waiting for {what}: "
                               f"{len(self.data)}/{total} B")
                self.lock.wait(min(remaining, 1.0))
            t = next(t for t, n in self.data_events if n >= total)
            return t

    def mark(self):
        with self.lock:
            return len(self.lines)

    def seen(self, pred, start=0):
        with self.lock:
            return [(t, l) for t, l in self.lines[start:] if pred(l)]


class Rig:
    def __init__(self, args, outdir):
        self.args = args
        self.out = outdir
        os.makedirs(outdir, exist_ok=True)
        self.events = open(os.path.join(outdir, "events.log"), "w")
        self.t0 = time.monotonic()
        self.procs = []
        self.metrics = []

    def log(self, msg):
        self.events.write(msg + "\n")
        self.events.flush()
        if self.args.verbose:
            print(msg, flush=True)

    def metric(self, key, value):
        self.metrics.append((key, value))
        self.log(f"METRIC {key}={value}")

    def start(self):
        a = self.args
        tokens = os.path.join(self.out, "tokens.conf")
        with open(tokens, "w") as f:
            f.write("alpha_tok:ALPHA:alpha\nbravo_tok:BRAVO:bravo\n")
        env = dict(os.environ)
        if a.tx_model == "hardware":
            env["ULTRA_SIM_PAPR_PENALTY"] = "1"
        serve_log = open(os.path.join(self.out, "serve.log"), "w")
        cfo = a.cfo_hz
        serve = subprocess.Popen(
            [os.path.join(BUILD, "ota_simulator"), "serve",
             "--bind", "127.0.0.1:0", "--udp-bind", "127.0.0.1:0",
             "--tokens", tokens, "--captures-root", os.path.join(self.out, "caps"),
             "--lobby-channel", a.channel, "--lobby-snr-db", str(a.snr_db),
             "--lobby-seed", str(a.seed),
             "--lobby-rx-cfo-hz", f"BRAVO:{cfo:g},ALPHA:{-cfo:g}",
             "--shutdown-deadline-sec", "10"],
            stdout=serve_log, stderr=subprocess.STDOUT, env=env)
        self.procs.append(serve)
        grpc = None
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline and grpc is None:
            time.sleep(0.25)
            with open(os.path.join(self.out, "serve.log")) as f:
                for line in f:
                    if "OTASIM_SERVE_READY" in line:
                        for tok in line.split():
                            if tok.startswith("grpc="):
                                grpc = tok.split("=", 1)[1]
        if grpc is None:
            raise Fail("ota_simulator did not start")
        self.ports = {}
        for name, call in (("alpha", "ALPHA"), ("bravo", "BRAVO")):
            # Parallel runs can draw the same free port between probe and bind: retry on a
            # bind failure (ultra_tnc exits 1) with a fresh port.
            for attempt in range(8):
                port = free_port_pair()
                while any(abs(port - p) <= 1 for p in self.ports.values()):
                    port = free_port_pair()
                path = os.path.join(self.out, f"{name}.stdout")
                stdout = open(path, "w")
                p = subprocess.Popen(
                    [os.path.join(BUILD, "ultra_tnc"), "--sim-audio",
                     "--ota-host", grpc, "--token", f"{name}_tok", "--station-id", call,
                     "--session-id", "lobby", "--callsign", call,
                     "--bind", "127.0.0.1", "--port", str(port),
                     "--log-level", "debug", "--log-category", "all",
                     "--log-file", os.path.join(self.out, f"{name}.log")],
                    stdout=stdout, stderr=subprocess.STDOUT, env=env)
                deadline = time.monotonic() + 20
                up = False
                while time.monotonic() < deadline:
                    with open(path) as f:
                        text = f.read()
                    if "ultra_tnc listening" in text and "[otasim] Connected" in text:
                        up = True
                        break
                    if p.poll() is not None:
                        break
                    time.sleep(0.2)
                if up:
                    self.ports[name] = port
                    self.procs.append(p)
                    break
                if p.poll() is None:
                    p.kill()
                    p.wait()
                time.sleep(random.uniform(0.2, 1.0))
            else:
                raise Fail(f"{name} TNC did not come up")
        self.t0 = time.monotonic()
        self.alpha = Host("ALPHA", self.ports["alpha"], self.log, self.t0)
        self.bravo = Host("BRAVO", self.ports["bravo"], self.log, self.t0)

    def stop(self):
        for p in reversed(self.procs):
            if p.poll() is None:
                p.send_signal(signal.SIGTERM)
        for p in self.procs:
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()
        self.events.close()

    def alive(self):
        return all(p.poll() is None for p in self.procs)


# ---------------------------------------------------------------- building blocks

def setup(rig, bravo_listen=True):
    a, b = rig.alpha, rig.bravo
    for h, call in ((a, "ALPHA"), (b, "BRAVO")):
        _, r = h.command(f"MYCALL {call}")
        if r != "OK":
            raise Fail(f"{h.name}: MYCALL -> {r}")
    if bravo_listen:
        _, r = b.command("LISTEN ON")
        if r != "OK":
            raise Fail(f"BRAVO: LISTEN ON -> {r}")


def connect(rig, label="connect", timeout=120):
    a, b = rig.alpha, rig.bravo
    ma, mb = a.mark(), b.mark()
    t_start = a.now()
    _, r = a.command("CONNECT ALPHA BRAVO")
    if r != "OK":
        raise Fail(f"ALPHA: CONNECT -> {r}")
    ta, la = a.wait_line(lambda l: l.startswith("CONNECTED") or l == "DISCONNECTED",
                         timeout, start=ma, what="CONNECTED")
    if la == "DISCONNECTED":
        raise Fail("ALPHA: DISCONNECTED instead of CONNECTED")
    tb, lb = b.wait_line(lambda l: l.startswith("CONNECTED"), 30, start=mb,
                         what="CONNECTED")
    for h, line in ((a, la), (b, lb)):
        if "ALPHA BRAVO" not in line:
            raise Fail(f"{h.name}: CONNECTED line '{line}' does not name ALPHA BRAVO")
    pend = b.seen(lambda l: l == "PENDING", mb)
    if not pend:
        rig.log("NOTE BRAVO saw no PENDING before CONNECTED")
    rig.metric(f"{label}_s", f"{ta - t_start:.1f}")
    return ta


def msg_bytes(n, seed):
    rng = random.Random(seed)
    # Text-like content (what a messaging client sends) - compressible on purpose.
    words = [b"QSL", b"TNX", b"FER", b"QSO", b"73", b"DE", b"K", b"WX", b"HR", b"RST", b"599",
             b"NAME", b"QTH", b"RIG", b"ANT", b"PSE", b"AGN", b"BK", b"SK", b"CQ"]
    out = bytearray()
    while len(out) < n:
        out += rng.choice(words) + b" "
    return bytes(out[:n])


def bin_bytes(n, seed):
    rng = random.Random(seed)
    return bytes(rng.randrange(256) for _ in range(n))


def deliver(rig, src, dst, payload, label, timeout):
    """src writes payload; wait until dst's data port has it (exact) and src reports BUFFER 0."""
    base = len(dst.data)
    m = src.mark()
    t_sent = src.now()
    src.write(payload)
    t_rx = dst.wait_data(base + len(payload), timeout, what=label)
    got = bytes(dst.data[base:base + len(payload)])
    if got != payload:
        raise Fail(f"{label}: payload mismatch at {dst.name} "
                   f"(first diff at byte {next(i for i in range(len(payload)) if got[i] != payload[i])})")
    # The sender's queue must drain to BUFFER 0 (a host's Flush() waits on it).
    try:
        t_b0, _ = src.wait_line(lambda l: l == "BUFFER 0", 60, start=m, what="BUFFER 0")
    except Fail:
        raise Fail(f"{label}: {src.name} never reported BUFFER 0 after delivery")
    rig.metric(f"{label}_deliver_s", f"{t_rx - t_sent:.1f}")
    rig.metric(f"{label}_buffer0_s", f"{t_b0 - t_sent:.1f}")
    return t_rx - t_sent


def disconnect(rig, who, label="disconnect", timeout=60):
    a, b = rig.alpha, rig.bravo
    other = b if who is a else a
    mw, mo = who.mark(), other.mark()
    t = who.now()
    _, r = who.command("DISCONNECT")
    if r != "OK":
        raise Fail(f"{who.name}: DISCONNECT -> {r}")
    tw, _ = who.wait_line(lambda l: l == "DISCONNECTED", timeout, start=mw,
                          what="DISCONNECTED")
    to, _ = other.wait_line(lambda l: l == "DISCONNECTED", timeout, start=mo,
                            what="DISCONNECTED (peer)")
    rig.metric(f"{label}_local_s", f"{tw - t:.1f}")
    rig.metric(f"{label}_peer_s", f"{to - t:.1f}")


def check_quiet_after(rig, seconds=8):
    """After a session ends neither TNC should keep transmitting or flip state."""
    a, b = rig.alpha, rig.bravo
    ma, mb = a.mark(), b.mark()
    time.sleep(seconds)
    for h, m in ((a, ma), (b, mb)):
        noisy = h.seen(lambda l: l.startswith(("PTT ON", "CONNECTED", "PENDING")), m)
        if noisy:
            raise Fail(f"{h.name}: activity after the session ended: {noisy[:3]}")


# ---------------------------------------------------------------- scenarios

def sc_short(rig):
    """Short messages one way then the other, one at a time, then a clean disconnect."""
    a, b = rig.alpha, rig.bravo
    setup(rig)
    connect(rig)
    for i, n in enumerate((1, 12, 60, 250, 900)):
        deliver(rig, a, b, msg_bytes(n, 100 + i), f"a2b_{n}B", rig.args.msg_timeout)
    for i, n in enumerate((5, 120)):
        deliver(rig, b, a, msg_bytes(n, 200 + i), f"b2a_{n}B", rig.args.msg_timeout)
    disconnect(rig, a)
    check_quiet_after(rig)


def sc_pingpong(rig):
    """Turn-taking like a mail-forwarding exchange: each side answers the other's message."""
    a, b = rig.alpha, rig.bravo
    setup(rig)
    connect(rig)
    sizes = (40, 25, 180, 30, 90, 12, 300, 8)
    for i, n in enumerate(sizes):
        src, dst = (a, b) if i % 2 == 0 else (b, a)
        deliver(rig, src, dst, msg_bytes(n, 300 + i), f"turn{i}_{src.name[0]}_{n}B",
                rig.args.msg_timeout)
    disconnect(rig, b, label="disconnect_by_bravo")
    check_quiet_after(rig)


def sc_both(rig):
    """Both hosts write at the same moment; both messages must arrive intact."""
    a, b = rig.alpha, rig.bravo
    setup(rig)
    connect(rig)
    pa, pb = msg_bytes(150, 401), msg_bytes(110, 402)
    ba, bb = len(b.data), len(a.data)
    t = a.now()
    a.write(pa)
    b.write(pb)
    tb = b.wait_data(ba + len(pa), rig.args.msg_timeout, what="ALPHA's message")
    ta = a.wait_data(bb + len(pb), rig.args.msg_timeout, what="BRAVO's message")
    if bytes(b.data[ba:]) != pa or bytes(a.data[bb:]) != pb:
        raise Fail("simultaneous messages corrupted")
    rig.metric("both_a2b_s", f"{tb - t:.1f}")
    rig.metric("both_b2a_s", f"{ta - t:.1f}")
    disconnect(rig, a)


def sc_bulk_then_short(rig):
    """A big body (burst path) then short messages both ways on the same connection."""
    a, b = rig.alpha, rig.bravo
    setup(rig)
    connect(rig)
    deliver(rig, a, b, bin_bytes(6000, 501), "bulk_a2b_6000B", rig.args.bulk_timeout)
    deliver(rig, b, a, msg_bytes(40, 502), "after_bulk_b2a_40B", rig.args.msg_timeout)
    deliver(rig, a, b, msg_bytes(30, 503), "after_bulk_a2b_30B", rig.args.msg_timeout)
    disconnect(rig, a)


def sc_chunked(rig):
    """A host that writes one message in small pieces (like a line-by-line client)."""
    a, b = rig.alpha, rig.bravo
    setup(rig)
    connect(rig)
    payload = msg_bytes(400, 601)
    base = len(b.data)
    m = a.mark()
    t = a.now()
    for i in range(0, len(payload), 50):
        a.write(payload[i:i + 50])
        time.sleep(0.1)
    tr = b.wait_data(base + len(payload), rig.args.msg_timeout, what="chunked message")
    if bytes(b.data[base:]) != payload:
        raise Fail("chunked message corrupted")
    a.wait_line(lambda l: l == "BUFFER 0", 60, start=m, what="BUFFER 0")
    rig.metric("chunked_400B_deliver_s", f"{tr - t:.1f}")
    disconnect(rig, a)


def sc_reconnect(rig):
    """Two sessions back to back; the second must behave like the first."""
    a, b = rig.alpha, rig.bravo
    setup(rig)
    connect(rig, label="connect1")
    deliver(rig, a, b, msg_bytes(50, 701), "s1_a2b_50B", rig.args.msg_timeout)
    disconnect(rig, a, label="disconnect1")
    time.sleep(3)
    connect(rig, label="connect2")
    deliver(rig, a, b, msg_bytes(50, 702), "s2_a2b_50B", rig.args.msg_timeout)
    deliver(rig, b, a, msg_bytes(20, 703), "s2_b2a_20B", rig.args.msg_timeout)
    disconnect(rig, a, label="disconnect2")
    check_quiet_after(rig)


def sc_abort(rig):
    """ABORT in the middle of a big send: both ends free, and a new session works."""
    a, b = rig.alpha, rig.bravo
    setup(rig)
    connect(rig, label="connect1")
    a.write(bin_bytes(8000, 801))
    time.sleep(12)
    ma, mb = a.mark(), b.mark()
    t = a.now()
    _, r = a.command("ABORT")
    if r != "OK":
        raise Fail(f"ALPHA: ABORT -> {r}")
    ta, _ = a.wait_line(lambda l: l == "DISCONNECTED", 10, start=ma, what="DISCONNECTED after ABORT")
    rig.metric("abort_local_s", f"{ta - t:.1f}")
    # The peer was not told; it notices on its own once the sender's whole retry budget has
    # passed (Connection::peerSilentLinkLostMs: ~250 s on OFDM, up to ~12 min on MC-DPSK).
    tb, _ = b.wait_line(lambda l: l == "DISCONNECTED", 900, start=mb,
                        what="BRAVO DISCONNECTED after peer ABORT")
    rig.metric("abort_peer_notices_s", f"{tb - t:.1f}")
    time.sleep(3)
    connect(rig, label="connect2")
    deliver(rig, a, b, msg_bytes(40, 802), "after_abort_a2b_40B", rig.args.msg_timeout)
    disconnect(rig, a, label="disconnect2")


def sc_noanswer(rig):
    """Calling a station that is not there: the host must get a clear end, then be usable."""
    a, b = rig.alpha, rig.bravo
    setup(rig, bravo_listen=False)
    m = a.mark()
    t = a.now()
    _, r = a.command("CONNECT ALPHA NOBODY")
    if r != "OK":
        raise Fail(f"ALPHA: CONNECT -> {r}")
    td, _ = a.wait_line(lambda l: l == "DISCONNECTED" or l.startswith("CONNECTED"), 300,
                        start=m, what="end of the unanswered call")
    if a.seen(lambda l: l.startswith("CONNECTED"), m):
        raise Fail("connected to a station that does not exist")
    rig.metric("noanswer_gives_up_s", f"{td - t:.1f}")
    # BRAVO was not listening: it must not have answered or announced anything.
    if b.seen(lambda l: l.startswith(("PENDING", "CONNECTED"))):
        raise Fail("BRAVO (not listening) reacted to the call")
    check_quiet_after(rig, 5)
    # Same host can now reach BRAVO once it listens.
    _, r = b.command("LISTEN ON")
    connect(rig, label="connect_after_noanswer")
    deliver(rig, a, b, msg_bytes(30, 901), "after_noanswer_a2b_30B", rig.args.msg_timeout)
    disconnect(rig, a)


def count_tx(rig, station):
    """Transmissions a station keyed, from its log (PING/PONG, frames, bursts, tone ACKs)."""
    path = os.path.join(rig.out, f"{station}.log")
    tag = f"[{station.upper()}] TX"
    with open(path, errors="replace") as f:
        return sum(1 for line in f if tag in line)


def sc_not_listening(rig):
    """A TNC whose host never said LISTEN ON takes no call and never keys its transmitter."""
    a, b = rig.alpha, rig.bravo
    setup(rig, bravo_listen=False)
    m = a.mark()
    t = a.now()
    _, r = a.command("CONNECT ALPHA BRAVO")
    if r != "OK":
        raise Fail(f"ALPHA: CONNECT -> {r}")
    td, line = a.wait_line(lambda l: l == "DISCONNECTED" or l.startswith("CONNECTED"), 300,
                           start=m, what="end of the call to a station not listening")
    if line.startswith("CONNECTED"):
        raise Fail("connected to a station whose host never said LISTEN ON")
    rig.metric("not_listening_gives_up_s", f"{td - t:.1f}")
    if b.seen(lambda l: l.startswith(("PENDING", "CONNECTED"))):
        raise Fail("BRAVO (not listening) reported the call to its host")
    tx = count_tx(rig, "bravo")
    rig.metric("bravo_tx_while_not_listening", tx)
    if tx:
        raise Fail(f"BRAVO (not listening) keyed its transmitter {tx} time(s)")


def sc_simultaneous(rig):
    """Both stations listen and call each other at the same moment: exactly one session results,
    both hosts agree on who called whom, and it carries data both ways."""
    a, b = rig.alpha, rig.bravo
    setup(rig)
    _, r = a.command("LISTEN ON")
    if r != "OK":
        raise Fail(f"ALPHA: LISTEN ON -> {r}")
    ma, mb = a.mark(), b.mark()
    t = a.now()
    a.send("CONNECT ALPHA BRAVO")
    b.send("CONNECT BRAVO ALPHA")
    ta, la = a.wait_line(lambda l: l.startswith("CONNECTED"), 300, start=ma, what="ALPHA CONNECTED")
    tb, lb = b.wait_line(lambda l: l.startswith("CONNECTED"), 60, start=mb, what="BRAVO CONNECTED")
    pair_a, pair_b = la.split()[1:3], lb.split()[1:3]
    if pair_a != pair_b:
        raise Fail(f"the hosts disagree on the session: ALPHA '{la}', BRAVO '{lb}'")
    rig.metric("simultaneous_connect_s", f"{max(ta, tb) - t:.1f}")
    rig.metric("simultaneous_winner", pair_a[0])
    deliver(rig, a, b, msg_bytes(40, 1101), "simul_a2b_40B", rig.args.msg_timeout)
    deliver(rig, b, a, msg_bytes(40, 1102), "simul_b2a_40B", rig.args.msg_timeout)
    for h, m in ((a, ma), (b, mb)):
        extra = h.seen(lambda l: l.startswith("CONNECTED"), m)
        if len(extra) != 1:
            raise Fail(f"{h.name}: {len(extra)} CONNECTED lines, want exactly one session")
    disconnect(rig, a)


def sc_marathon(rig):
    """Many sessions on the same two TNC processes, never restarted: each side calls in turn, each
    session carries several short messages and several bulk (file-path) blocks both ways, and every
    byte is checked."""
    a, b = rig.alpha, rig.bravo
    setup(rig)
    _, r = a.command("LISTEN ON")
    if r != "OK":
        raise Fail(f"ALPHA: LISTEN ON -> {r}")

    def connect_from(caller, callee, label):
        mc, me = caller.mark(), callee.mark()
        t = caller.now()
        _, r = caller.command(f"CONNECT {caller.name} {callee.name}")
        if r != "OK":
            raise Fail(f"{caller.name}: CONNECT -> {r}")
        tc, lc = caller.wait_line(lambda l: l.startswith("CONNECTED") or l == "DISCONNECTED",
                                  180, start=mc, what=f"{label} CONNECTED")
        if lc == "DISCONNECTED":
            raise Fail(f"{label}: {caller.name} got DISCONNECTED instead of CONNECTED")
        callee.wait_line(lambda l: l.startswith("CONNECTED"), 60, start=me,
                         what=f"{label} CONNECTED (callee)")
        expected = f"CONNECTED {caller.name} {callee.name}"
        if not lc.startswith(expected):
            raise Fail(f"{label}: '{lc}', want '{expected} ...'")
        rig.metric(f"{label}_connect_s", f"{tc - t:.1f}")

    seed = 2000
    sessions = [(a, b, a), (b, a, b), (a, b, b), (b, a, a), (a, b, a)]
    for i, (caller, callee, closer) in enumerate(sessions, start=1):
        label = f"s{i}"
        connect_from(caller, callee, label)
        for n in (30, 120):
            seed += 1
            deliver(rig, caller, callee, msg_bytes(n, seed), f"{label}_msg_{caller.name[0]}_{n}B",
                    rig.args.msg_timeout)
            seed += 1
            deliver(rig, callee, caller, msg_bytes(n // 2, seed),
                    f"{label}_msg_{callee.name[0]}_{n // 2}B", rig.args.msg_timeout)
        # Two bulk blocks (> 4 KB: the TNC sends them as file transfers), then one the other way.
        for k in range(2):
            seed += 1
            deliver(rig, caller, callee, bin_bytes(5000, seed),
                    f"{label}_bulk{k}_{caller.name[0]}_5000B", rig.args.bulk_timeout)
        seed += 1
        deliver(rig, callee, caller, bin_bytes(4500, seed),
                f"{label}_bulk_{callee.name[0]}_4500B", rig.args.bulk_timeout)
        seed += 1
        deliver(rig, caller, callee, msg_bytes(20, seed), f"{label}_after_bulk_{caller.name[0]}_20B",
                rig.args.msg_timeout)
        disconnect(rig, closer, label=f"{label}_disconnect")
        time.sleep(3)
    rig.metric("sessions_completed", len(sessions))
    for p in rig.procs:
        if p.poll() is not None:
            raise Fail("a process exited during the marathon")


def sc_cancel(rig):
    """DISCONNECT while the call is still being set up cancels it on both ends."""
    a, b = rig.alpha, rig.bravo
    setup(rig)
    m = a.mark()
    mb = b.mark()
    a.command("CONNECT ALPHA BRAVO")
    time.sleep(4)
    t = a.now()
    a.command("DISCONNECT")
    td, _ = a.wait_line(lambda l: l == "DISCONNECTED", 15, start=m, what="DISCONNECTED (cancel)")
    rig.metric("cancel_s", f"{td - t:.1f}")
    time.sleep(20)
    lines_b = b.seen(lambda l: l.startswith(("CONNECTED", "PENDING", "CANCELPENDING",
                                            "DISCONNECTED")), mb)
    rig.log(f"BRAVO after the cancelled call: {lines_b}")
    if any(l.startswith("CONNECTED") for _, l in lines_b) and not any(
            l == "DISCONNECTED" for _, l in lines_b):
        raise Fail("BRAVO left CONNECTED after ALPHA cancelled the call")
    connect(rig, label="connect_after_cancel")
    deliver(rig, a, b, msg_bytes(30, 1001), "after_cancel_a2b_30B", rig.args.msg_timeout)
    disconnect(rig, a)


def sc_commands(rig):
    """Command-port hygiene: every command answers, unknown ones answer WRONG."""
    a = rig.alpha
    for cmd, want in (("VERSION", "VERSION"), ("MYCALL ALPHA", "OK"), ("BW2300", "OK"),
                      ("CHAT ON", "OK"), ("COMPRESSION TEXT", "OK"), ("LISTEN OFF", "OK"),
                      ("PUBLIC ON", "OK"), ("CWID OFF", "OK"), ("NOSUCHCMD", "WRONG"),
                      ("LISTEN MAYBE", "WRONG")):
        _, r = a.command(cmd)
        if not r.startswith(want):
            raise Fail(f"{cmd} -> {r} (want {want})")
    m = a.mark()
    a.send("BUFFER")
    a.wait_line(lambda l: l.startswith("BUFFER"), 5, start=m, what="BUFFER reply")
    m = a.mark()
    a.send("STATS")
    a.wait_line(lambda l: l.startswith("STATS"), 5, start=m, what="STATS reply")
    # IAMALIVE keepalive must arrive on an idle port.
    m = a.mark()
    a.wait_line(lambda l: l == "IAMALIVE", 70, start=m, what="IAMALIVE")


SCENARIOS = {
    "commands": sc_commands,
    "short": sc_short,
    "pingpong": sc_pingpong,
    "both": sc_both,
    "chunked": sc_chunked,
    "bulk_then_short": sc_bulk_then_short,
    "reconnect": sc_reconnect,
    "abort": sc_abort,
    "noanswer": sc_noanswer,
    "not_listening": sc_not_listening,
    "simultaneous": sc_simultaneous,
    "marathon": sc_marathon,
    "cancel": sc_cancel,
}


def run_one(args, name):
    outdir = os.path.join(args.out, name)
    if os.path.isdir(outdir):
        shutil.rmtree(outdir)
    rig = Rig(args, outdir)
    result, reason = "PASS", ""
    t0 = time.monotonic()
    watchdog = threading.Timer(args.timeout, lambda: os.kill(os.getpid(), signal.SIGALRM))
    try:
        rig.start()
        watchdog.start()
        SCENARIOS[name](rig)
    except Fail as e:
        result, reason = "FAIL", str(e)
    except TimeoutError as e:
        result, reason = "FAIL", f"scenario timeout: {e}"
    except Exception as e:  # noqa: BLE001 - report anything as a failure with its text
        result, reason = "FAIL", f"{type(e).__name__}: {e}"
    finally:
        watchdog.cancel()
        dead = [p.args[0] for p in rig.procs if p.poll() is not None]
        if dead and result == "PASS":
            result, reason = "FAIL", f"process exited: {dead}"
        rig.stop()
    elapsed = time.monotonic() - t0
    metrics = " ".join(f"{k}={v}" for k, v in rig.metrics)
    line = (f"RESULT={result} scenario={name} channel={args.channel} snr={args.snr_db} "
            f"seed={args.seed} elapsed={elapsed:.0f}s {metrics}"
            + (f" reason=\"{reason}\"" if reason else ""))
    with open(os.path.join(outdir, "summary.txt"), "w") as f:
        f.write(line + "\n")
    print(line, flush=True)
    return result == "PASS"


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--scenario", default="all", help=f"one of {', '.join(SCENARIOS)} or all")
    p.add_argument("--channel", default="awgn")
    p.add_argument("--snr-db", type=float, default=10)
    p.add_argument("--seed", type=int, default=42)
    p.add_argument("--cfo-hz", type=float, default=7.3)
    p.add_argument("--tx-model", choices=("hardware", "sim"), default="hardware")
    p.add_argument("--msg-timeout", type=float, default=240, help="per short message (s)")
    p.add_argument("--bulk-timeout", type=float, default=600, help="per bulk body (s)")
    p.add_argument("--timeout", type=float, default=1500, help="whole scenario (s)")
    p.add_argument("--out", required=True)
    p.add_argument("--verbose", action="store_true")
    p.add_argument("--bin-dir", default=None,
                   help="directory holding ultra_tnc and ota_simulator (default: <repo>/build)")
    args = p.parse_args()
    if args.bin_dir:
        globals()["BUILD"] = os.path.abspath(args.bin_dir)

    def on_alarm(signum, frame):
        raise TimeoutError("whole-scenario watchdog")
    signal.signal(signal.SIGALRM, on_alarm)

    names = list(SCENARIOS) if args.scenario == "all" else args.scenario.split(",")
    for n in names:
        if n not in SCENARIOS:
            sys.exit(f"unknown scenario {n}")
    os.makedirs(args.out, exist_ok=True)
    ok = True
    for n in names:
        ok = run_one(args, n) and ok
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
