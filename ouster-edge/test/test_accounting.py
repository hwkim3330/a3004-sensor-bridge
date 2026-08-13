#!/usr/bin/env python3
"""Two questions the first run left open:
   1. is the 128 -> 99 shortfall loopback drop, or the daemon losing packets?
   2. does missed_columns actually count a deliberate gap?
"""
import json, os, socket, struct, subprocess, sys, tempfile, time
BIN=os.environ.get("OUSTER_EDGE_BIN", "./ouster-edge")
STATUS=tempfile.mkstemp(suffix=".json")[1]
CH,COLS,WIDTH=64,16,1024
PORT=26512
def px(r): return struct.pack("<IBBHHH", r, 200, 0, 0, 0, 0)
def build(frame, mids):
    body=b""
    for mid in mids:
        body += struct.pack("<QHH", mid*1000, mid, 1) + px(1000+mid)*CH
    return struct.pack("<HHI",0x1,frame,0)+b"\0"*24+body+b"\0"*32

def run(skip_packets, pace, label):
    if os.path.exists(STATUS): os.remove(STATUS)
    p=subprocess.Popen([BIN,"-f","-p",str(PORT),"-c",str(CH),"-C",str(COLS),
        "-w",str(WIDTH),"-s","1024","-S",STATUS,"-I","50"],stderr=subprocess.DEVNULL)
    time.sleep(0.6)
    tx=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    tx.setsockopt(socket.SOL_SOCKET, socket.SO_SNDBUF, 1<<20)
    sent=0
    for frame in (7,8):
        for i,start in enumerate(range(0,WIDTH,COLS)):
            if frame==8 and i in skip_packets: continue
            tx.sendto(build(frame,range(start,start+COLS)),("127.0.0.1",PORT))
            sent+=1
            time.sleep(pace)
    time.sleep(1.2)
    p.terminate(); p.wait(timeout=3)
    st=json.load(open(STATUS))
    print(f"{label}: sent={sent} received={st['packets']} "
          f"missed_columns={st['missed_columns']} invalid={st['invalid_columns']} bad_size={st['bad_size']}")
    return sent, st

print("Q1: no skips, paced 5 ms -> does the daemon see every packet?")
sent, st = run(set(), 0.005, "  paced")
ok1 = (st['packets']==sent and st['missed_columns']==0)
print(f"  -> {'PASS' if ok1 else 'FAIL'}")

print("Q2: skip 1 packet (16 columns) in frame 8")
sent, st = run({30}, 0.005, "  one gap")
ok2 = (st['packets']==sent and st['missed_columns']==16)
print(f"  -> {'PASS' if ok2 else 'FAIL'} (expected missed_columns=16)")

print("Q3: skip 3 packets (48 columns)")
sent, st = run({10,30,50}, 0.005, "  three gaps")
ok3 = (st['packets']==sent and st['missed_columns']==48)
print(f"  -> {'PASS' if ok3 else 'FAIL'} (expected missed_columns=48)")

print("Q4: a column id past every real scan width -> counted, not logged forever")
# measurement_id is 16 bits of unvalidated network input. The width-learning
# path used to log and reassign on every such column, so one bad stream wrote
# thousands of 'scan width 4096 -> 4096' lines a second into the ring buffer.
# The first column still legitimately widens 1024 -> 4096 and logs once; the
# remaining 15 are then recognised as bogus and counted.
if os.path.exists(STATUS): os.remove(STATUS)
err = tempfile.mkstemp(suffix=".log")[1]
with open(err, "w") as ef:
    p = subprocess.Popen([BIN,"-f","-p",str(PORT),"-c",str(CH),"-C",str(COLS),
        "-w",str(WIDTH),"-s","1024","-S",STATUS,"-I","50"], stderr=ef)
    time.sleep(0.6)
    tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    tx.sendto(build(9, [65535]*COLS), ("127.0.0.1", PORT))
    time.sleep(0.8)
    p.terminate(); p.wait(timeout=3)
st = json.load(open(STATUS))
widthlines = sum(1 for l in open(err) if "scan width" in l)
print(f"  bogus mids: invalid={st['invalid_columns']} "
      f"'scan width' log lines={widthlines}")
ok4 = (st['invalid_columns'] == COLS - 1 and widthlines <= 1)
print(f"  -> {'PASS' if ok4 else 'FAIL'} "
      f"(expected invalid={COLS-1}, at most 1 log line)")
os.remove(err)

print("Q5: an azimuth window means absent ids are not loss")
# A sensor restricted to an arc does not send the columns outside it. Feed only
# the ids inside a window and assert the daemon does not call the rest missing -
# on real hardware this made a healthy OS-1-64 report 72% loss.
if os.path.exists(STATUS): os.remove(STATUS)
# The end is chosen so the window holds a whole number of packets. With 257 ids
# and 16 columns per packet the last one cannot be sent, and the daemon is right
# to count it - that off-by-one was the test's, not its.
AZ = (315000, 44700)
lo = AZ[0] * WIDTH // 360000
hi = AZ[1] * WIDTH // 360000
assert ((WIDTH - lo) + (hi + 1)) % COLS == 0, "window must hold whole packets"
p = subprocess.Popen([BIN,"-f","-p",str(PORT),"-c",str(CH),"-C",str(COLS),
    "-w",str(WIDTH),"-s","1024","-S",STATUS,"-I","50",
    "--azimuth-window", f"{AZ[0]}:{AZ[1]}"], stderr=subprocess.DEVNULL)
time.sleep(0.6)
tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
inside = [m for m in range(WIDTH) if (m >= lo or m <= hi)]
for frame in (11, 12):
    for i in range(0, len(inside), COLS):
        chunk = inside[i:i + COLS]
        if len(chunk) < COLS:
            break
        tx.sendto(build(frame, chunk), ("127.0.0.1", PORT))
        time.sleep(0.005)
time.sleep(1.0)
p.terminate(); p.wait(timeout=3)
st = json.load(open(STATUS))
print(f"  window {AZ}: missed_columns={st['missed_columns']} "
      f"packets={st['packets']} azimuth_window={st.get('azimuth_window')}")
ok5 = st['missed_columns'] == 0 and st['packets'] > 0
print(f"  -> {'PASS' if ok5 else 'FAIL'} (expected 0 missed inside the window)")

sys.exit(0 if (ok1 and ok2 and ok3 and ok4 and ok5) else 1)
