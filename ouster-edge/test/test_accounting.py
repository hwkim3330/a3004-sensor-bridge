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

sys.exit(0 if (ok1 and ok2 and ok3) else 1)
