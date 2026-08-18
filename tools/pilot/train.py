#!/usr/bin/env python3
"""Clone the operator: recorded rings in, the intent that went with them out.

    train.py [--episodes ../webconsole/episodes] [--epochs 60] [--out policy.pt]

Only rows where the vehicle was armed are used. A disarmed row records a lidar
scan and an intent of zero regardless of what the operator meant, so it teaches
"whatever you see, do nothing" - and those rows outnumber the driving ones in any
session that includes parking, walking around, and looking at the map.

The refusal below matters more than the training loop. Behaviour cloning on labels
that never vary produces a net with a good loss and no behaviour: it predicts the
mean and the mean is a constant. That is indistinguishable from success on the loss
curve, so it is checked before the first epoch rather than diagnosed afterwards.
"""
import argparse
import glob
import os
import sys

import numpy as np
import torch
import torch.nn as nn

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from model import RingPolicy, rings_to_input      # noqa: E402


def load_episodes(pattern):
    xs, ys, kept, seen = [], [], 0, 0
    files = sorted(glob.glob(pattern))
    if not files:
        sys.exit(f"no episodes matched {pattern}")
    beams = None
    for f in files:
        d = np.load(f)
        ring, armed = d["ring"], d["armed"].astype(bool)
        seen += len(ring)
        if beams is None:
            beams = ring.shape[1]
        elif ring.shape[1] != beams:
            print(f"  skipping {os.path.basename(f)}: {ring.shape[1]} beams, "
                  f"not {beams}")
            continue
        if not armed.any():
            print(f"  {os.path.basename(f)}: no armed rows, skipped")
            continue
        xs.append(rings_to_input(ring[armed]))
        ys.append(np.stack([d["x"][armed], d["y"][armed], d["r"][armed]], 1))
        kept += int(armed.sum())
        print(f"  {os.path.basename(f)}: {int(armed.sum())} of {len(ring)} rows armed")
    if not xs:
        sys.exit("nothing usable: every episode was recorded disarmed")
    return (np.concatenate(xs), np.concatenate(ys).astype(np.float32),
            beams, kept, seen)


def main():
    ap = argparse.ArgumentParser()
    here = os.path.dirname(os.path.abspath(__file__))
    ap.add_argument("--episodes",
                    default=os.path.join(here, "..", "webconsole", "episodes"))
    ap.add_argument("--epochs", type=int, default=60)
    ap.add_argument("--batch", type=int, default=64)
    ap.add_argument("--lr", type=float, default=1e-3)
    ap.add_argument("--out", default=os.path.join(here, "policy.pt"))
    ap.add_argument("--min-rows", type=int, default=500,
                    help="refuse to train on less than this")
    a = ap.parse_args()

    X, Y, beams, kept, seen = load_episodes(os.path.join(a.episodes, "*.npz"))
    print(f"  {kept} armed rows of {seen} recorded, {beams} beams")

    # Does the data contain any behaviour to clone?
    spread = Y.std(axis=0)
    names = ("strafe", "forward", "yaw")
    print("  label spread: " +
          ", ".join(f"{n} {s:.3f}" for n, s in zip(names, spread)))
    if spread.max() < 0.02:
        sys.exit("the operator's intent barely changes across this data - there is "
                 "no behaviour here to clone. Record while actually driving.")
    if kept < a.min_rows:
        sys.exit(f"{kept} armed rows is not enough to train on (want "
                 f"{a.min_rows}+, which is about a minute of driving). "
                 f"Pass --min-rows to override.")

    dev = "cuda" if torch.cuda.is_available() else "cpu"
    print(f"  training on {dev}")
    # Split by time, not at random: consecutive rings are nearly identical, so a
    # random split puts near-copies of the validation rows in the training set and
    # the validation loss stops meaning anything.
    cut = int(len(X) * 0.85)
    xt = torch.from_numpy(X[:cut]).to(dev)
    yt = torch.from_numpy(Y[:cut]).to(dev)
    xv = torch.from_numpy(X[cut:]).to(dev)
    yv = torch.from_numpy(Y[cut:]).to(dev)

    net = RingPolicy(beams=beams, outputs=3).to(dev)
    opt = torch.optim.Adam(net.parameters(), lr=a.lr)
    lossf = nn.SmoothL1Loss()
    best = float("inf")
    for ep in range(1, a.epochs + 1):
        net.train()
        perm = torch.randperm(len(xt), device=dev)
        tot = 0.0
        for i in range(0, len(xt), a.batch):
            j = perm[i:i + a.batch]
            opt.zero_grad()
            l = lossf(net(xt[j]), yt[j])
            l.backward()
            opt.step()
            tot += l.item() * len(j)
        net.eval()
        with torch.no_grad():
            vl = lossf(net(xv), yv).item() if len(xv) else float("nan")
        if vl < best:
            best = vl
            torch.save({"state": net.state_dict(), "beams": beams, "outputs": 3,
                        "val": vl, "rows": kept}, a.out)
        if ep % 10 == 0 or ep == 1:
            print(f"    epoch {ep:3d}  train {tot/len(xt):.5f}  val {vl:.5f}")

    # Against the only baseline that matters for a cloner: predicting the mean.
    mean = torch.from_numpy(Y[:cut].mean(axis=0)).to(dev)
    with torch.no_grad():
        base = lossf(mean.expand_as(yv), yv).item() if len(xv) else float("nan")
    print(f"  best val {best:.5f}, predicting the mean would give {base:.5f}")
    if best >= base:
        print("  the net is no better than a constant - it has learned nothing "
              "useful from this data")
    print(f"  wrote {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
