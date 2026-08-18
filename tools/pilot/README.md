# A lidar policy, on the PC

The router publishes a ring: one range per direction, 360 of them, ten times a
second. That is a 2D scan, which is exactly the input a small 1D-convolutional
policy eats — the TinyLidarNet shape. So the model that fits this robot is not a
camera-based driving model trained on roads; it is a scan-to-command net, and the
sensor format is already the right one with no conversion in between.

This machine has the hardware for it: RTX 3090, 24 GB, torch with CUDA. Inference
on one ring measures **~2.5 ms**, against a 100 ms ring interval, so the GPU is not
the constraint and never will be for a net this size.

    record   in the browser console: the Record button
    train    ./train.py
    drive    ./pilot.py            (dry run, cannot move the vehicle)
             ./pilot.py --arm      (it can)

## What is blocked, and on what

**There is no training data yet, and there cannot be until the vehicle drives.**
Behaviour cloning needs demonstrations: a person driving well, with the scan that
person was looking at. The CAN wiring is not finished, so nothing has been driven,
so every episode recorded so far has an intent that either does not vary or is not
real. Recorded and checked:

    label spread: strafe 0.000, forward 0.260, yaw 0.071
    best val 0.00543, predicting the mean would give 0.00588

The net trained on that is not better than a constant, and `pilot.py` shows exactly
that — the same `forward +0.06 yaw +0.05` on every ring, whatever the room looks
like. Both of those readouts exist so that this is a sentence someone reads rather
than a week someone spends.

So what is built here is the part that does not need the vehicle: the recorder, the
input contract, the training loop, and the driving harness. When CAN comes up, data
collection is one button.

## Two guards in train.py, and why

`train.py` refuses rather than trains when:

- **the labels barely vary.** A cloner on constant labels predicts the mean, which
  has a fine loss and no behaviour. On a loss curve that is indistinguishable from
  working.
- **there are too few rows.** Under a minute of driving is not a dataset.

It also always prints the loss of *predicting the training mean*. If the net cannot
beat that, it has learned nothing, and the number says so next to the number that
looks like success.

Only armed rows are used. A disarmed row pairs a real scan with an intent of zero
whatever the operator meant, and in any real session those rows outnumber the
driving ones — parking, walking about, looking at the map.

## The safety story is that there isn't a new one

`pilot.py` is a client of the web console, the same as a browser. It asks for
control and sends intent; it does not write `TCMD` to the wire. So:

- the console server disarms it after 250 ms of silence,
- `teleop` on the router goes neutral after 300 ms without frames,
- control is a single slot, so it cannot drive while a person holds it,

and none of that had to be extended or trusted afresh. Taking the other route — a
process that speaks to `teleop` directly — would have meant a second implementation
of the arming rules, which is how two of them end up disagreeing.

It is **disarmed unless `--arm`**, and `--max` scales the output (0.4 by default).
The first run of a cloned policy is the one you least want to be a surprise, and
its interesting failure — steering into the wall it was trained to avoid — is
visible in the printout before it is visible in the room.

## The duplicate ring

This machine has two interfaces on the router's subnet, wired and wifi, so a
broadcast to 192.168.1.255 is delivered to a socket bound to `0.0.0.0` **twice**:
18.9 datagrams a second carrying 10.1 distinct frame ids. The tablet has one
interface and never saw it.

Both the console server and `pilot.py` drop a repeat of the last frame id. It is
not cosmetic: a recorder that keeps both copies hands a model two identical inputs
for every one the vehicle will actually see.
