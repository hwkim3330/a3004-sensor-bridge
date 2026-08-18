#!/usr/bin/env python3
"""The net, and the one place that decides how a ring becomes an input.

Shared by training and by driving on purpose. A behaviour-cloning policy fails
silently and completely if the two disagree about normalisation - the loss curve
looks fine and the vehicle drives into a wall - so there is one function and both
callers import it.

The shape is the TinyLidarNet arrangement: 1D convolutions along the scan, then a
small head. It suits a range scan for the reason it was built for one - a wall is
a local pattern of neighbouring beams, and the convolution is the operator that
says so, at a fraction of the parameters a dense layer over 360 beams would need.
"""
import numpy as np
import torch
import torch.nn as nn

# Indoors, and clipped deliberately.
#
# The sensor reports out to 100 m and the recorded episodes contain returns at
# 97 m - a corridor's far end through a doorway. Feeding that raw means the range
# that matters, the two metres in front of the vehicle, occupies two percent of
# the input scale. Ten metres is past anything this robot must react to.
MAX_CM = 1000.0


def rings_to_input(ring):
    """(..., beams) int16 in cm, -1 for no return -> float32 in [0, 1].

    A sector with no return becomes 1.0, the far end of the scale, because that is
    what it means: nothing within range in that direction. Substituting 0 would
    tell the policy there is a wall against the sensor, which is the opposite.
    """
    x = np.asarray(ring, dtype=np.float32)
    x = np.where(x < 0, MAX_CM, x)
    np.clip(x, 0.0, MAX_CM, out=x)
    return x / MAX_CM


class RingPolicy(nn.Module):
    """ring -> (strafe, forward, yaw), each in [-1, 1]."""

    def __init__(self, beams=360, outputs=3):
        super().__init__()
        self.beams = beams
        self.conv = nn.Sequential(
            nn.Conv1d(1, 24, 10, stride=4), nn.ReLU(),
            nn.Conv1d(24, 36, 8, stride=4), nn.ReLU(),
            nn.Conv1d(36, 48, 4, stride=2), nn.ReLU(),
            nn.Conv1d(48, 64, 3), nn.ReLU(),
            nn.Conv1d(64, 64, 3), nn.ReLU(),
        )
        with torch.no_grad():
            n = self.conv(torch.zeros(1, 1, beams)).numel()
        self.head = nn.Sequential(
            nn.Flatten(), nn.Dropout(0.1),
            nn.Linear(n, 100), nn.ReLU(),
            nn.Linear(100, 50), nn.ReLU(),
            nn.Linear(50, 10), nn.ReLU(),
            nn.Linear(10, outputs), nn.Tanh(),
        )

    def forward(self, x):
        if x.dim() == 2:
            x = x.unsqueeze(1)
        return self.head(self.conv(x))


def load(path, device="cpu"):
    """A checkpoint plus the geometry it was trained for, so a mismatch is loud."""
    ck = torch.load(path, map_location=device, weights_only=False)
    m = RingPolicy(beams=ck["beams"], outputs=ck["outputs"])
    m.load_state_dict(ck["state"])
    m.to(device).eval()
    return m, ck
