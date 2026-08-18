#!/usr/bin/env python3
"""Turn the router's raw-lidar relay on for as long as something is reading it.

    relay.py status [host]
    relay.py on <dest> [host]        # dest like 192.168.1.20:7502
    relay.py off [host]

Or, from a tool that wants the relay for the duration of a run:

    with relay.borrowed(host, "192.168.1.20:7502") as r:
        ...   # r.note says what was done

Why this exists. `ouster-edge --relay` forwards the sensor's own 12544-byte
datagrams to one address, and UDP gives it no way to know whether anything is
listening. It was found sending **64.2 Mbit/s to a port with nothing bound to
it**, for as long as the setting had been there: clearing it took the board's
kernel time from 56% to 21%, idle from 22% to 64%, and load average from 5.43 to
3.24. About 1.4 of four cores, for packets nobody read.

So the relay is off by default, which then broke the tools that need it - they sat
receiving nothing, because a UDP socket with no traffic looks exactly like a UDP
socket whose sender is switched off. Leaving that in a README was not a fix.

Borrowing it is the honest arrangement: on while a reader is running, off when it
stops, and both said out loud. It costs a restart of `ouster-edge` at each end,
about nine seconds, because `--relay` is a start argument.
"""
import contextlib
import subprocess
import sys


def _ssh(host, cmd, timeout=90):
    r = subprocess.run(
        ["ssh", "-o", "ConnectTimeout=5", "-o", "BatchMode=yes",
         f"root@{host}", cmd],
        capture_output=True, text=True, timeout=timeout)
    if r.returncode != 0:
        raise RuntimeError(f"ssh to {host} failed: "
                           f"{(r.stderr or r.stdout).strip() or r.returncode}")
    return r.stdout.strip()


def status(host="192.168.1.1"):
    """The current relay destination, or "" when it is off."""
    return _ssh(host, "uci -q get ouster-edge.lidar.relay || true")


def set_to(host, dest):
    """Point the relay at `dest`, or switch it off with an empty string."""
    quoted = dest.replace("'", "")
    _ssh(host, f"uci set ouster-edge.lidar.relay='{quoted}'; "
               f"uci commit ouster-edge; "
               f"/etc/init.d/ouster-edge restart >/dev/null 2>&1; sleep 9; "
               f"pgrep -f 'ouster-edge --port' >/dev/null && echo up || echo DOWN")
    return status(host)


@contextlib.contextmanager
def borrowed(host, dest, quiet=False):
    """Relay to `dest` for the body, then put it back exactly as it was.

    Restores on the way out even if the body raises, because the alternative is
    the state this module exists to prevent: a relay left running at a reader
    that has gone.
    """
    class R:
        note = ""
    r = R()
    before = status(host)
    if before == dest:
        r.note = f"relay was already {dest}, left alone"
        if not quiet:
            print(f"  {r.note}")
        yield r
        return
    if not quiet:
        print(f"  relay: {before or 'off'} -> {dest} (restarting ouster-edge, ~9 s)",
              flush=True)
    set_to(host, dest)
    r.note = f"relay {before or 'off'} -> {dest}"
    try:
        yield r
    finally:
        if not quiet:
            print(f"  relay: {dest} -> {before or 'off'} (restarting, ~9 s)",
                  flush=True)
        try:
            set_to(host, before)
        except Exception as e:
            print(f"  WARNING: could not restore the relay to "
                  f"{before or 'off'}: {e}\n"
                  f"  it is still sending to {dest}. "
                  f"Fix with: relay.py off {host}", file=sys.stderr)


def local_address_for(router):
    """This machine's address on the router's network.

    Asked of the routing table rather than guessed: this machine has several
    addresses, and only the one the kernel would use to reach the router is the one
    the router can send back to.
    """
    import socket
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect((router, 9))
        return s.getsockname()[0]
    finally:
        s.close()


def borrow_for_process(host, dest, quiet=False):
    """Borrow the relay for the life of this process.

    A context manager is tidier but would mean restructuring three readers around
    an indent. atexit covers what matters: a normal return, an unhandled exception,
    and Ctrl-C, which is how these are usually stopped. It does not cover being
    killed outright - so the note says what to run if that happens.
    """
    import atexit
    before = status(host)
    if before == dest:
        if not quiet:
            print(f"  relay already {dest}")
        return before
    if not quiet:
        print(f"  relay: {before or 'off'} -> {dest} "
              f"(restarting ouster-edge, ~9 s)", flush=True)
    set_to(host, dest)

    def restore():
        try:
            if not quiet:
                print(f"\n  relay: {dest} -> {before or 'off'} (restarting, ~9 s)",
                      flush=True)
            set_to(host, before)
        except Exception as e:
            print(f"  WARNING: relay left pointing at {dest}: {e}\n"
                  f"  switch it off with: relay.py off {host}", file=sys.stderr)

    atexit.register(restore)
    return before


def main():
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    op = sys.argv[1]
    if op == "status":
        host = sys.argv[2] if len(sys.argv) > 2 else "192.168.1.1"
        s = status(host)
        print(f"  relay: {s or 'off'}")
    elif op == "on":
        if len(sys.argv) < 3:
            sys.exit("on needs a destination, like 192.168.1.20:7502")
        dest = sys.argv[2]
        host = sys.argv[3] if len(sys.argv) > 3 else "192.168.1.1"
        print(f"  setting relay to {dest} (restarting ouster-edge, ~9 s)",
              flush=True)
        print(f"  relay: {set_to(host, dest) or 'off'}")
    elif op == "off":
        host = sys.argv[2] if len(sys.argv) > 2 else "192.168.1.1"
        print("  switching the relay off (restarting ouster-edge, ~9 s)",
              flush=True)
        print(f"  relay: {set_to(host, '') or 'off'}")
    else:
        sys.exit(__doc__)
    return 0


if __name__ == "__main__":
    sys.exit(main())
