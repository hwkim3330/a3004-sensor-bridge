# Ports, and who owns each one

Every port in this kit, in one place, because they were not.

Three daemons had disagreed about where the motion command path lives, and
none of it had failed yet only because two of them are off by default and had
never run beside the mapper:

- `agx-cmd` listened on **7602**, which is the ring. Two daemons wanting one
  port, and a lidar ring arriving at something that would read it as a malformed
  command.
- `teleop` forwarded to **7720**, where nothing listens at all.
- `navigate` sent TELE to **7721**, which is `teleop` - and `teleop` speaks
  `TCMD`, not `TELE`, so every frame would have been rejected as malformed.

The map below is the one to change. Change it here and in the config it names,
together.

## The map

| port | proto | owner | who talks to it | payload |
|---|---|---|---|---|
| 80 | TCP | uhttpd | tablet, browser | dashboard, status JSON, `map.s2mp` |
| 7502 | UDP | ouster-edge | the sensor | raw lidar, 4352–12544 B |
| 7602 | UDP | slam2d / navigate / the tablet | ouster-edge | the ring, `OSED`, 1100 B |
| 7603 | TCP | ouster-edge | dashboard | ring as Server-Sent Events |
| 7604 | UDP | navigate | tablet, console | `GOAL x_cm y_cm`, `ROUTE x y ...`, `STOP` |
| 7605 | UDP | slam2d | tablet, console | `SAVE <path>`, `LOAD <path>`, `RESET` |
| 7701 | UDP | can-bridge | a host that may inject | `BCAN` frame datagrams |
| 7721 | UDP | teleop | tablet, console | `TCMD`, 24 B — operator intent |
| **7722** | UDP | agx-cmd | teleop, navigate | `TELE`, 32 B — motion command |
| 7723 | UDP | anything watching | teleop | `TELE`, 32 B — a broadcast copy of what teleop accepted |
| 8080 | TCP | ustreamer | tablet, browser | MJPEG |
| 8082 | TCP | mic-stream | tablet | PCM |
| 8083 | TCP | teleop | browser | its own control page |
| 8090 | TCP | webconsole, **on the pc** | browser | the console page, `/ws`, `/camera.mjpg` |

## Two magics, two shapes, one direction

`TCMD` and `TELE` are not the same frame and the difference is deliberate.

`TCMD` is 24 bytes and carries what a person is asking for. It comes from a
tablet or a browser, over WiFi, and `teleop` is the thing that decides whether
to believe it - sequence numbers, a deadman, an armed flag.

`TELE` is 32 bytes and carries a motion command that something downstream will
turn into CAN. `agx-cmd` is the only consumer, and it has its own arming and its
own ramp-down. Two sources produce it: `teleop`, translating a human, and
`navigate`, deciding for itself.

Keeping them distinct is what makes "autonomy is a new source of intent, not a
new path to the motors" true rather than aspirational: `navigate` cannot
accidentally be mistaken for an operator, and an operator's frame cannot skip
`teleop`'s deadman.

## The tablet side

`app/src/main/java/re/keti/a3004bridge/Wire.kt` repeats these numbers and
`WireTest` pins them. It is the same contract in another language, so when a
port moves here it moves there in the same commit or the panel that depends on
it goes quietly blank.
