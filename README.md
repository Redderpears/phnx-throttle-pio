# Phoenix Throttle, smaller this time

This repo contains the code for the throttle ESC, ported to the STMF303. It would likely work for other stm boards as
well,
so long as they have BxCAN controllers.

See the [design doc](https://github.com/ISC-Project-Phoenix/design/blob/main/software/embed/Throttle.md) for more info.

## Pinout

All with respect to labels on the nucleo PCB, _not_ the same as in code:

| PIN | Connection           |
|-----|----------------------|
| A3  | ESC Pedal input (J5) |
| D2  | CAN Trans Tx         |
| D10 | CAN Trans Rx         |

Obviously create a common ground with the ESC.
