# Sessions on the real robot

One file per session: what was planned, what happened, the numbers. The step-by-step setup
guide is `crates/kuka-rsi/controller/BRINGUP.md`; lessons that apply to every setup move from
here into its troubleshooting table.

| Session | Date | Content |
| --- | --- | --- |
| [1](2026-11-05-session-1.md) | 2026-11-05 | First bring-up: monitor, hold, A6 45° |
| [2](2026-11-05-session-2.md) | 2026-11-05 | Single and multi-axis moves up to 30°, speed steps, streaming and fault-path plan |

`--log runs/NAME.csv` on `rsi-monitor`, `jog` and `stream_trajectory` writes every cycle
(header in `kuka_rsi::record`); `runs/` is ignored by git. Keep the logs of a session together
with its file; the results tables quote them.
