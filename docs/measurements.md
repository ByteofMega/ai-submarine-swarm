# Measurements on this PC

PC: laptop with RTX 4060 (8 GB graphics memory), Ubuntu 24.04 in WSL2 (12 processor
cores and 9.7 GB memory visible), ROS 2 Jazzy, Gazebo Harmonic 8.

How to repeat a load measurement: `bash scripts/measure_load.sh "label"`.
"Gazebo CPU" is in percent of ONE processor core (100% = one core fully busy),
measured about 25 to 35 s after start, so it is a rough number.

## Simulation load (no window, 2 subs, 2026-11-15)

| Setup | Simulation speed | Gazebo CPU | Graphics memory used by Gazebo |
|---|---|---|---|
| 2 subs, each: camera, sonar, altimeter, IMU | 1.00 | 39% | none measurable |
| same, plus one DVL per sub | 1.00 | 91 to 93% (two runs) | none measurable |

**DVL cost: about 27% of one core per sub. Graphics memory: none.**

Important: Gazebo draws its sensors with `llvmpipe`. That is software drawing on
the main processor, not on the graphics card (checked in `~/.gz/rendering/ogre2.log`
and with `glxinfo -B`, which says "Accelerated: no"). So camera, sonar, altimeter
and DVL cost processor time, and the graphics card stays free for the LLM.

Not measured yet: more than 2 subs, and the load with the 3D window open.

## DVL accuracy (sub1 driven forward and down, 111 samples, 2026-11-15)

- Forward speed error: average +0.001 m/s, spread 0.014 m/s
- Sideways speed error: average -0.003 m/s, spread 0.015 m/s
- Up/down speed error: average +0.001 m/s, spread 0.006 m/s
- Configured noise: 0.01 m/s. Seabed lock on all 4 beams, at 29 to 33 m from the seabed.
- Message rate through the bridge: 7.4 per second (10 were asked for).

## LLM (Ollama on Windows, 2026-11-14 and 15)

| | qwen2.5:7b-instruct | qwen2.5:14b-instruct |
|---|---|---|
| Size when loaded | 4.8 GB, 100% on the graphics card | 10 GB, 62% graphics card / 38% main processor |
| Time per short decision | 0.5 s | 2.5 s (about 5 s for a longer answer) |
| Writing speed | 52 tokens/s | 10 tokens/s |
| Right action in 21 test decisions | 15 | 18 |

- Graphics memory with 14b loaded: about 6.8 GB of 8.2 GB.
- The simulation stayed at speed 1.00 while the LLM was answering.
- The first call while a model loads can fail with an error.
