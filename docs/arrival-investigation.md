# Dense gather investigation

Status: open. This note records a reproducible model limitation, not a corrected controller. It compares behavior and analysis using the `boids-2d-v13` model, schema 2, with 0.10 m body radius. The Rust movement rules were not changed during this investigation.

## Observation

For the existing pilot matrix, each case starts 20 agents near (-4, 0), with seed 7, 42 or 99, body radius 0.10 m, speed 1 m/s and a 1 m soft target/corridor radius. At the original horizon of 20 s all nine controlled runs were process-valid and had zero terminated agents, but none reached 90% arrived. Extending exactly those runs to 100 s gave:

| Scenario | Seed 7 arrived/moving | Seed 42 arrived/moving | Seed 99 arrived/moving | Terminated |
| --- | --- | --- | --- | --- |
| Gather in open space | 16 / 4 | 17 / 3 | 15 / 5 | 0 |
| Gather around rectangle | 16 / 4 | 15 / 5 | 16 / 4 | 0 |
| Follow bent corridor | 17 / 3 | 15 / 5 | 16 / 4 | 0 |

All three seeds in each scenario attained 50% but not 90% by 20 s. The 100 s outcomes are state counts at the horizon, not a proof that an earlier threshold was never reached. In the open-space seed-7 run the four still-moving agents are 0.31–0.70 m from the target center at 100 s. With positions sampled every second during seconds 90–100, their accumulated sampled path lengths are approximately 0.18–0.78 m, versus net displacements 0.008–0.203 m. All have a reported final speed of 1 m/s. These samples indicate repeated local motion with little net progress; they do not reconstruct motion between sampled positions.

The existing `local_goal` tries to select a locally free parking position, while cached local targets, spacing requirements, safety steering and the settling timer interact. The exact cause of each agent's oscillation has not been isolated. A simple candidate that allowed earlier parking with a reduced clearance produced only 13 arrived for seed 7 at 100 s versus the original 16, so that candidate was discarded. No arrival state was artificially assigned to inflate completion.

## Reproduce

From the repository root, build the current executable and run the open-space control:

```powershell
cargo build --release --locked --bin simulation
./target/release/simulation.exe --steps 1000 --dt 0.1 --group -4 0 20 1 7 boids --speed 1 --turn-response 0.1 --neighbor-radius 1 --separation-radius 0.3 --body-radius 0.1 --separation-weight 10 --alignment-weight 1 --cohesion-weight 3 --command-weight 15 --obstacle-clearance 0.15 --obstacle-weight 1 --route-centering 0.25 --route-cohesion 1 --gather 4 0 1
```

For a trace, add `--trace work/analysis/gather-trace.jsonl --trace-every 10`, using a path that does not already exist. The original 18 configurations are in `analysis/experiments/first_experiments.json`. New analysis including connectivity, elongation and corridor occupancy was saved separately under `work/analysis/cohesion-v3/`; historical results remain untouched.

Free Boids is a different case: its three social rules and heading response are not a predictive collision-avoidance guarantee. Collisions are expected to remain possible and should not be hidden by declaring them an implementation error. Investigate avoidance extensions as an explicit model/scenario choice, separately from classical Boids.

## Next correction to investigate

Build a small deterministic reproduction with a few agents and inspect local targets, nearest-neighbor clearance, velocity and progress per tick. Evaluate a candidate on matched seeds, open space, obstacles and routes, including already-safe single-agent and sparse-group behavior, physical contacts and near-boundary cases. A genuine fix should reduce persistent oscillation without forcing arrivals, violating separation or changing Free Boids. If movement rules change, increment the model version and document compatibility with previous results.

## Українська вижимка

Проблема залишається відкритою. Для груп із 20 агентів через 100 с у дев'яти керованих запусках 3–5 агентів і далі рухаються; загиблих немає. У прикладі Gather вони вже близько до центру, але роблять локальні рухи з малим підсумковим переміщенням. Проста спроба дозволити раніше зупинятися погіршила результат, тому її відхилено. Потрібен окремий малий відтворюваний тест і перевірка запропонованого виправлення без штучного встановлення стану прибуття. Столкнення у класичному Free Boids можливі за визначенням поточної моделі; це окреме питання від стабілізації.
