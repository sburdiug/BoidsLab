# Dense gather investigation

Status: the reproduced blocked-parking behavior was corrected in `boids-2d-v14` (schema 2). The measurements below record the prior `boids-2d-v13` behavior with a 0.10 m body radius; they remain historical and should not be relabeled.

In v14, a cached destination is re-evaluated when a visible parked agent blocks the path. A neighbor behind the direction of travel does not block moving away. If the available candidates remain blocked and an agent is already near the gathering region, it may choose its current position when physical clearance is available. This is a fallback to a safe local hold request, not a direct change to `Arrived`: the existing settling duration and physical contact logic still decide the outcome. The desired separation distance is not a hard constraint; the fallback can hold with a smaller gap. The rules are unchanged for Free Boids.

The nine cases in the table were repeated with the same seeds and parameters for 100 s: all 20 agents reached `Arrived` and none terminated in each run. A Rust regression also checks zero final velocities and nonoverlapping bodies. This verifies these specific cases, not all group sizes, densities, noise, wind, geometries or parameter settings. Earlier v13 data remain unchanged. No weights were tuned to select a preferred swarm shape.

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

From the repository root, build the current executable and run the open-space control. The current build produces v14 results; use the earlier v13 revision to reproduce the historical counts:

```powershell
cargo build --release --locked --bin simulation
./target/release/simulation.exe --steps 1000 --dt 0.1 --group -4 0 20 1 7 boids --speed 1 --turn-response 0.1 --neighbor-radius 1 --separation-radius 0.3 --body-radius 0.1 --separation-weight 10 --alignment-weight 1 --cohesion-weight 3 --command-weight 15 --obstacle-clearance 0.15 --obstacle-weight 1 --route-centering 0.25 --route-cohesion 1 --gather 4 0 1
```

For a trace, add `--trace work/analysis/gather-trace.jsonl --trace-every 10`, using a path that does not already exist. The original 18 configurations are in `analysis/experiments/first_experiments.json`. New analysis including connectivity, elongation and corridor occupancy was saved separately under `work/analysis/cohesion-v3/`; historical results remain untouched.

Free Boids is a different case: its three social rules and heading response are not a predictive collision-avoidance guarantee. Collisions are expected to remain possible and should not be hidden by declaring them an implementation error. Investigate avoidance extensions as an explicit model/scenario choice, separately from classical Boids.

## Verification and remaining limits

The correction was checked against the original matched-seed cases, existing single-agent and obstacle tests, and the full Rust test suite. A focused test checks that a parked neighbor blocks motion toward it but does not block motion away. The engine still requires physical clearance and a settling interval; it does not guarantee preferred separation or collision-free behavior under arbitrary circumstances. Broader parameter and population comparisons remain separate experiments.

## Українська вижимка

Для перевіреного випадку виправлено зациклення біля заблокованої точки. Модель v14 перевіряє шлях до збереженої локальної цілі та може обрати поточну позицію, якщо шлях перекритий і є фізичний зазор. Стан прибуття, як і раніше, визначається після стабілізації та перевірки контактів. У дев'яти контрольних запусках по 20 агентів усі зупинилися без загиблих; це не гарантія для інших параметрів. Бажана дистанція розділення є м'якою умовою. Класичні Free Boids не змінено, старі результати v13 збережено.
