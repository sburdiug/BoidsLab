# Research roadmap: metrics, 2D scenarios and 3D

Planning snapshot: 2026-11-10. Companion: [current issues and fixes](current-issues-and-fixes.md). This describes intended work, not implemented capabilities. English is the implementation reference; a Ukrainian overview follows.

## 1. Objective and agreed decisions

Build an inspectable swarm playground and an analysis workflow that answer: **does the group accomplish its task safely while remaining a coherent swarm, and how do parameters change that behavior?** The project should be reproducible and demonstrable beyond a course assignment.

- Keep 2D as a supported mode; add 3D as a separate experiment layer later.
- Nominal agent radius is **0.10 m**, diameter **0.20 m**. Test larger bodies separately. This default change is planned, not yet applied.
- Preserve independent groups and directed React/Avoid/Ignore policies. Ignore removes perception/response, not physical contacts.
- Retain environment presets, routes, obstacles, wind and observation noise. These are controlled experimental factors, not evidence of realistic vehicle dynamics.
- Begin with small interpretable experiments. Do not optimize an arbitrary combined score or call one configuration universally best.

## 2. Questions and metrics

### Safety

Record terminated fraction relative to the initial group population, time to first contact, agent-agent contact events and obstacle contact events separately. Simultaneous contact graphs can produce multiple events; distinguish event count, unique affected agents and any later definition of a collision episode.

For each recorded frame compute surface gap `distance(center_i, center_j) - radius_i - radius_j`, including between different groups. Report minimum and a low quantile, plus the fraction of observed time below a predefined safety margin. Define and record that margin before comparing configurations. Sampled gaps cannot prove continuous-time safety; contact events remain the physical contact evidence.

### Task completion and efficiency

- Arrival fractions and times to 50%, 90% and 100% of the original population. Terminated agents do not disappear from the denominator. Unreached thresholds are censored at the horizon, not assigned a fake arrival time.
- Gather: distance of the swarm centroid to the requested center, agent distance distribution and fraction inside the preferred area. The area is soft; overflow alone is not failure.
- Stabilization: a predefined fraction meets the spatial and low-speed criteria continuously for a predefined duration. Record thresholds; this is distinct from the engine's arrived flag.
- Path length per agent and excess relative to direct start-to-target distance, for completed trips with nonzero direct distance. Obstacles make the direct distance a lower bound, not necessarily an attainable optimum. Report survivors/completers separately to avoid survivorship bias.
- Route tracking: median/p95 distance to the route, time outside the corridor, recovery time after a detour, and progression along the intended segment. Nearest distance to any segment can hide shortcuts at self-intersections; use waypoint progress for such routes.
- Stalling: time without meaningful task progress while unfinished. Separate physically blocked or deliberately waiting agents from controller oscillation. Free flocking has no target-progress stall metric.

### Swarm cohesion and shape

- Number of connected components and fraction in the largest component among living agents. Start with an undirected geometric graph using a documented link distance. Keep that distance fixed across a comparison; do not silently equate this graph to directed perception or radio connectivity.
- Centroid and RMS spread; also a robust radius containing 90% of living agents.
- Elongation: ratio of principal spatial spreads from position covariance. Report undefined/degenerate cases explicitly for too few agents or zero transverse variance. In route scenarios also report width perpendicular to travel and length along it. This directly tests the complaint that a swarm becomes a long queue.
- Polarization over moving living agents, alongside their count and survival fraction. Report a time curve and a fixed-window mean, not only the final frame. A single remaining moving agent has polarization 1 but does not demonstrate a healthy coordinated swarm.
- Time to reunite after splitting around an obstacle: largest-component fraction recovers above a declared threshold and stays there for a declared interval.

### Robustness and runtime

Compare matched runs with/without wind or noise: deviation, contacts, task delay and recovery after leaving the affected zone. Keep seed and initial geometry comparable. These are robustness measures for this model.

Measure initialization, core stepping, serialization, Python analysis and plotting separately. Include median/p95 step time and peak memory for scaling work. Do not label end-to-end execution as core speed or GUI FPS.

Do not report battery use, physical energy or realistic actuator effort: the current engine has no validated model for these quantities. Heading changes and path length may be labeled geometric control proxies only.

### Minimum useful next metric set

Implement safety/termination, task-time applicability, largest-component fraction, elongation and route corridor occupancy first. Add richer robustness/runtime statistics only when their scenarios are ready. This keeps the initial dashboard/report interpretable.

## 3. Parameter experiments: the sliders we want to study

The following values are proposed pilot ranges, not calibrated limits or accepted optimal settings.

| Factor | Pilot values | Keep explicit |
| --- | --- | --- |
| Speed | 0.5, 1, 2 m/s | dt, body radius, view radius, scene scale |
| Body radius | 0.10, 0.20, 0.50 m | Diameters 0.20, 0.40, 1.00 m; spawn feasibility |
| Cohesion weight | 0, 1, 3, 6 | Separation and alignment fixed initially |
| Alignment weight | 0, 1, 3 | Same initial seeds and survival reporting |
| Separation weight | 5, 10, 20 | Separation activation distance and body size |
| Separation distance | 1.5, 2, 3 body diameters | View radius must cover the chosen distance |
| View radius | 1, 2, 4 m where valid | Must not violate separation/view constraints |
| Population | 10, 20, 50 first | Separate fixed-area and fixed-density studies |
| Route width / opening width | Relative to body diameter | Report physical clearance as well as ratio |

First change one factor at a time to learn its effect. Then investigate likely interactions: speed × view radius, body size × opening width, cohesion × separation, population × density. Do not run the full Cartesian product above.

For body size, distinguish two questions: (a) larger agents in the same scene, with reduced clearance and increased occupancy; (b) a geometrically scaled scene with comparable size ratios. Do not attribute every observed change in (a) to controller behavior alone.

Use the same seed set for paired comparisons. Three seeds are a pilot; after estimating variability, expand the selected comparisons (for example to ten or more), rather than pretending three establish a universal ranking. Predefine horizon and task criteria. Changing dt changes the effective per-step heading response, so it is not automatically a pure numerical accuracy experiment.

## 4. The 2D scenario sequence

| Scenario | Main question | Main metrics |
| --- | --- | --- |
| Free flocking, varying density | Does the swarm form, fragment or collide? | Polarization, components, elongation, losses |
| Gather in open space | Does it settle near the center without crowding? | Arrival times, spatial spread, center error, contacts |
| One rotated rectangle | Does it split and reunite? | Detour length, contacts, reunion time, shape |
| Narrow opening between rectangles | Is the opening passable and how does a queue form? | Throughput, waiting, elongation, losses |
| Bent wide route with an obstacle | Does the swarm retain width and regain its corridor? | Corridor occupancy, progress, reunion/recovery |
| Head-on groups | What does asymmetric perception cost each group? | Per-group losses, delays, minimum inter-group gaps |
| Local wind patch | Can it recover after displacement? | Drift, delay, contacts, recovery |
| Observation noise, separately | How sensitive are decisions to measurement error? | Variability, contacts, stalled time |

### Head-on groups: mandatory directed comparison

Create equal groups on opposite sides and mirrored routes or targets. Keep body sizes, speeds, populations, spacing and seed pairs fixed within each comparison.

1. A Avoid B, B Avoid A: mutual avoidance baseline.
2. A Ignore B, B Avoid A: A does not perceive B; B bears the avoidance burden.
3. A Avoid B, B Ignore A: swap the role, and also mirror the layout to expose geometric bias.
4. A Ignore B, B Ignore A: physical-contact control. Collisions remain enabled and are not automatically a software bug.
5. React/React as a separate social-response case; social cohesion can change the task beyond avoidance.

Specify whether policies apply only to the other group or also to obstacle response. The existing Ignore policy concerns a selected group pair; it does **not** mean that the agent ignores its own group, goals and every obstacle. Preserve that distinction in UI labels and experiment descriptions.

Report each group's outcome independently and the pair's aggregate result. A safe total can hide one group waiting indefinitely. Measure crossing delay and the duration of low progress, not just the final number of survivors.

### Narrow passage definition

Measure opening width between physical obstacle surfaces. Compare it to the diameter plus the desired margins on both sides. Include a clearly passable opening, a constrained opening and a deliberately impossible opening. For the last, the question is whether the agents stop or collide; do not demand successful traversal.

Throughput is the number of distinct living agent UIDs crossing an exit line in the intended direction per unit time; avoid counting oscillating recrossings multiple times. Observe behavior after exit as well as inside the bottleneck.

## 5. Dedicated 3D layer

### 5.1 Why add 3D?

The research question changes when agents can separate vertically: can they avoid head-on traffic or obstacles more safely, does the swarm split into altitude layers, and when does additional freedom increase dispersion? A 3D view of unchanged planar physics does not answer these questions.

Keep 2D experiments as a baseline and regression suite. 3D should be an independently selectable mode with its own scenarios and report comparisons. Do not remove the usable 2D playground.

### 5.2 First 3D model

- Position and velocity are 3D vectors in metres and metres per second. Use a documented right-handed coordinate system with Z up.
- Agent body is initially a sphere with configurable radius, base 0.10 m. The sphere is a collision envelope, not a detailed airframe.
- Free Boids retains separation, alignment and cohesion using 3D distances and vectors. Initial perception is spherical and omnidirectional. Directed group policies retain their meaning.
- Controlled motion uses 3D target guidance, avoidance and stopping. Initially allow equal directional capability, explicitly as an agent simplification. Do not add aircraft banking, gravity or actuator dynamics implicitly.
- First obstacles are simple boxes with explicit dimensions; start axis-aligned if that gives a verifiable model sooner. Avoidance must choose a feasible 3D detour, and swept contacts must detect agents/boxes between recorded positions.
- A target is a soft sphere; a route is a polyline with a tubular corridor. Width means full diameter, radius means half-width, consistently across UI, CLI and output.
- Add explicit altitude bounds and ground contact policy before terrain scenarios. An unbounded volume is a separate scenario option; do not let agents evade every wall by climbing without limit in a supposedly bounded test.

### 5.3 Architecture and data boundaries

Preserve one headless engine entry point per mode and keep rendering out of model calculations. Share orchestration, event concepts, units and analysis where appropriate. Do not force a generic dimension abstraction before the 3D geometry requirements are understood; also avoid copying the whole application into two diverging products.

Before implementation, decide which types are dimension-specific (vectors, obstacle geometry, route queries, collision broad phase) and which remain shared (UIDs, group policies, run provenance, CLI execution and reports). Extend the export with explicit dimension/model version and documented XYZ vectors. A 2D parser must reject an unsupported 3D record rather than silently dropping Z.

Presets need a dimension tag and versioned geometry. Loading a 2D preset in 3D must be rejected or use a deliberately selected conversion with defined obstacle heights and spawn altitude. No automatic unlimited-height extrusion by accident. Keep run UID scope and contact event semantics consistent.

### 5.4 3D interface and observability

Provide orbit/pan/zoom, a ground grid, scale indicator and altitude reference. Selection must distinguish overlapping projections. Show selected agent radius, XYZ position and velocity, plus optional perception sphere and route corridor.

A single perspective image is insufficient for analysis. Export synchronized XY, XZ and YZ projections, altitude-versus-time plots and optional 3D trajectories. Add altitude slicing or selected-group isolation before increasing visual density. Headless runs must work without opening a 3D window.

### 5.5 First 3D scenarios

1. One agent in an empty volume: straight-line movement, stopping and units.
2. Free swarm in a bounded study volume with explicit boundary policy: cohesion and contacts.
3. A box between the swarm and its target: compare lateral and vertical detours; bound the available altitude.
4. Two head-on groups at the same altitude, then at different initial altitudes: mutual versus asymmetric avoidance.
5. A box opening or tunnel: verify actual 3D passability and contacts with floor/ceiling, not only lateral walls.
6. A rising/falling tubular route: progress, spread and recovery after leaving the corridor.

Only after these are verified: a simple analytic hill, then a height map. A terrain format must specify horizontal scale, elevation units, origin, resolution and the interpretation between samples. Terrain collision/clearance and nearest-surface queries are model work, not just mesh rendering. Overhangs require a more general representation than a height map and are outside the first terrain milestone.

### 5.6 Metrics specific to 3D

Reuse survival, contacts, task times, 3D path length and component metrics. Add altitude range/variance, vertical separation at encounters, time spent near altitude bounds and clearance above terrain. Use three covariance eigenvalues to distinguish a compact volume, a flat layer and a long filament; do not reuse the 2D elongation label without specifying its 3D definition.

Measure perpendicular distance to a 3D route and occupancy of its tube. An XY projection can show apparent overlaps despite safe vertical separation; use full 3D geometry for safety metrics. Vertical distance traveled is a geometric measure, not battery expenditure.

### 5.7 Fair comparison and acceptance gates

First verify a planar 3D restriction: Z fixed, vertical motion disabled, and equivalent obstacle geometry should reproduce the defined 2D baseline within an agreed numerical tolerance. Unconstrained 3D is a different experiment.

Equal agent count in a plane and a volume is not equal density. Record available area/volume, body size, occupied fraction, neighbor counts and bounds. State which dimensionless ratios are matched. Compare matched seed sets, but do not claim the same integer seed necessarily produces equivalent spatial samples across dimensions.

3D acceptance requires analytic one-agent checks, pair and box contact tests (including high-speed tunneling), deterministic repeated runs in the same environment, route/target checks and an explicit validation of altitude boundaries. Then require the full path: CLI experiment → validated trace → metrics → projections/report. A visually attractive 3D animation alone is not completion.

## 6. Delivery order

1. Apply the [priority fixes](current-issues-and-fixes.md), including the approved body default.
2. Add cohesion/shape metrics and clear task/safety summaries.
3. Run targeted 2D slider studies and head-on/narrow-passage scenarios; interpret failures.
4. Finalize the 3D model/export decisions and implement the simple-volume milestone.
5. Add a separate 3D experiment/report layer; only then consider terrain.

No implementation or new experimental result is asserted by this roadmap. Parameter ranges, thresholds and the 3D implementation shape are proposals to validate; the 0.10 m base radius and preserving 2D are confirmed user decisions.

## Українська вижимка: мета, метрики та сценарії

Хочемо перевіряти не лише прибуття, а чи виконує група задачу безпечно й залишається роєм. Базовий радіус погоджено: **0,10 м**, діаметр **0,20 м**; більші розміри досліджуємо окремо.

Найважливіші нові метрики: частка найбільшого зв'язаного скупчення, кількість скупчень, витягнутість, втрати й контакти, час прибуття та стабілізації, вихід із коридору й відновлення після обходу. Поляризацію показуємо разом із кількістю живих рухомих агентів. Загибель не повинна штучно покращувати оцінку рою.

Починаємо з керованих змін швидкості, cohesion/alignment/separation, радіуса тіла й огляду. Потім перевіряємо поєднання: швидкість × огляд, розмір тіла × ширина проходу, згуртування × розділення. Не перебираємо всі значення одночасно. Число агентів і щільність — окремі фактори.

Перші нові сцени: обхід прямокутника з повторним об'єднанням, вузький прохід, ламаний маршрут із перешкодою та зустріч двох груп. Для зустрічі порівнюємо Avoid/Avoid, Ignore/Avoid, зворотні ролі та Ignore/Ignore. Політика Ignore стосується конкретної іншої групи, а не автоматично всього світу. Контакти залишаються фізичними. Окремо дивимося затримки й втрати кожної групи.

## Українська вижимка: як бачимо 3D

**2D зберігаємо; 3D додаємо як окремий режим і окремий шар досліджень.** Мета — з'ясувати, що змінює можливість вертикального розходження, а не просто перемалювати плоску симуляцію об'ємною камерою.

Перша версія: XYZ-координати, Z угору, сферичні агенти, прості коробки-перешкоди, сферична ціль і маршрут-труба. Правила Boids і напрямлені політики груп зберігаються, але відстані, огляд, уникнення й контакти стають тривимірними. Реалістичну динаміку коптерів, гравітацію та витрату батареї не додаємо неявно.

Потрібні чіткі межі висоти й правила контакту із землею: інакше агент зможе нескінченно підніматися над будь-якою стіною. CLI має працювати без графічного вікна. Формат даних і пресетів отримує ознаку розмірності; 2D-дані не перетворюються на 3D мовчки.

Початкові перевірки: один агент, вільний рій, обліт коробки збоку/зверху, зустріч груп на одній і різних висотах, тунель, маршрут із підйомом. Лише після цього — проста гора, потім карта висот. Для ландшафту треба визначити масштаб, висоти, початок координат і реальну геометрію контакту.

У звіті потрібні проєкції XY/XZ/YZ та висота в часі. Контакти оцінюємо в 3D: перетин на проєкції ще не означає зіткнення. Додаємо вертикальне розділення, діапазон висот, запас над землею та форму рою за трьома головними напрямками.

Для чесного порівняння спочатку перевіряємо плоский режим 3D із зафіксованим Z проти 2D. Однакова кількість агентів у площині та об'ємі не означає однакової щільності. Повний критерій готовності: коректна модель, контрольні тести, CLI-запуск, перевірений експорт і аналітичний звіт — не тільки анімація.
