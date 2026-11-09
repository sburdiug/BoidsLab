use super::{Agent, Config, SimulationError, math::*};

pub(super) fn actuate(
    agent: Agent,
    desired: [f64; 2],
    config: Config,
) -> Result<([f64; 2], [f64; 2]), SimulationError> {
    let error = sub(desired, agent.velocity);
    let limit = if dot(agent.velocity, error) < 0.0 {
        config.max_braking
    } else {
        config.max_acceleration
    };
    let jerk_step = config.max_jerk * config.dt;
    let requested_magnitude =
        ((jerk_step * jerk_step + 2.0 * config.max_jerk * norm(error)).sqrt() - jerk_step)
            .max(0.0)
            .min(limit)
            .min(norm(error) / config.dt);
    let request = scale(unit(error).unwrap_or([0.0; 2]), requested_magnitude);
    let base = sub(agent.acceleration, limited(agent.acceleration, jerk_step));
    let candidate = add(
        agent.acceleration,
        limited(sub(request, agent.acceleration), jerk_step),
    );
    let feasible = |acceleration: [f64; 2]| {
        let magnitude = norm(acceleration);
        let bound = if dot(agent.velocity, acceleration) < 0.0 {
            config.max_braking
        } else {
            config.max_acceleration
        };
        let ramp_velocity = scale(
            acceleration,
            config.dt + magnitude / (2.0 * config.max_jerk),
        );
        let braking = magnitude <= config.max_acceleration
            || dot(agent.velocity, acceleration) >= 0.0
            || norm(ramp_velocity)
                <= -dot(agent.velocity, unit(acceleration).unwrap_or([0.0; 2])) + 1e-12;
        finite(ramp_velocity)
            && magnitude <= bound + 1e-10
            && braking
            && norm(add(agent.velocity, ramp_velocity)) <= config.speed + 1e-12
    };
    if !feasible(base) {
        return Err(SimulationError::NumericOverflow);
    }
    let mut acceleration = base;
    let mut fraction = 1.0;
    for _ in 0..20 {
        let attempt = add(base, scale(sub(candidate, base), fraction));
        if feasible(attempt) {
            acceleration = attempt;
            break;
        }
        fraction *= 0.5;
    }
    let velocity = add(agent.velocity, scale(acceleration, config.dt));
    if !finite(velocity) || !finite(acceleration) {
        return Err(SimulationError::NumericOverflow);
    }
    Ok((velocity, acceleration))
}
