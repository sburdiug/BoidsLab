pub(super) fn finite(v: [f64; 2]) -> bool {
    v.into_iter().all(f64::is_finite)
}

pub(super) fn limited(vector: [f64; 2], maximum: f64) -> [f64; 2] {
    if norm(vector) > maximum {
        scale(unit(vector).unwrap_or([0.0; 2]), maximum)
    } else {
        vector
    }
}
pub(super) fn add(a: [f64; 2], b: [f64; 2]) -> [f64; 2] {
    [a[0] + b[0], a[1] + b[1]]
}
pub(super) fn sub(a: [f64; 2], b: [f64; 2]) -> [f64; 2] {
    [a[0] - b[0], a[1] - b[1]]
}
pub(super) fn scale(a: [f64; 2], b: f64) -> [f64; 2] {
    [a[0] * b, a[1] * b]
}
pub(super) fn norm(a: [f64; 2]) -> f64 {
    a[0].hypot(a[1])
}
pub(super) fn dot(a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}

pub(super) fn unit(vector: [f64; 2]) -> Option<[f64; 2]> {
    if !finite(vector) {
        return None;
    }
    let largest = vector[0].abs().max(vector[1].abs());
    if largest == 0.0 {
        return None;
    }
    let scaled = [vector[0] / largest, vector[1] / largest];
    Some(scale(scaled, 1.0 / norm(scaled)))
}

pub(super) fn rotate(v: [f64; 2], angle: f64) -> [f64; 2] {
    if angle == 0.0 {
        return v;
    }
    let (sin, cos) = angle.sin_cos();
    [cos * v[0] - sin * v[1], sin * v[0] + cos * v[1]]
}
