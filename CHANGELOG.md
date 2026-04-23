# Changelog

## 1.0.0 — 2026-04-23

### Breaking

- `/vessel` body-frame quantities now publish in aerospace FRD (right-handed,
  `x=forward`, `y=right`, `z=down`) instead of KSP's native vessel frame
  (left-handed, `x=right`, `y=forward`, `z=down`).

  Transformed fields:
  - `moment_of_inertia` — `(y, x, z)` permute (`x=roll`, `y=pitch`, `z=yaw`).
  - `inertia` tensor — `ixx<->iyy`, `ixz<->iyz` (xy, zz invariant).
  - `rotation` — `(x, y, z, w) → (y, x, z, -w)`.
  - `angular_velocity_body` — `(y, x, z)` permute.

  Fields in the active reference frame (kerbin celestial-body frame by
  default) stay in kRPC's native left-handed frame:
  - `/vessel.position`, `.velocity`, `.direction`, `.angular_velocity`
  - `/vessel/flight.*`, `/vessel/orbit.*`, `/celestial_bodies.*`

  Control scalars (`/vessel/control.pitch`, `.yaw`, `.roll`, `/set_sas`) are
  unchanged — they are named-axis scalars matching aerospace sign conventions.

  Downstream consumers that integrate `angular_velocity_body` or read
  `moment_of_inertia` by named axis must drop any prior `(x=pitch, y=yaw,
  z=roll)` indexing and adopt `(x=roll, y=pitch, z=yaw)`.

### Known gaps

- `/vessel/parts` body-frame fields (part positions, rotations, inertia) are
  still published in KSP-native LH. Scheduled for a follow-up work package.
