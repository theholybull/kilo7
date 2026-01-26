# Lessons Learned (bullets only)

- Bootstrap deadlocks can arise when hardware defaults to fail-closed (relay KILL) and application layers mirror that state without a release path. Resolve by conditioning hardware release on authoritative gate truth (`safe_to_move`) and known lock cause (`RELAY_KILLED`).
- Keep authority surfaces minimal: adding a bootstrap release in `relay_kill` must not introduce a new motion authority; it should react to existing truth (`/kilo/state/control_json` caches) and remain supporting.
- Observability matters: a small `echo_json_once.py` helper reduced ambiguity vs truncated `ros2 topic echo` output and sped up verification.
- Active-LOW semantics require explicit documentation and probes (GPIO17 LOW=RUN, HIGH=KILL). Confirm with hardware-level checks during end-to-end tests.
- Separate STOP latch clearing (`CLEAR_STOP`) from UNLOCK (`override` intent). This makes operator actions auditable and matches field expectations.
