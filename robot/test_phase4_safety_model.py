#!/usr/bin/env python3

"""
Phase 4 — Speed-Aware Safety Model (calculation-only tests)

Contracts-only tests for stop distance scaling. No backend enforcement changes.

Formula:
- Reaction distance: d_r = v * t_r
- Brake distance: d_b = v^2 / (2 * a)
- Total stop distance: d = d_r + d_b + buffer_m
"""

import sys
from typing import Dict

from kilo_core.util import load_yaml, resolve_config_path, get_cfg


def stop_distance_m(velocity_mps: float, reaction_time_s: float, decel_mps2: float, buffer_m: float = 0.0) -> float:
    if decel_mps2 <= 0:
        raise ValueError("brake_decel_mps2 must be > 0")
    return buffer_m + (velocity_mps * reaction_time_s) + (velocity_mps ** 2) / (2.0 * decel_mps2)


def _assert_monotonic_increasing(seq):
    for i in range(1, len(seq)):
        assert seq[i] > seq[i - 1], f"Expected monotonic increase: {seq[i]} !> {seq[i - 1]} at index {i}"


def load_profiles() -> Dict[str, Dict[str, float]]:
    cfg_path = resolve_config_path(None, package_name="kilo_core")
    cfg = load_yaml(cfg_path)
    profiles = get_cfg(cfg, "safety_model.profiles", {}) or {}
    return profiles


def test_scaling_with_velocity(prof):
    velocities = [0.0, 0.5, 1.0, 2.0]
    buffers = [stop_distance_m(v, prof["reaction_time_s"], prof["brake_decel_mps2"], prof["buffer_m"]) for v in velocities]
    _assert_monotonic_increasing(buffers)


def test_longer_reaction_time_increases_buffer(prof):
    v = 1.0
    decel = prof["brake_decel_mps2"]
    short = stop_distance_m(v, 0.3, decel, prof["buffer_m"])
    long = stop_distance_m(v, 0.9, decel, prof["buffer_m"])
    assert long > short, f"Expected longer reaction to increase buffer: {long} !> {short}"


def test_stronger_braking_reduces_buffer(prof):
    v = 1.5
    tr_s = prof["reaction_time_s"]
    weak = stop_distance_m(v, tr_s, 1.5, prof["buffer_m"])
    strong = stop_distance_m(v, tr_s, 3.0, prof["buffer_m"])
    assert strong < weak, f"Expected stronger braking to reduce buffer: {strong} !< {weak}"


def test_profiles_ordering_reasonable(profiles):
    if not all(k in profiles for k in ("crawl", "normal", "sport")):
        return
    v = 1.2
    crawl = profiles["crawl"]
    normal = profiles["normal"]
    sport = profiles["sport"]
    d_crawl = stop_distance_m(v, crawl["reaction_time_s"], crawl["brake_decel_mps2"], crawl["buffer_m"])
    d_normal = stop_distance_m(v, normal["reaction_time_s"], normal["brake_decel_mps2"], normal["buffer_m"])
    d_sport = stop_distance_m(v, sport["reaction_time_s"], sport["brake_decel_mps2"], sport["buffer_m"])
    assert d_crawl >= d_normal >= d_sport, f"Profile ordering invalid: crawl={d_crawl}, normal={d_normal}, sport={d_sport}"


def run_all():
    profiles = load_profiles()
    prof = profiles.get("normal")
    if not prof:
        print("FAIL: safety_model.profiles.normal missing")
        sys.exit(1)

    test_scaling_with_velocity(prof)
    test_longer_reaction_time_increases_buffer(prof)
    test_stronger_braking_reduces_buffer(prof)
    test_profiles_ordering_reasonable(profiles)

    sample = stop_distance_m(1.0, prof["reaction_time_s"], prof["brake_decel_mps2"], prof["buffer_m"])
    print({"result": "PASS", "sample_normal_1.0mps": sample})


if __name__ == "__main__":
    run_all()
