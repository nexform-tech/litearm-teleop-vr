#!/usr/bin/env python3
# Copyright 2026 LiteArm VR teleop authors
#
# Licensed under the BSD-3-Clause license. See LICENSE for details.
"""Measure the jitter in a recorded teleoperation command stream.

Reads a rosbag2 recording of ``/joint_states`` and
``/vr_forward_controller/commands`` and answers, per joint, the questions that
separate the ways a command stream can be rough:

* how fast samples arrive, and how regular that is;
* the peak-to-peak ripple once the motion has settled;
* the fraction of consecutive samples that reverse direction. A stream that
  alternates every single sample is not "noise", it is a specific failure - the
  loop closing on the solver's own residual - and it shows up here as a fraction
  near 1.0 where a smooth stream gives a small one;
* where the energy is, from an FFT: the top three peaks with their amplitudes. A
  per-sample alternation sits at the Nyquist frequency of the stream, which for a
  100 Hz publish rate is 50 Hz, so the dominant peak frequency is the second half
  of the same diagnosis;
* for the commands, how often the velocity field sits at the ceiling the
  controller enforces (1.0 rad/s by default), and the size of a typical position
  step.

Two verdict lines are printed on purpose, because the numbers mean little
without them: whether a per-sample flip is present, and whether there is a peak
at the publish rate's Nyquist.

The thresholds are arguments, not constants, and ``--selftest`` proves that they
fire on a synthetic flip and stay quiet on a synthetic smooth signal - a detector
that has not been shown to respond is not evidence.

What the numbers do and do not cover: this reads the ROS side of the chain. The
firmware's own reference is not in the bag (it is visible in the daemon's log
counters), so a ripple measured here is the ripple the arm is being asked for,
not necessarily the ripple it performed.

Usage
-----
    python3 joint_report.py /tmp/harness/baseline_synth
    python3 joint_report.py /tmp/harness/baseline_episode --settle 8 --csv /tmp/harness/base_ep
    python3 joint_report.py --selftest
"""

import argparse
import csv
import math
import os
import sys

import numpy as np

DEFAULT_JOINTS = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6", "joint7")

# Direction reversals: a stream that flips every sample lands at 1.0; a smooth
# trajectory reverses only when the motion does, which over a settled window is
# a small fraction. Halfway is the natural dividing line and is an argument.
DEFAULT_FLIP_THRESHOLD = 0.5

# A peak counts as "at the publish rate" when it is within a fifth of the
# Nyquist frequency, i.e. above 0.4 * rate. For a 100 Hz stream that is 40 Hz and
# up, which cannot be anything but the sampling grid's own signature.
DEFAULT_NYQUIST_FRACTION = 0.4

# |v| is compared against this share of the ceiling so that a value sitting on
# the limit is counted even after a round trip through the file.
CEILING_SHARE = 0.999


# ------------------------------------------------------------------------------
# Reading
# ------------------------------------------------------------------------------


def _stamp_seconds(stamp):
    return float(stamp.sec) + 1e-9 * float(stamp.nanosec)


def read_bag(path, topics):
    """{topic: {"t": array, "names": [...], "x": (N, M), "v": (N, M) or None}}.

    The join between the two topics is done on joint NAME, not on column index:
    the joint state broadcaster publishes the gripper beside the arm and the
    order is its own, while the command is a flat array whose meaning is the
    configured joint order. Comparing column 3 of one with column 3 of the other
    would be a comparison of two different joints.
    """
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=path, storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(input_serialization_format="cdr",
                                    output_serialization_format="cdr"))
    available = {entry.name: entry.type for entry in reader.get_all_topics_and_types()}

    wanted = [topic for topic in topics if topic in available]
    missing = [topic for topic in topics if topic not in available]
    if missing:
        print("topics not in the bag: {0} (present: {1})".format(
            ", ".join(missing), ", ".join(sorted(available)) or "none"))
    classes = {topic: get_message(available[topic]) for topic in wanted}

    collected = {topic: {"t": [], "names": None, "x": [], "v": [], "source": None}
                 for topic in wanted}
    while reader.has_next():
        topic, data, received = reader.read_next()
        if topic not in collected:
            continue
        message = deserialize_message(data, classes[topic])
        entry = collected[topic]
        received_seconds = received * 1e-9
        if hasattr(message, "header"):
            header_seconds = _stamp_seconds(message.header.stamp)
            entry["source"] = "header stamp" if header_seconds > 0.0 else "receive time"
            seconds = header_seconds if header_seconds > 0.0 else received_seconds
        else:
            entry["source"] = "receive time"
            seconds = received_seconds

        if topic.endswith("joint_states"):
            if entry["names"] is None:
                entry["names"] = list(message.name)
            entry["t"].append(seconds)
            entry["x"].append(list(message.position))
            entry["v"].append(list(message.velocity) if message.velocity else
                              [float("nan")] * len(message.position))
        else:
            count = len(message.data) // 2
            if entry["names"] is None:
                entry["names"] = list(DEFAULT_JOINTS[:count])
            entry["t"].append(seconds)
            entry["x"].append(list(message.data[:count]))
            entry["v"].append(list(message.data[count:2 * count]))

    out = {}
    for topic, entry in collected.items():
        if not entry["t"]:
            continue
        out[topic] = {
            "t": np.asarray(entry["t"], dtype=float),
            "names": entry["names"],
            "x": np.asarray(entry["x"], dtype=float),
            "v": np.asarray(entry["v"], dtype=float),
            "source": entry["source"],
        }
    return out


def select(data, names):
    """Columns of `names` in order, from a topic read: (indices, present names)."""
    return [data["names"].index(name) for name in names if name in data["names"]]


# ------------------------------------------------------------------------------
# Statistics
# ------------------------------------------------------------------------------


def sample_rate(seconds):
    """(rate from the total span, rate from the median interval)."""
    if len(seconds) < 2:
        return 0.0, 0.0
    span = seconds[-1] - seconds[0]
    rate = (len(seconds) - 1) / span if span > 0 else 0.0
    interval = float(np.median(np.diff(seconds)))
    return rate, (1.0 / interval if interval > 0 else 0.0)


def reversal_fraction(values):
    """Share of consecutive samples that reverse the direction of travel.

    Zero-length steps are dropped rather than counted as a reversal: a stream
    that alternates between two values repeats neither, but a stream that holds a
    value for a while is not reversing either.
    """
    if len(values) < 3:
        return 0.0
    steps = np.diff(values)
    steps = steps[steps != 0.0]
    if steps.size < 2:
        return 0.0
    signs = np.sign(steps)
    return float(np.count_nonzero(signs[1:] != signs[:-1])) / float(signs.size - 1)


def spectrum(seconds, values, peaks=3):
    """The strongest components of a series: ([(frequency, amplitude)...], rate, extra).

    The series is resampled onto a uniform grid first. A recorded stream is not
    perfectly uniform - that irregularity is part of what is being measured - and
    an FFT of non-uniform samples smears its energy across the spectrum, which
    would hide exactly the peak this is looking for. Resampling onto the median
    interval with linear interpolation is a low-pass in itself, so a peak found
    here is a peak that survived it.

    A linear trend is removed before the transform. A joint travelling somewhere
    is a ramp, and a ramp's energy piles up in the lowest bins and outranks
    everything else; what the peaks are for is the fine structure on top of the
    motion, and the size of the trend is reported separately as the ripple.

    Missing samples are not invented: a gap larger than four median intervals
    ends the series at the gap, and the rest is still analysed.
    """
    extra = {"nyquist_amplitude": 0.0, "max_amplitude": 0.0}
    if len(values) < 32:
        return [], 0.0, extra
    interval = float(np.median(np.diff(seconds)))
    if interval <= 0.0:
        return [], 0.0, extra
    gap = np.flatnonzero(np.diff(seconds) > 4.0 * interval)
    if gap.size:
        values = values[:gap[0] + 1]
        seconds = seconds[:gap[0] + 1]
        if len(values) < 32:
            return [], 0.0, extra

    grid = np.arange(seconds[0], seconds[-1], interval)
    if grid.size < 32:
        return [], 0.0, extra
    uniform = np.interp(grid, seconds, values)
    rate = 1.0 / interval

    uniform = uniform - np.polyval(np.polyfit(grid - grid[0], uniform, 1), grid - grid[0])
    window = np.hanning(uniform.size)
    amplitudes = np.abs(np.fft.rfft(uniform * window)) * 2.0 / max(1e-12, window.sum())
    frequencies = np.fft.rfftfreq(uniform.size, d=interval)
    amplitudes[0] = 0.0  # the mean is not a peak

    order = np.argsort(amplitudes)[::-1][:peaks]
    found = []
    for index in order:
        if amplitudes[index] <= 0.0:
            continue
        found.append((float(frequencies[index]), float(amplitudes[index])))
    extra["max_amplitude"] = float(amplitudes.max()) if amplitudes.size else 0.0
    extra["nyquist_amplitude"] = float(amplitudes[int(np.argmin(np.abs(frequencies - 0.5 * rate)))])
    return found, rate, extra


def metrics(seconds, values, settle):
    """Everything reported per joint, on the settled part of one series."""
    # Sliced by index rather than by a boolean mask, so that a mismatch between
    # the two arrays cannot silently pair a sample with the wrong time.
    size = min(len(seconds), len(values))
    seconds, values = seconds[:size], values[:size]
    start = int(np.searchsorted(seconds, seconds[0] + settle)) if size else 0
    if size - start < 8:
        start = 0
    time = seconds[start:]
    series = values[start:]
    rate_span, rate_median = sample_rate(time)
    peaks, fft_rate, extra = spectrum(time, series)
    return {
        "count": int(series.size),
        "rate_span": rate_span,
        "rate_median": rate_median,
        "ripple": float(series.max() - series.min()) if series.size else 0.0,
        "std": float(series.std()) if series.size else 0.0,
        "flips": reversal_fraction(series),
        "peaks": peaks,
        "fft_rate": fft_rate,
        "nyquist_amplitude": extra["nyquist_amplitude"],
        "max_amplitude": extra["max_amplitude"],
        "nonzero_step_min": float(np.abs(np.diff(series)[np.diff(series) != 0.0]).min())
                            if np.count_nonzero(np.diff(series)) else 0.0,
    }


def step_statistics(values):
    """|step| mean / p95 / max of a series, over its whole length."""
    steps = np.abs(np.diff(values))
    if steps.size == 0:
        return 0.0, 0.0, 0.0
    return float(steps.mean()), float(np.percentile(steps, 95)), float(steps.max())


def ceiling_fraction(velocities, ceiling):
    if velocities.size == 0 or ceiling <= 0.0:
        return 0.0, 0.0
    magnitude = np.abs(velocities)
    per_sample = np.count_nonzero(magnitude >= CEILING_SHARE * ceiling, axis=1) > 0
    return float(np.count_nonzero(per_sample)) / float(velocities.shape[0]), float(magnitude.max())


# ------------------------------------------------------------------------------
# Reporting
# ------------------------------------------------------------------------------


def _table(title, rows, header):
    print("")
    print("── {0} ──".format(title))
    widths = [max(len(str(header[index])), *(len(str(row[index])) for row in rows))
              for index in range(len(header))]
    print("  ".join(str(header[index]).ljust(widths[index]) for index in range(len(header))))
    for row in rows:
        print("  ".join(str(row[index]).ljust(widths[index]) for index in range(len(header))))


def report_topic(topic, data, joints, settle, ceiling, wants_velocity):
    indices = select(data, joints)
    names = [joints[index] for index in range(len(joints)) if joints[index] in data["names"]]
    rows = []
    per_joint = []
    for name, column in zip(names, indices):
        stats = metrics(data["t"], data["x"][:, column], settle)
        peak_text = " ".join("{0:5.1f}Hz {1:.2e}".format(frequency, amplitude)
                             for frequency, amplitude in stats["peaks"]) or "-"
        mean, p95, maximum = step_statistics(data["x"][:, column])
        rows.append([name, "{0:d}".format(stats["count"]),
                     "{0:.2f}".format(stats["rate_median"]),
                     "{0:.3e}".format(stats["ripple"]),
                     "{0:.3f}".format(stats["flips"]),
                     "{0:.3e}".format(mean), "{0:.3e}".format(p95), "{0:.3e}".format(maximum),
                     peak_text])
        per_joint.append((name, stats))

    _table(topic, rows,
           ["joint", "n", "rate Hz", "p2p ripple", "flip frac", "|step| mean", "p95", "max",
            "top 3 peaks (freq, amplitude)"])

    if wants_velocity and data["v"].shape[1] >= len(joints):
        fraction, peak = ceiling_fraction(data["v"][:, :len(indices)], ceiling)
        print("  samples with some joint at the {0:.2f} rad/s ceiling: {1:.3f} "
              "(peak |v| observed {2:.4f} rad/s)".format(ceiling, fraction, peak))

    return per_joint, rows


def summary(per_joint):
    """The worst joint's figures, which is what the verdicts are decided on."""
    worst_flip = max(per_joint, key=lambda item: item[1]["flips"])
    worst_ripple = max(per_joint, key=lambda item: item[1]["ripple"])
    quiet = min(per_joint, key=lambda item: item[1]["ripple"])
    return worst_flip, worst_ripple, quiet


def verdicts(joint_states_per_joint, commands_per_joint, flip_threshold, nyquist_fraction):
    """Print and return the two verdict lines."""
    lines = []
    for label, per_joint in (("joint_states", joint_states_per_joint),
                             ("commands", commands_per_joint)):
        if not per_joint:
            continue
        flips = [(name, stats["flips"]) for name, stats in per_joint]
        worst_name, worst = max(flips, key=lambda item: item[1])
        present = worst >= flip_threshold
        lines.append(
            "{0}: per-sample flip {1} (worst joint {2}, reversal fraction {3:.3f}, "
            "threshold {4:.2f})".format(label, "PRESENT" if present else "absent",
                                        worst_name, worst, flip_threshold))

        candidates = []
        for name, stats in per_joint:
            if not stats["peaks"] or stats["fft_rate"] <= 0.0:
                continue
            frequency, amplitude = stats["peaks"][0]
            candidates.append((name, frequency, amplitude, stats))
        if candidates:
            name, frequency, amplitude, stats = max(candidates, key=lambda item: item[2])
            rate = stats["fft_rate"]
            present = frequency >= nyquist_fraction * rate
            lines.append(
                "{0}: peak at the publish rate (Nyquist) {1} (dominant peak {2:.1f} Hz at "
                "{3:.2e} on {4}; amplitude at Nyquist {5:.2e} against a largest of {6:.2e}; "
                "rate {7:.1f} Hz, Nyquist {8:.1f} Hz)".format(
                    label, "PRESENT" if present else "absent", frequency, amplitude, name,
                    stats["nyquist_amplitude"], stats["max_amplitude"], rate, 0.5 * rate))
        else:
            lines.append("{0}: peak at the publish rate (Nyquist) undecidable "
                         "(no usable spectrum)".format(label))
    print("")
    for line in lines:
        print("verdict: " + line)
    return lines


def write_csv(path, topic, data, joints):
    indices = select(data, joints)
    names = [name for name in joints if name in data["names"]]
    with open(path, "w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        header = ["t"] + names
        if data["v"].shape[1] > len(indices):
            header += ["v_" + name for name in names]
        writer.writerow(header)
        for row in range(len(data["t"])):
            values = [data["t"][row]] + [data["x"][row, column] for column in indices]
            if data["v"].shape[1] > len(indices):
                values += [data["v"][row, column] for column in indices]
            writer.writerow(["{0:.9f}".format(value) for value in values])
    return path


def run(path, args):
    topics = ["/joint_states", "/vr_forward_controller/commands"]
    data = read_bag(path, topics)
    if not data:
        print("nothing to report: the bag has neither topic")
        return 1

    print("bag    : {0}".format(path))
    print("settled: everything after t0 + {0:.1f} s (--settle)".format(args.settle))

    joint_per_joint = []
    command_per_joint = []
    if "/joint_states" in data:
        entry = data["/joint_states"]
        rate_span, rate_median = sample_rate(entry["t"])
        print("")
        print("/joint_states: {0} samples, {1:.3f} s, {2:.2f} Hz over the span, {3:.2f} Hz "
              "from the median interval (times from the {4})".format(
                  len(entry["t"]), entry["t"][-1] - entry["t"][0], rate_span, rate_median,
                  entry["source"]))
        joint_per_joint, _ = report_topic(
            "/joint_states", entry, list(args.joints), args.settle, args.ceiling, False)
        write_csv("{0}_joint_states.csv".format(args.csv), "/joint_states", entry,
                  list(args.joints))

    if "/vr_forward_controller/commands" in data:
        entry = data["/vr_forward_controller/commands"]
        rate_span, rate_median = sample_rate(entry["t"])
        print("")
        print("/vr_forward_controller/commands: {0} samples, {1:.3f} s, {2:.2f} Hz over the "
              "span, {3:.2f} Hz from the median interval (times from the {4})".format(
                  len(entry["t"]), entry["t"][-1] - entry["t"][0], rate_span, rate_median,
                  entry["source"]))
        command_per_joint, _ = report_topic(
            "/vr_forward_controller/commands", entry, list(args.joints), args.settle,
            args.ceiling, True)
        write_csv("{0}_commands.csv".format(args.csv), "/vr_forward_controller/commands",
                  entry, list(args.joints))

    for label, per_joint in (("joint_states", joint_per_joint), ("commands", command_per_joint)):
        if not per_joint:
            continue
        worst_flip, worst_ripple, quiet = summary(per_joint)
        print("")
        print("{0} summary: largest ripple {1:.3e} on {2}; smallest {3:.3e} on {4}; "
              "most reversals {5:.3f} on {6}".format(
                  label, worst_ripple[1]["ripple"], worst_ripple[0], quiet[1]["ripple"],
                  quiet[0], worst_flip[1]["flips"], worst_flip[0]))

    verdicts(joint_per_joint, command_per_joint, args.flip_threshold, args.nyquist_fraction)

    print("")
    print("csv written: {0}_joint_states.csv, {0}_commands.csv".format(args.csv))
    return 0


# ------------------------------------------------------------------------------
# Self-test: does the detector respond at all?
# ------------------------------------------------------------------------------


def _synth(kind, rate=100.0, seconds=6.0, frequency=2.0):
    count = int(rate * seconds)
    time = np.arange(count) / rate
    moving = 0.3 * np.sin(2.0 * math.pi * frequency * time)
    settled = 0.02 * time
    if kind == "smooth":
        # A joint on its way somewhere, smooth.
        return time, moving + settled
    if kind == "flip":
        # A still joint with a sample-to-sample alternation on top of it: the
        # failure this reporter exists to find, with the ripple dominating the
        # local motion the way it does when the operator's hand is not moving.
        return time, settled + 4.0e-4 * np.where(np.arange(count) % 2 == 0, 1.0, -1.0)
    if kind == "moving":
        # The same alternation under a motion 100 times larger. A ripple that is
        # invisible next to the motion is not a failure, and this case is here to
        # say so rather than to be counted as one.
        return time, moving + settled + 4.0e-4 * np.where(np.arange(count) % 2 == 0, 1.0, -1.0)
    if kind == "staircase":
        # A 100 Hz zero order hold sampled at 250 Hz: repeats, not flips.
        fine_rate = 250.0
        fine_count = int(fine_rate * seconds)
        fine_time = np.arange(fine_count) / fine_rate
        source = np.minimum((fine_time * rate).astype(int), count - 1)
        return fine_time, (moving + settled)[source]
    raise SystemExit("unknown synthetic series {0}".format(kind))


def selftest(args):
    failures = []
    cases = (
        ("smooth 2 Hz motion at 100 Hz", "smooth", False, False),
        ("still joint + per-sample flip at 100 Hz", "flip", True, True),
        ("the same flip under a 100x larger motion", "moving", False, False),
        ("100 Hz zero order hold sampled at 250 Hz", "staircase", False, False),
    )
    for label, kind, expect_flip, expect_peak in cases:
        time, values = _synth(kind)
        stats = metrics(time, values, settle=0.5)
        flips = stats["flips"]
        top_frequency = stats["peaks"][0][0] if stats["peaks"] else 0.0
        flip_seen = flips >= args.flip_threshold
        peak_seen = top_frequency >= args.nyquist_fraction * stats["fft_rate"]
        print("── {0} ──".format(label))
        print("   rate {0:.1f} Hz, ripple {1:.3e}, reversal fraction {2:.3f}, dominant peak "
              "{3:.1f} Hz".format(stats["fft_rate"], stats["ripple"], flips, top_frequency))
        print("   per-sample flip {0} (expected {1}); Nyquist peak {2} (expected {3})".format(
            "PRESENT" if flip_seen else "absent", "PRESENT" if expect_flip else "absent",
            "PRESENT" if peak_seen else "absent", "PRESENT" if expect_peak else "absent"))
        if flip_seen != expect_flip or peak_seen != expect_peak:
            failures.append(label)
        print("")

    print("── resolution ──")
    _, smooth = _synth("smooth")
    _, flip = _synth("flip")
    smooth_flips = metrics(np.arange(len(smooth)) / 100.0, smooth, 0.5)["flips"]
    flip_flips = metrics(np.arange(len(flip)) / 100.0, flip, 0.5)["flips"]
    print("   reversal fraction: smooth {0:.3f} vs flip {1:.3f} (threshold {2:.2f})".format(
        smooth_flips, flip_flips, args.flip_threshold))
    if not flip_flips > 3.0 * max(smooth_flips, 1e-6):
        failures.append("the threshold does not separate a flip from a smooth stream")

    print("")
    if failures:
        print("FAILED: {0}".format("; ".join(failures)))
        return 1
    print("OK: the detector fires on a per-sample flip and stays quiet on smooth and "
          "staircase streams")
    return 0


# ------------------------------------------------------------------------------
# Entry point
# ------------------------------------------------------------------------------


def parse_arguments(argv):
    parser = argparse.ArgumentParser(
        description="Report the jitter measured in a rosbag2 recording of the teleoperation "
                    "command path.",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("bag", nargs="?", default=None, help="rosbag2 directory")
    parser.add_argument("--settle", type=float, default=3.0,
                        help="seconds from the first sample to skip before measuring; the "
                             "return to the start pose lives in there")
    parser.add_argument("--ceiling", type=float, default=1.0,
                        help="the controller's velocity ceiling, rad/s")
    parser.add_argument("--flip-threshold", type=float, default=DEFAULT_FLIP_THRESHOLD,
                        help="reversal fraction at or above which a per-sample flip is reported")
    parser.add_argument("--nyquist-fraction", type=float, default=DEFAULT_NYQUIST_FRACTION,
                        help="share of the sample rate above which a peak counts as being at "
                             "the publish rate (Nyquist is 0.5)")
    parser.add_argument("--joints", nargs="+", default=list(DEFAULT_JOINTS),
                        help="joint names, in the order the command array uses")
    parser.add_argument("--csv", default=None,
                        help="prefix of the CSV files to write (default <bag>/joint_report)")
    parser.add_argument("--selftest", action="store_true",
                        help="check the detector on synthetic series, with no bag")
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_arguments(sys.argv[1:] if argv is None else argv)
    if args.selftest:
        return selftest(args)
    if not args.bag:
        sys.stderr.write("no bag given; pass a rosbag2 directory or --selftest\n")
        return 2
    if not os.path.isdir(args.bag):
        sys.stderr.write("no such bag directory: {0}\n".format(args.bag))
        return 2
    if args.csv is None:
        args.csv = os.path.join(args.bag, "joint_report")
    return run(args.bag, args)


if __name__ == "__main__":
    sys.exit(main())
