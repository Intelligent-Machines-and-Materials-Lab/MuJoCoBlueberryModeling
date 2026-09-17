#!/usr/bin/env python3
"""
Written by Claude 6/25/26
extract_topics_to_csv.py

Extracts /blueberry_dc/load_cell_data (std_msgs/Float64MultiArray) and ALL
THREE accelerometer topics -- /accel0, /accel1, /accel2 (geometry_msgs/
Vector3) -- from one or more ROS2 bag folders, and writes one separate,
clean CSV per topic per bag (4 files per bag total), each at its own
native sample rate with no blank/NaN rows. This keeps each signal
suitable for FFT or other signal-processing analysis, since interleaving
different-rate signals into one file would introduce gaps that corrupt
frequency content.

Every accelerometer is extracted for every bag/trial -- there is no
branch-to-topic guessing or filtering. This was a deliberate simplification
after finding that mapping branch number -> specific accel topic by
filename was not reliably consistent across trials; pulling all three
every time avoids that risk entirely.

All output CSVs for a given bag share the same bag_timestamp_ns clock
(ROS bag recording time, nanoseconds since epoch), so they can still be
re-synced/overlaid on a shared time axis afterward (e.g. for plotting)
just by loading the files you need and using that shared timestamp
column directly -- no merging or interpolation needed for that step.

Requires a sourced ROS2 environment (rclpy, rosbag2_py, rosidl_runtime_py).
Run this on the machine where ROS2 is installed -- NOT in this sandbox.

USAGE
-----
Single bag:
    python3 extract_topics_to_csv.py /path/to/bag_folder

Many bags (all immediate subfolders of a parent directory that each
contain a .db3 + metadata.yaml):
    python3 extract_topics_to_csv.py /path/to/parent_dir --batch

Custom output directory:
    python3 extract_topics_to_csv.py /path/to/parent_dir --batch -o /path/to/output

OUTPUT
------
For each bag folder named e.g. "bush_4_branch_2_trial_3_height_235_0",
writes FOUR files:

  bush_4_branch_2_trial_3_height_235_0_load_cell.csv
    bag_timestamp_ns       - ns since epoch, this topic's own samples only
    load_cell_field_0 .. N - every element of the load_cell Float64MultiArray
    load_cell_timestamp    - alias for load_cell_field_1 (embedded "timestamp" value)
    load_cell_reading      - alias for load_cell_field_2 (the load cell reading)
    actuator_displacement  - alias for load_cell_field_3 (the actuator displacement)

  bush_4_branch_2_trial_3_height_235_0_accel0.csv
    bag_timestamp_ns, accel0_x, accel0_y, accel0_z

  bush_4_branch_2_trial_3_height_235_0_accel1.csv
    bag_timestamp_ns, accel1_x, accel1_y, accel1_z

  bush_4_branch_2_trial_3_height_235_0_accel2.csv
    bag_timestamp_ns, accel2_x, accel2_y, accel2_z

Each file is sorted by bag_timestamp_ns and contains ONLY that topic's
real, recorded samples -- no blanks, no NaNs, no interpolation. To
overlay multiple signals on one time axis later, load the relevant CSVs
and convert bag_timestamp_ns to seconds (or subtract the bag's start
time) in each; since all columns come from the same clock they'll
already line up.
"""

import argparse
import csv
import sys
from pathlib import Path

try:
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError:
    print(
        "ERROR: Could not import ROS2 Python packages (rosbag2_py, rclpy, "
        "rosidl_runtime_py).\n"
        "This script must be run in a sourced ROS2 environment, e.g.:\n"
        "    source /opt/ros/<distro>/setup.bash\n"
        "    python3 extract_topics_to_csv.py ...\n",
        file=sys.stderr,
    )
    sys.exit(1)


LOAD_CELL_TOPIC = "/blueberry_dc/load_cell_data"

# Every accelerometer is extracted for every bag/trial -- no branch-based
# filtering. (Branch->topic mapping by filename was tried and found to be
# unreliable across trials, so we just grab all three every time instead.)
ACCEL_TOPICS = ["/accel0", "/accel1", "/accel2"]


def detect_storage_id(bag_dir: Path) -> str:
    """Guess the storage_id rosbag2 needs based on file extensions present."""
    if list(bag_dir.glob("*.db3")):
        return "sqlite3"
    if list(bag_dir.glob("*.mcap")):
        return "mcap"
    raise FileNotFoundError(
        f"No .db3 or .mcap file found in {bag_dir} -- is this a valid bag folder?"
    )


def read_bag_topics(bag_dir: Path, topics_of_interest):
    """
    Reads a single ROS2 bag folder and returns a dict:
        {topic_name: [(timestamp_ns, deserialized_msg), ...]}
    Only for topics in topics_of_interest that are actually present in the bag.
    """
    storage_id = detect_storage_id(bag_dir)
    storage_options = StorageOptions(uri=str(bag_dir), storage_id=storage_id)
    converter_options = ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )

    reader = SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topic_types}

    missing = [t for t in topics_of_interest if t not in type_map]
    if missing:
        print(
            f"  WARNING: topic(s) not found in {bag_dir.name}: {missing} "
            f"(skipping these for this bag)"
        )

    msg_type_cache = {
        t: get_message(type_map[t]) for t in topics_of_interest if t in type_map
    }

    results = {t: [] for t in msg_type_cache}

    while reader.has_next():
        topic, data, timestamp = reader.read_next()
        if topic in msg_type_cache:
            msg = deserialize_message(data, msg_type_cache[topic])
            results[topic].append((timestamp, msg))

    return results


def load_cell_rows(messages):
    """
    Converts a list of (timestamp_ns, Float64MultiArray msg) into row dicts.
    Exports every array element as load_cell_field_<i>, plus named aliases
    for indices 1, 2, 3 per the user's spec (0-indexed).
    """
    rows = []
    for ts, msg in messages:
        data = list(msg.data)
        row = {"bag_timestamp_ns": ts}
        for i, val in enumerate(data):
            row[f"load_cell_field_{i}"] = val
        # Named aliases for the three fields of interest (0-indexed as confirmed)
        row["load_cell_timestamp"] = data[1] if len(data) > 1 else ""
        row["load_cell_reading"] = data[2] if len(data) > 2 else ""
        row["actuator_displacement"] = data[3] if len(data) > 3 else ""
        rows.append(row)
    return rows


def accel_rows(messages, column_prefix="accel"):
    """Converts a list of (timestamp_ns, Vector3 msg) into row dicts.
    column_prefix lets the output columns reflect whichever accel topic
    was actually used (e.g. 'accel0', 'accel1', 'accel2'), so the CSV is
    self-documenting about which sensor the data came from."""
    rows = []
    for ts, msg in messages:
        rows.append(
            {
                "bag_timestamp_ns": ts,
                f"{column_prefix}_x": msg.x,
                f"{column_prefix}_y": msg.y,
                f"{column_prefix}_z": msg.z,
            }
        )
    return rows


def write_load_cell_csv(load_cell_msgs, output_path: Path):
    """Writes load_cell_data messages to their own clean CSV (own native rate, no gaps)."""
    rows = load_cell_rows(load_cell_msgs)
    if not rows:
        return 0

    rows.sort(key=lambda r: r["bag_timestamp_ns"])

    max_fields = max(
        sum(1 for k in r if k.startswith("load_cell_field_")) for r in rows
    )
    fieldnames = ["bag_timestamp_ns"]
    fieldnames += [f"load_cell_field_{i}" for i in range(max_fields)]
    fieldnames += ["load_cell_timestamp", "load_cell_reading", "actuator_displacement"]

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, restval="")
        writer.writeheader()
        writer.writerows(rows)

    return len(rows)


def write_accel_csv(accel_msgs, output_path: Path, column_prefix="accel"):
    """Writes accel messages to their own clean CSV (own native rate, no gaps)."""
    rows = accel_rows(accel_msgs, column_prefix=column_prefix)
    if not rows:
        return 0

    rows.sort(key=lambda r: r["bag_timestamp_ns"])

    fieldnames = ["bag_timestamp_ns", f"{column_prefix}_x", f"{column_prefix}_y", f"{column_prefix}_z"]

    output_path.parent.mkdir(parents=True, exist_ok=True)
    with open(output_path, "w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames, restval="")
        writer.writeheader()
        writer.writerows(rows)

    return len(rows)


def process_bag(bag_dir: Path, output_dir: Path):
    bag_dir = bag_dir.resolve()
    print(f"Processing bag: {bag_dir.name}")

    all_topics = [LOAD_CELL_TOPIC] + ACCEL_TOPICS
    topics_data = read_bag_topics(bag_dir, all_topics)
    load_cell_msgs = topics_data.get(LOAD_CELL_TOPIC, [])

    print(f"  {LOAD_CELL_TOPIC}: {len(load_cell_msgs)} messages")

    any_data_found = bool(load_cell_msgs)

    if load_cell_msgs:
        lc_path = output_dir / f"{bag_dir.name}_load_cell.csv"
        n = write_load_cell_csv(load_cell_msgs, lc_path)
        print(f"  Wrote {n} rows -> {lc_path}")
    else:
        print(f"  No load_cell data -- skipping that CSV for this bag.")

    for accel_topic in ACCEL_TOPICS:
        accel_label = accel_topic.lstrip("/")  # e.g. "accel0"
        accel_msgs = topics_data.get(accel_topic, [])
        print(f"  {accel_topic}: {len(accel_msgs)} messages")

        if accel_msgs:
            any_data_found = True
            ac_path = output_dir / f"{bag_dir.name}_{accel_label}.csv"
            n = write_accel_csv(accel_msgs, ac_path, column_prefix=accel_label)
            print(f"  Wrote {n} rows -> {ac_path}")
        else:
            print(f"  No {accel_topic} data -- skipping that CSV for this bag.")

    if not any_data_found:
        print(f"  No data found for any topic of interest in {bag_dir.name}.")


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "path",
        help="Path to a single bag folder, OR (with --batch) a parent directory containing many bag folders.",
    )
    parser.add_argument(
        "--batch",
        action="store_true",
        help="Treat `path` as a parent directory; process every immediate subfolder that looks like a bag (contains a .db3 or .mcap file).",
    )
    parser.add_argument(
        "-o",
        "--output-dir",
        default="./csv_output",
        help="Directory to write CSVs into (default: ./csv_output)",
    )
    args = parser.parse_args()

    input_path = Path(args.path)
    output_dir = Path(args.output_dir)

    if not input_path.exists():
        print(f"ERROR: path does not exist: {input_path}", file=sys.stderr)
        sys.exit(1)

    if args.batch:
        bag_dirs = [
            d for d in sorted(input_path.iterdir())
            if d.is_dir() and (list(d.glob("*.db3")) or list(d.glob("*.mcap")))
        ]
        if not bag_dirs:
            print(f"ERROR: no bag folders (containing .db3/.mcap) found under {input_path}", file=sys.stderr)
            sys.exit(1)
        print(f"Found {len(bag_dirs)} bag folders under {input_path}\n")
        failed_bags = []
        for bag_dir in bag_dirs:
            try:
                process_bag(bag_dir, output_dir)
            except Exception as e:
                print(f"  ERROR processing {bag_dir.name}: {e}", file=sys.stderr)
                failed_bags.append(bag_dir.name)
            print()
        if failed_bags:
            print(f"WARNING: {len(failed_bags)} of {len(bag_dirs)} bags failed and were skipped:", file=sys.stderr)
            for name in failed_bags:
                print(f"  - {name}", file=sys.stderr)
    else:
        process_bag(input_path, output_dir)

    print("Done.")


if __name__ == "__main__":
    main()