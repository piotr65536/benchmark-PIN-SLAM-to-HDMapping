#!/usr/bin/env python3
# Converts a PIN-SLAM run to an HDMapping session (chunked LAZ + trajectory
# CSVs + poses.reg + session.json), matching the format produced by the C++
# *-to-hdmapping converters of the other benchmarks.
#
# PIN-SLAM runs offline and stores its trajectory as TUM-format pose files
# (timestamp tx ty tz qx qy qz qw, timestamps = sensor time from the bag). The
# world-frame cloud is rebuilt here by transforming every input scan from the
# original bag with its (nearest-in-time) PIN-SLAM pose — this gives a DENSE
# cloud, denser than what most online-recorded benchmarks capture.
#
# Pose file preference: slam_poses_tum.txt (loop-closure/PGO corrected) over
# odom_poses_tum.txt.

import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np

CHUNK_SIZE = 2_000_000
POSE_MATCH_TOLERANCE_NS = int(0.06 * 1e9)  # 60 ms
MIN_RANGE_M = 0.5                          # blind-zone filter, same as configs


def quat_to_rot(qx, qy, qz, qw):
    n = np.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])


def tait_bryan_from_matrix(m):
    # Same convention as pose_tait_bryan_from_affine_matrix in the C++ converters.
    if m[0, 2] < 1:
        if m[0, 2] > -1:
            fi = np.arcsin(m[0, 2])
            om = np.arctan2(-m[1, 2], m[2, 2])
            ka = np.arctan2(-m[0, 1], m[0, 0])
        else:
            fi = -np.pi / 2.0
            om = -np.arctan2(m[1, 0], m[1, 1])
            ka = 0.0
    else:
        fi = np.pi / 2.0
        om = np.arctan2(m[1, 0], m[1, 1])
        ka = 0.0
    return om, fi, ka


def load_tum_poses(run_dir):
    for name in ("slam_poses_tum.txt", "odom_poses_tum.txt"):
        path = os.path.join(run_dir, name)
        if os.path.isfile(path):
            data = np.loadtxt(path, comments="#")
            if data.ndim == 1:
                data = data.reshape(1, -1)
            print(f"Using pose file: {path} ({data.shape[0]} poses)")
            poses = []
            for row in data:
                ts_ns = int(round(row[0] * 1e9))
                t = row[1:4]
                qx, qy, qz, qw = row[4:8]
                T = np.eye(4)
                T[:3, :3] = quat_to_rot(qx, qy, qz, qw)
                T[:3, 3] = t
                poses.append((ts_ns, T, (qw, qx, qy, qz)))
            poses.sort(key=lambda p: p[0])
            return poses
    print(f"ERROR: no slam_poses_tum.txt / odom_poses_tum.txt in {run_dir}")
    sys.exit(1)


def read_bag_clouds(bag_path, topic):
    from rosbags.highlevel import AnyReader

    with AnyReader([Path(bag_path)]) as reader:
        connections = [c for c in reader.connections if c.topic == topic]
        if not connections:
            print(f"ERROR: topic {topic} not found in bag. Available:")
            for c in reader.connections:
                print(f"  {c.topic}  {c.msgtype}")
            sys.exit(1)
        for connection, _, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            yield stamp_ns, msg


def parse_pointcloud2(msg):
    """Return (N,3) float32 xyz and (N,) float32 intensity."""
    offsets = {f.name: (f.offset, f.datatype) for f in msg.fields}
    if not all(k in offsets for k in ("x", "y", "z")):
        return None, None
    step = msg.point_step
    n = msg.width * msg.height
    buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, step)

    def field_f32(name):
        off = offsets[name][0]
        return buf[:, off:off + 4].copy().view(np.float32).reshape(n)

    xyz = np.stack([field_f32("x"), field_f32("y"), field_f32("z")], axis=1)
    intensity = field_f32("intensity") if "intensity" in offsets else np.zeros(n, np.float32)
    return xyz, intensity


def save_laz(path, xyz, intensity, ts_ns):
    import laspy

    header = laspy.LasHeader(version="1.2", point_format=1)
    header.scales = np.array([0.0001, 0.0001, 0.0001])
    header.offsets = np.array([0.0, 0.0, 0.0])
    las = laspy.LasData(header)
    las.x = xyz[:, 0]
    las.y = xyz[:, 1]
    las.z = xyz[:, 2]
    las.intensity = np.clip(intensity, 0, 65535).astype(np.uint16)
    las.gps_time = ts_ns.astype(np.float64)
    las.write(str(path))
    print(f"saved {path} ({xyz.shape[0]} points)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True, help="input bag (ROS1 .bag file or ROS2 dir)")
    ap.add_argument("--run-dir", required=True, help="PIN-SLAM experiment run directory")
    ap.add_argument("--topic", required=True, help="point cloud topic in the bag")
    ap.add_argument("--out", required=True, help="output HDMapping session directory")
    ap.add_argument("--point-skip", type=int, default=1, help="keep every Nth point (default 1 = all)")
    args = ap.parse_args()

    poses = load_tum_poses(args.run_dir)
    pose_ts = np.array([p[0] for p in poses], dtype=np.int64)

    os.makedirs(args.out, exist_ok=True)

    # ── Rebuild world-frame cloud, chunked ───────────────────────────────────
    chunks = []          # list of dicts: xyz, intensity, ts, ts_min, ts_max
    cur_xyz, cur_int, cur_ts = [], [], []
    cur_count = 0
    matched_frames = 0
    total_frames = 0

    def flush_chunk():
        nonlocal cur_xyz, cur_int, cur_ts, cur_count
        if cur_count == 0:
            return
        xyz = np.concatenate(cur_xyz)
        inten = np.concatenate(cur_int)
        ts = np.concatenate(cur_ts)
        chunks.append({"xyz": xyz, "intensity": inten, "ts": ts,
                       "ts_min": int(ts.min()), "ts_max": int(ts.max())})
        cur_xyz, cur_int, cur_ts = [], [], []
        cur_count = 0

    print("Rebuilding world-frame cloud from bag + PIN-SLAM poses...")
    for stamp_ns, msg in read_bag_clouds(args.bag, args.topic):
        total_frames += 1
        idx = np.searchsorted(pose_ts, stamp_ns)
        best, best_dt = None, None
        for j in (idx - 1, idx):
            if 0 <= j < len(pose_ts):
                dt = abs(int(pose_ts[j]) - stamp_ns)
                if best_dt is None or dt < best_dt:
                    best, best_dt = j, dt
        if best is None or best_dt > POSE_MATCH_TOLERANCE_NS:
            continue
        matched_frames += 1

        xyz, intensity = parse_pointcloud2(msg)
        if xyz is None:
            continue
        finite = np.isfinite(xyz).all(axis=1)
        rng2 = (xyz * xyz).sum(axis=1)
        keep = finite & (rng2 > MIN_RANGE_M * MIN_RANGE_M)
        xyz, intensity = xyz[keep], intensity[keep]
        if args.point_skip > 1:
            xyz, intensity = xyz[::args.point_skip], intensity[::args.point_skip]
        if xyz.shape[0] == 0:
            continue

        T = poses[best][1]
        world = xyz.astype(np.float64) @ T[:3, :3].T + T[:3, 3]

        cur_xyz.append(world)
        cur_int.append(intensity)
        cur_ts.append(np.full(world.shape[0], stamp_ns, dtype=np.int64))
        cur_count += world.shape[0]
        if cur_count > CHUNK_SIZE:
            flush_chunk()
    flush_chunk()  # keep ANY non-empty trailing chunk (do not drop small tails)

    print(f"Matched {matched_frames}/{total_frames} frames to poses; {len(chunks)} chunks.")
    if not chunks:
        print("ERROR: no frames matched any pose — check the topic and pose timestamps.")
        sys.exit(1)

    # ── Index trajectory into chunks (first matching chunk, like the C++) ────
    chunks_traj = [[] for _ in chunks]
    for ts_ns, T, quat in poses:
        for j, ch in enumerate(chunks):
            if ch["ts_min"] <= ts_ns <= ch["ts_max"]:
                chunks_traj[j].append((ts_ns, T, quat))
                break
    for j, trj in enumerate(chunks_traj):
        print(f"chunk {j}: {len(trj)} trajectory elements, {chunks[j]['xyz'].shape[0]} points")

    # ── Offset = mean of matched trajectory positions ────────────────────────
    all_pos = [T[:3, 3] for trj in chunks_traj for (_, T, _) in trj]
    offset = np.mean(all_pos, axis=0) if all_pos else np.zeros(3)

    # ── Save LAZ + trajectory CSVs ───────────────────────────────────────────
    m_poses, file_names = [], []
    csv_header = ("timestamp_nanoseconds pose00 pose01 pose02 pose03 "
                  "pose10 pose11 pose12 pose13 pose20 pose21 pose22 pose23 "
                  "timestampUnix_nanoseconds om_rad fi_rad ka_rad")

    for i, (ch, trj) in enumerate(zip(chunks, chunks_traj)):
        if not trj:
            continue
        first_T = trj[0][1]
        first_inv = np.linalg.inv(first_T)

        local = ch["xyz"] @ first_inv[:3, :3].T + first_inv[:3, 3]

        laz_name = f"scan_lio_{i}.laz"
        save_laz(os.path.join(args.out, laz_name), local, ch["intensity"], ch["ts"])
        file_names.append(laz_name)
        m_poses.append(first_T.copy())

        csv_path = os.path.join(args.out, f"trajectory_lio_{i}.csv")
        with open(csv_path, "w") as f:
            f.write(csv_header + "\n")
            for ts_ns, T, _ in trj:
                rel = first_inv @ T
                om, fi, ka = tait_bryan_from_matrix(T)
                vals = " ".join(f"{rel[r, c]:.10g}" for r in range(3) for c in range(4))
                f.write(f"{ts_ns} {vals} {ts_ns} {om:.20g} {fi:.20g} {ka:.20g} \n")
        print(f"saved {csv_path}")

    for T in m_poses:
        T[:3, 3] -= offset

    # ── poses.reg / lio_initial_poses.reg ────────────────────────────────────
    def save_poses_reg(path):
        with open(path, "w") as f:
            f.write(f"{len(m_poses)}\n")
            for name, T in zip(file_names, m_poses):
                f.write(name + "\n")
                for r in range(3):
                    f.write(" ".join(f"{T[r, c]:.6g}" for c in range(4)) + "\n")
                f.write("0 0 0 1\n")

    save_poses_reg(os.path.join(args.out, "lio_initial_poses.reg"))
    save_poses_reg(os.path.join(args.out, "poses.reg"))

    # ── session.json ─────────────────────────────────────────────────────────
    out_abs = os.path.abspath(args.out)
    session = {
        "Session Settings": {
            "folder_name": out_abs,
            "initial_poses_file_name": os.path.join(out_abs, "lio_initial_poses.reg"),
            "lidar_odometry_version": "HdMap",
            "offset_x": 0.0,
            "offset_y": 0.0,
            "offset_z": 0.0,
            "out_folder_name": out_abs,
            "out_poses_file_name": os.path.join(out_abs, "poses.reg"),
            "poses_file_name": os.path.join(out_abs, "poses.reg"),
        },
        "laz_file_names": [{"file_name": os.path.join(out_abs, n)} for n in file_names],
    }
    session_path = os.path.join(out_abs, "session.json")
    with open(session_path, "w") as f:
        json.dump(session, f, indent=2, sort_keys=True)
    print(f"saving file: '{session_path}'")
    print("=== DONE ===")


if __name__ == "__main__":
    main()
