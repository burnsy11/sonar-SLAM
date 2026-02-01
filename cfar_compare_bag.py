#!/usr/bin/env python3
"""
Compare CFAR variants on Oculus Ping data from a ROS2 bag.

Example:
  python3 cfar_compare_bag.py --bag /path/to/bag
  python3 cfar_compare_bag.py --bag /path/to/bag --topic /sonar/ping --max-frames 200 --stride 2
  python3 cfar_compare_bag.py --bag /path/to/bag --save-video /tmp/cfar_compare.mp4 --fps 15

Notes:
  - ping.sample_size == 1 -> uint8 intensity samples
  - ping.sample_size == 2 -> uint16 little-endian samples (handled per-row if has_gains)
  - If ping.has_gains: each row begins with 4 bytes of gain data; those bytes are skipped.
  - Threshold scaling uses CA-CFAR analytic alpha as an approximation for GOCA/TM/OS,
    unless --alpha-mode empirical is used to calibrate alpha from data.
"""

import argparse
import sys
import warnings

import numpy as np
import cv2
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

import rclpy
from rclpy.serialization import deserialize_message
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions, StorageFilter
from rosidl_runtime_py.utilities import get_message

try:
    from oculus_interfaces.msg import Ping  # noqa: F401
except Exception:
    Ping = None

try:
    from bruce_slam.utils.topics import SONAR_TOPIC
except Exception:
    SONAR_TOPIC = "/sonar/ping"


def ping_to_polar_image(ping):
    n_ranges = ping.n_ranges
    n_beams = ping.n_beams
    sample_size = ping.sample_size
    step = ping.step
    has_gains = ping.has_gains

    ping_data = np.frombuffer(ping.ping_data, dtype=np.uint8)

    if sample_size not in (1, 2):
        raise ValueError(f"Unsupported sample_size={sample_size}, expected 1 or 2")

    if has_gains:
        rows = []
        row_bytes = n_beams * sample_size
        for i in range(n_ranges):
            row_start = i * step
            row_data = ping_data[row_start + 4 : row_start + 4 + row_bytes]
            if sample_size == 1:
                row = row_data
            else:
                row = row_data.view("<u2")
            rows.append(row)
        img = np.stack(rows, axis=0)
    else:
        if sample_size == 1:
            img = ping_data.reshape(n_ranges, n_beams)
        else:
            img = ping_data.view("<u2").reshape(n_ranges, n_beams)

    return img


class PolarToCartesianMapper:
    def __init__(self):
        self.res = None
        self.height = None
        self.rows = None
        self.width = None
        self.cols = None
        self.map_x = None
        self.map_y = None
        self.REVERSE_Z = 1

    def update(self, ping):
        _res = ping.range_resolution
        _height = ping.n_ranges * _res
        _rows = ping.n_ranges
        bearings_deg = np.array(ping.bearings) * 0.01
        bearings_rad = bearings_deg * np.pi / 180.0
        _width = np.sin((bearings_rad[-1] - bearings_rad[0]) / 2.0) * _height * 2
        _cols = int(np.ceil(_width / _res))

        if (
            self.res == _res
            and self.height == _height
            and self.rows == _rows
            and self.width == _width
            and self.cols == _cols
        ):
            return

        self.res, self.height, self.rows, self.width, self.cols = (
            _res,
            _height,
            _rows,
            _width,
            _cols,
        )

        bearings_deg = np.asarray(ping.bearings, dtype=np.float32) * 0.01
        bearings = bearings_deg * np.pi / 180.0
        f_bearings = interp1d(
            bearings,
            range(len(bearings)),
            kind="linear",
            bounds_error=False,
            fill_value=-1,
            assume_sorted=True,
        )

        XX, YY = np.meshgrid(range(self.cols), range(self.rows))
        x = self.res * (self.rows - YY)
        y = self.res * (-self.cols / 2.0 + XX + 0.5)
        b = np.arctan2(y, x) * self.REVERSE_Z
        r = np.sqrt(np.square(x) + np.square(y))
        self.map_y = np.asarray(r / self.res, dtype=np.float32)
        self.map_x = np.asarray(f_bearings(b), dtype=np.float32)


def calc_alpha_ca(pfa, n_ref):
    return n_ref * (pfa ** (-1.0 / n_ref) - 1.0)


def _z_hat_goca(leading, trailing):
    mean_lead = float(np.mean(leading)) if leading.size else 0.0
    mean_trail = float(np.mean(trailing)) if trailing.size else 0.0
    return max(mean_lead, mean_trail)


def _z_hat_tm(refs_sorted, trim_frac):
    n_ref = refs_sorted.size
    trim = int(np.ceil(trim_frac * n_ref))
    trim = min(trim, n_ref - 1)
    kept = refs_sorted[: n_ref - trim]
    return float(np.mean(kept)) if kept.size else 0.0


def _z_hat_os(refs_sorted, os_rank=None, os_rank_frac=0.75):
    n_ref = refs_sorted.size
    if os_rank is None:
        k = int(round(os_rank_frac * n_ref))
    else:
        k = int(os_rank)
    k = max(1, min(n_ref, k))
    return float(refs_sorted[k - 1])


def detect_cfar(
    polar_img,
    ntc,
    ngc,
    method,
    alpha,
    os_rank=None,
    os_rank_frac=0.75,
    trim_frac=0.2,
    intensity_threshold=0,
):
    n_ranges, n_beams = polar_img.shape
    mask = np.zeros_like(polar_img, dtype=bool)
    start = ntc + ngc
    end = n_ranges - (ntc + ngc)
    n_ref = 2 * ntc

    for b in range(n_beams):
        col = polar_img[:, b].astype(np.float32)
        for r in range(start, end):
            leading = col[r - ngc - ntc : r - ngc]
            trailing = col[r + ngc + 1 : r + ngc + 1 + ntc]
            if leading.size != ntc or trailing.size != ntc:
                continue

            if method == "goca":
                z_hat = _z_hat_goca(leading, trailing)
            else:
                refs = np.concatenate([leading, trailing])
                refs_sorted = np.sort(refs)
                if method == "tm":
                    z_hat = _z_hat_tm(refs_sorted, trim_frac)
                elif method == "os":
                    z_hat = _z_hat_os(refs_sorted, os_rank=os_rank, os_rank_frac=os_rank_frac)
                else:
                    raise ValueError(f"Unknown CFAR method: {method}")

            if z_hat <= 0:
                continue
            if col[r] > alpha * z_hat:
                mask[r, b] = True

    if intensity_threshold > 0:
        mask &= polar_img > intensity_threshold

    return mask


def calibrate_alpha(
    frames,
    ntc,
    ngc,
    method,
    pfa,
    os_rank=None,
    os_rank_frac=0.75,
    trim_frac=0.2,
    max_ratios=200000,
):
    ratios = []
    n_ref = 2 * ntc
    start = ntc + ngc

    for frame in frames:
        polar = frame["polar"]
        n_ranges, n_beams = polar.shape
        end = n_ranges - (ntc + ngc)
        for b in range(n_beams):
            col = polar[:, b].astype(np.float32)
            for r in range(start, end):
                leading = col[r - ngc - ntc : r - ngc]
                trailing = col[r + ngc + 1 : r + ngc + 1 + ntc]
                if leading.size != ntc or trailing.size != ntc:
                    continue
                if method == "goca":
                    z_hat = _z_hat_goca(leading, trailing)
                else:
                    refs = np.concatenate([leading, trailing])
                    refs_sorted = np.sort(refs)
                    if method == "tm":
                        z_hat = _z_hat_tm(refs_sorted, trim_frac)
                    elif method == "os":
                        z_hat = _z_hat_os(
                            refs_sorted, os_rank=os_rank, os_rank_frac=os_rank_frac
                        )
                    else:
                        raise ValueError(f"Unknown CFAR method: {method}")
                if z_hat > 0:
                    ratios.append(col[r] / z_hat)
                if len(ratios) >= max_ratios:
                    ratios = ratios[::2]
        if len(ratios) >= max_ratios:
            ratios = ratios[::2]

    if not ratios:
        return calc_alpha_ca(pfa, n_ref)

    ratios = np.asarray(ratios, dtype=np.float32)
    return float(np.quantile(ratios, 1.0 - pfa))


def normalize_for_display(img):
    if img.dtype == np.uint8:
        return img
    max_val = np.iinfo(img.dtype).max
    scaled = (img.astype(np.float32) / float(max_val)) * 255.0
    return np.clip(scaled, 0.0, 255.0).astype(np.uint8)


def apply_overlay(base_rgb, mask, alpha):
    if alpha <= 0:
        return base_rgb
    out = base_rgb.astype(np.float32)
    red = np.array([255.0, 0.0, 0.0], dtype=np.float32)
    idx = mask.astype(bool)
    out[idx] = out[idx] * (1.0 - alpha) + red * alpha
    return out.astype(np.uint8)


def read_bag_frames(bag_path, topic, max_frames, stride, start_index, storage_id="sqlite3"):
    reader = SequentialReader()
    storage_options = StorageOptions(uri=bag_path, storage_id=storage_id)
    converter_options = ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader.open(storage_options, converter_options)

    topics = reader.get_all_topics_and_types()
    type_map = {t.name: t.type for t in topics}
    if topic not in type_map:
        available = "\n".join(sorted(type_map.keys()))
        raise RuntimeError(f"Topic '{topic}' not found. Available topics:\n{available}")

    msg_type = get_message(type_map[topic])
    reader.set_filter(StorageFilter(topics=[topic]))

    frames = []
    msg_idx = 0
    kept = 0

    while reader.has_next():
        topic_name, data, _t = reader.read_next()
        if topic_name != topic:
            continue
        if msg_idx < start_index:
            msg_idx += 1
            continue
        if (msg_idx - start_index) % stride != 0:
            msg_idx += 1
            continue

        ping = deserialize_message(data, msg_type)
        polar = ping_to_polar_image(ping)
        frames.append({"ping": ping, "polar": polar, "index": msg_idx})
        kept += 1
        msg_idx += 1
        if max_frames is not None and kept >= max_frames:
            break

    return frames


class CFARViewer:
    def __init__(
        self,
        frames,
        mapper,
        ntc,
        ngc,
        pfa,
        intensity_threshold,
        os_rank,
        os_rank_frac,
        trim_frac,
        alpha_os,
        alpha_tm,
        alpha_goca,
        coordinates="cartesian",
        overlay_alpha=0.6,
    ):
        self.frames = frames
        self.mapper = mapper
        self.ntc = ntc
        self.ngc = ngc
        self.pfa = pfa
        self.intensity_threshold = intensity_threshold
        self.os_rank = os_rank
        self.os_rank_frac = os_rank_frac
        self.trim_frac = trim_frac
        self.alpha_os = alpha_os
        self.alpha_tm = alpha_tm
        self.alpha_goca = alpha_goca
        self.coordinates = coordinates
        self.overlay_alpha = overlay_alpha

        self.idx = 0
        self.cache = {}
        self.current_shape = None

        self.fig, self.axes = plt.subplots(2, 2, figsize=(12, 9))
        self.fig.canvas.mpl_connect("key_press_event", self.on_key)
        self.img_artists = None
        self.fig.suptitle("CFAR comparison (n/p for next/prev, q to quit, s to save PNG)")
        self.show_frame(0)

    def _compute_masks(self, polar):
        return {
            "os": detect_cfar(
                polar,
                self.ntc,
                self.ngc,
                "os",
                self.alpha_os,
                os_rank=self.os_rank,
                os_rank_frac=self.os_rank_frac,
                intensity_threshold=self.intensity_threshold,
            ),
            "tm": detect_cfar(
                polar,
                self.ntc,
                self.ngc,
                "tm",
                self.alpha_tm,
                trim_frac=self.trim_frac,
                intensity_threshold=self.intensity_threshold,
            ),
            "goca": detect_cfar(
                polar,
                self.ntc,
                self.ngc,
                "goca",
                self.alpha_goca,
                intensity_threshold=self.intensity_threshold,
            ),
        }

    def _get_masks(self, frame_idx):
        if frame_idx not in self.cache:
            polar = self.frames[frame_idx]["polar"]
            self.cache[frame_idx] = self._compute_masks(polar)
        return self.cache[frame_idx]

    def _compose_views(self, frame, masks):
        polar = frame["polar"]
        if self.coordinates == "cartesian":
            self.mapper.update(frame["ping"])
            base = cv2.remap(polar, self.mapper.map_x, self.mapper.map_y, cv2.INTER_LINEAR)
            mask_os = (
                cv2.remap(
                    masks["os"].astype(np.uint8),
                    self.mapper.map_x,
                    self.mapper.map_y,
                    cv2.INTER_NEAREST,
                )
                != 0
            )
            mask_tm = (
                cv2.remap(
                    masks["tm"].astype(np.uint8),
                    self.mapper.map_x,
                    self.mapper.map_y,
                    cv2.INTER_NEAREST,
                )
                != 0
            )
            mask_goca = (
                cv2.remap(
                    masks["goca"].astype(np.uint8),
                    self.mapper.map_x,
                    self.mapper.map_y,
                    cv2.INTER_NEAREST,
                )
                != 0
            )
        else:
            base = polar
            mask_os = masks["os"]
            mask_tm = masks["tm"]
            mask_goca = masks["goca"]

        base_disp = normalize_for_display(base)
        base_rgb = np.dstack([base_disp] * 3)

        os_overlay = apply_overlay(base_rgb, mask_os, self.overlay_alpha)
        tm_overlay = apply_overlay(base_rgb, mask_tm, self.overlay_alpha)
        goca_overlay = apply_overlay(base_rgb, mask_goca, self.overlay_alpha)

        return base_rgb, os_overlay, tm_overlay, goca_overlay

    def _format_titles(self, frame):
        ping = frame["ping"]
        stamp = ping.header.stamp
        ts = f"{stamp.sec}.{stamp.nanosec:09d}"
        base = f"Frame {frame['index']} ping_id={ping.ping_id} t={ts}"
        os_rank = self.os_rank if self.os_rank is not None else int(
            round(self.os_rank_frac * (2 * self.ntc))
        )
        titles = [
            f"{base} Original ({self.coordinates})",
            f"{base} OS-CFAR (ntc={self.ntc}, ngc={self.ngc}, pfa={self.pfa}, k={os_rank})",
            f"{base} TM-CFAR (ntc={self.ntc}, ngc={self.ngc}, pfa={self.pfa}, trim={self.trim_frac})",
            f"{base} GOCA-CFAR (ntc={self.ntc}, ngc={self.ngc}, pfa={self.pfa})",
        ]
        return titles

    def show_frame(self, idx):
        idx = max(0, min(len(self.frames) - 1, idx))
        self.idx = idx
        frame = self.frames[idx]
        masks = self._get_masks(idx)
        views = self._compose_views(frame, masks)

        if self.img_artists is None or views[0].shape != self.current_shape:
            self.current_shape = views[0].shape
            for ax in self.axes.flat:
                ax.clear()
                ax.set_xticks([])
                ax.set_yticks([])
            self.img_artists = []
            for ax, img in zip(self.axes.flat, views):
                self.img_artists.append(ax.imshow(img))
        else:
            for artist, img in zip(self.img_artists, views):
                artist.set_data(img)

        for ax, title in zip(self.axes.flat, self._format_titles(frame)):
            ax.set_title(title, fontsize=9)

        self.fig.canvas.draw_idle()

    def on_key(self, event):
        if event.key == "right":
            print("Right key pressed")
            self.show_frame(self.idx + 1)
        elif event.key == "left":
            print("Left key pressed")
            self.show_frame(self.idx - 1)
        elif event.key in ("q", "escape"):
            plt.close(self.fig)
        elif event.key == "s":
            fname = f"cfar_frame_{self.idx:06d}.png"
            self.fig.savefig(fname, dpi=150)
            print(f"Saved {fname}")


def save_video(
    viewer,
    out_path,
    fps,
):
    viewer.show_frame(0)
    viewer.fig.canvas.draw()
    width, height = viewer.fig.canvas.get_width_height()
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    writer = cv2.VideoWriter(out_path, fourcc, fps, (width, height))
    if not writer.isOpened():
        raise RuntimeError(f"Failed to open video writer for {out_path}")

    for idx in range(len(viewer.frames)):
        viewer.show_frame(idx)
        viewer.fig.canvas.draw()
        rgb = np.frombuffer(viewer.fig.canvas.tostring_rgb(), dtype=np.uint8)
        rgb = rgb.reshape(height, width, 3)
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        writer.write(bgr)

    writer.release()


def parse_args(argv):
    parser = argparse.ArgumentParser(description="Compare CFAR variants on a ROS2 sonar bag.")
    parser.add_argument("--bag", required=True, help="Path to rosbag2 directory")
    parser.add_argument("--topic", default=SONAR_TOPIC, help="Oculus Ping topic name")
    parser.add_argument("--max-frames", type=int, default=500, help="Max frames to load (0=all)")
    parser.add_argument("--stride", type=int, default=1, help="Stride between frames")
    parser.add_argument("--start-index", type=int, default=0, help="Start index for frames")
    parser.add_argument("--storage-id", default="sqlite3", help="rosbag2 storage id (default sqlite3)")

    parser.add_argument("--ntc", type=int, default=40, help="Training cells per side")
    parser.add_argument("--ngc", type=int, default=10, help="Guard cells per side")
    parser.add_argument("--pfa", type=float, default=1e-2, help="Probability of false alarm")
    parser.add_argument(
        "--intensity-threshold", type=float, default=65, help="Post-gating intensity threshold"
    )
    parser.add_argument("--os-rank-frac", type=float, default=0.75, help="OS rank fraction")
    parser.add_argument("--os-rank", type=int, default=None, help="Explicit OS rank (1..N)")
    parser.add_argument("--trim-frac", type=float, default=0.2, help="Trim fraction for TM-CFAR")

    parser.add_argument(
        "--alpha-mode",
        choices=["analytic", "empirical"],
        default="analytic",
        help="Alpha scaling mode",
    )
    parser.add_argument(
        "--calib-frames", type=int, default=50, help="Frames to use for empirical calibration"
    )
    parser.add_argument(
        "--calib-max-ratios",
        type=int,
        default=200000,
        help="Max CUT/z_hat ratios to keep for calibration",
    )

    parser.add_argument(
        "--coordinates",
        choices=["cartesian", "polar"],
        default="cartesian",
        help="Display coordinates",
    )
    parser.add_argument("--overlay-alpha", type=float, default=0.6, help="Overlay alpha")
    parser.add_argument(
        "--alpha-scale-os",
        type=float,
        default=0.7,
        help="Scale factor for OS-CFAR alpha (lower = less strict)",
    )
    parser.add_argument(
        "--alpha-scale-goca",
        type=float,
        default=0.7,
        help="Scale factor for GOCA-CFAR alpha (lower = less strict)",
    )
    parser.add_argument("--save-video", default=None, help="Optional MP4 output path")
    parser.add_argument("--fps", type=int, default=15, help="FPS for saved video")

    return parser.parse_args(argv)


def main(argv):
    args = parse_args(argv)

    if args.max_frames is not None and args.max_frames <= 0:
        warnings.warn("Loading all frames; this may use significant memory.")
        max_frames = None
    else:
        max_frames = args.max_frames

    rclpy.init(args=None)
    try:
        frames = read_bag_frames(
            args.bag,
            args.topic,
            max_frames,
            args.stride,
            args.start_index,
            storage_id=args.storage_id,
        )
    finally:
        rclpy.shutdown()

    if not frames:
        print("No frames loaded. Check topic, start-index, stride, and bag contents.")
        return 1

    if max_frames is not None and len(frames) >= max_frames:
        print(f"Loaded {len(frames)} frames (max-frames reached).")
    else:
        print(f"Loaded {len(frames)} frames.")

    n_ref = 2 * args.ntc
    if args.alpha_mode == "analytic":
        alpha_os = calc_alpha_ca(args.pfa, n_ref)
        alpha_tm = calc_alpha_ca(args.pfa, n_ref)
        alpha_goca = calc_alpha_ca(args.pfa, n_ref)
    else:
        calib_frames = frames[: min(args.calib_frames, len(frames))]
        alpha_os = calibrate_alpha(
            calib_frames,
            args.ntc,
            args.ngc,
            "os",
            args.pfa,
            os_rank=args.os_rank,
            os_rank_frac=args.os_rank_frac,
            trim_frac=args.trim_frac,
            max_ratios=args.calib_max_ratios,
        )
        alpha_tm = calibrate_alpha(
            calib_frames,
            args.ntc,
            args.ngc,
            "tm",
            args.pfa,
            os_rank=args.os_rank,
            os_rank_frac=args.os_rank_frac,
            trim_frac=args.trim_frac,
            max_ratios=args.calib_max_ratios,
        )
        alpha_goca = calibrate_alpha(
            calib_frames,
            args.ntc,
            args.ngc,
            "goca",
            args.pfa,
            os_rank=args.os_rank,
            os_rank_frac=args.os_rank_frac,
            trim_frac=args.trim_frac,
            max_ratios=args.calib_max_ratios,
        )

    alpha_os *= args.alpha_scale_os
    alpha_goca *= args.alpha_scale_goca

    mapper = PolarToCartesianMapper()
    viewer = CFARViewer(
        frames,
        mapper,
        args.ntc,
        args.ngc,
        args.pfa,
        args.intensity_threshold,
        args.os_rank,
        args.os_rank_frac,
        args.trim_frac,
        alpha_os,
        alpha_tm,
        alpha_goca,
        coordinates=args.coordinates,
        overlay_alpha=args.overlay_alpha,
    )

    if args.save_video:
        print(f"Saving video to {args.save_video} ...")
        save_video(viewer, args.save_video, args.fps)
        print("Video saved.")

    plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
