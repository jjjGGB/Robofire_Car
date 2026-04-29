#!/usr/bin/env python3

import argparse
import json
import math
import os
import time
from dataclasses import dataclass
from typing import Dict, List, Optional

import psutil
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2


DEFAULT_PROFILE_MATCHES = {
    'node': ['livox_ros_driver2_node', 'livox_to_pointcloud2_node'],
    'component': ['component_container_mt', 'livox_container'],
}


def percentile(values: List[float], pct: float) -> Optional[float]:
    if not values:
        return None
    ordered = sorted(values)
    index = (len(ordered) - 1) * pct
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return ordered[int(index)]
    lower_value = ordered[lower]
    upper_value = ordered[upper]
    return lower_value + (upper_value - lower_value) * (index - lower)


@dataclass
class RunningStats:
    count: int = 0
    total: float = 0.0
    min_value: Optional[float] = None
    max_value: Optional[float] = None

    def add(self, value: float) -> None:
        self.count += 1
        self.total += value
        if self.min_value is None or value < self.min_value:
            self.min_value = value
        if self.max_value is None or value > self.max_value:
            self.max_value = value

    def summary(self) -> Dict[str, Optional[float]]:
        average = self.total / self.count if self.count else None
        return {
            'count': self.count,
            'avg': average,
            'min': self.min_value,
            'max': self.max_value,
        }


class TopicStatsNode(Node):
    def __init__(self, topic_name: str) -> None:
        super().__init__('livox_perf_monitor')
        self.topic_name = topic_name
        self.message_count = 0
        self.total_payload_bytes = 0
        self.first_receive_monotonic_ns: Optional[int] = None
        self.last_receive_monotonic_ns: Optional[int] = None
        self.arrival_intervals_sec: List[float] = []
        self.latencies_ms: List[float] = []
        self.payload_sizes_bytes: List[int] = []
        self.create_subscription(
            PointCloud2,
            topic_name,
            self._callback,
            qos_profile_sensor_data,
        )

    def _callback(self, msg: PointCloud2) -> None:
        now_mono_ns = time.monotonic_ns()
        now_ros_ns = self.get_clock().now().nanoseconds

        if self.last_receive_monotonic_ns is not None:
            delta_sec = (now_mono_ns - self.last_receive_monotonic_ns) / 1e9
            self.arrival_intervals_sec.append(delta_sec)

        if self.first_receive_monotonic_ns is None:
            self.first_receive_monotonic_ns = now_mono_ns
        self.last_receive_monotonic_ns = now_mono_ns

        stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if stamp_ns > 0:
            latency_ms = (now_ros_ns - stamp_ns) / 1e6
            if latency_ms >= 0.0:
                self.latencies_ms.append(latency_ms)

        payload_size = len(msg.data)
        self.message_count += 1
        self.total_payload_bytes += payload_size
        self.payload_sizes_bytes.append(payload_size)

    def summary(self, wall_duration_sec: float) -> Dict[str, Optional[float]]:
        effective_hz = None
        if self.first_receive_monotonic_ns is not None and self.last_receive_monotonic_ns is not None:
            elapsed_sec = (self.last_receive_monotonic_ns - self.first_receive_monotonic_ns) / 1e9
            if elapsed_sec > 0.0 and self.message_count > 1:
                effective_hz = (self.message_count - 1) / elapsed_sec

        throughput_mbps = None
        if wall_duration_sec > 0.0:
            throughput_mbps = (self.total_payload_bytes * 8.0) / wall_duration_sec / 1e6

        avg_payload_bytes = None
        if self.payload_sizes_bytes:
            avg_payload_bytes = sum(self.payload_sizes_bytes) / len(self.payload_sizes_bytes)

        return {
            'topic': self.topic_name,
            'message_count': self.message_count,
            'effective_hz': effective_hz,
            'throughput_mbps': throughput_mbps,
            'avg_payload_bytes': avg_payload_bytes,
            'latency_ms_avg': sum(self.latencies_ms) / len(self.latencies_ms) if self.latencies_ms else None,
            'latency_ms_p95': percentile(self.latencies_ms, 0.95),
            'latency_ms_max': max(self.latencies_ms) if self.latencies_ms else None,
            'arrival_interval_ms_avg': (
                (sum(self.arrival_intervals_sec) / len(self.arrival_intervals_sec)) * 1000.0
                if self.arrival_intervals_sec else None
            ),
            'arrival_interval_ms_p95': (
                percentile([value * 1000.0 for value in self.arrival_intervals_sec], 0.95)
                if self.arrival_intervals_sec else None
            ),
        }


def read_thread_name(pid: int, tid: int) -> str:
    path = f'/proc/{pid}/task/{tid}/comm'
    try:
        with open(path, 'r', encoding='utf-8') as handle:
            return handle.read().strip()
    except OSError:
        return str(tid)


def process_matches(proc: psutil.Process, patterns: List[str]) -> bool:
    try:
        cmdline = ' '.join(proc.cmdline())
        haystack = f'{proc.name()} {cmdline}'
    except (psutil.NoSuchProcess, psutil.AccessDenied):
        return False
    return any(pattern in haystack for pattern in patterns)


def discover_processes(patterns: List[str]) -> Dict[int, psutil.Process]:
    matches: Dict[int, psutil.Process] = {}
    for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
        if process_matches(proc, patterns):
            matches[proc.pid] = proc
    return matches


def snapshot_processes(procs: Dict[int, psutil.Process]) -> Dict[int, Dict[str, object]]:
    snapshot: Dict[int, Dict[str, object]] = {}
    for pid, proc in list(procs.items()):
        try:
            cpu_times = proc.cpu_times()
            mem_info = proc.memory_info()
            ctx_switches = proc.num_ctx_switches()
            threads = {}
            for thread in proc.threads():
                threads[thread.id] = thread.user_time + thread.system_time
            snapshot[pid] = {
                'name': proc.name(),
                'cmdline': proc.cmdline(),
                'cpu_time_sec': cpu_times.user + cpu_times.system,
                'rss_bytes': mem_info.rss,
                'vms_bytes': mem_info.vms,
                'ctx_voluntary': ctx_switches.voluntary,
                'ctx_involuntary': ctx_switches.involuntary,
                'threads': threads,
            }
        except (psutil.NoSuchProcess, psutil.AccessDenied, ProcessLookupError):
            continue
    return snapshot


def ensure_processes(patterns: List[str], timeout_sec: float) -> Dict[int, psutil.Process]:
    deadline = time.monotonic() + timeout_sec
    while time.monotonic() < deadline:
        matches = discover_processes(patterns)
        if matches:
            return matches
        time.sleep(0.2)
    return discover_processes(patterns)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description='Monitor Livox pipeline performance for node/component comparisons.')
    parser.add_argument('--profile', choices=sorted(DEFAULT_PROFILE_MATCHES.keys()), default='node')
    parser.add_argument('--mode-label', default=None, help='Label written into the JSON summary.')
    parser.add_argument('--topic', default='/livox/lidar/pointcloud')
    parser.add_argument('--duration', type=float, default=10.0, help='Sampling duration in seconds.')
    parser.add_argument('--interval', type=float, default=0.5, help='Sampling interval in seconds.')
    parser.add_argument('--startup-timeout', type=float, default=10.0)
    parser.add_argument('--process-match', action='append', default=[], help='Extra process match substring. Repeatable.')
    parser.add_argument('--output', default=None, help='Optional path to save JSON output.')
    parser.add_argument('--top-threads', type=int, default=5)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    mode_label = args.mode_label or args.profile
    patterns = list(DEFAULT_PROFILE_MATCHES[args.profile])
    patterns.extend(args.process_match)

    rclpy.init()
    node = TopicStatsNode(args.topic)
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    try:
        processes = ensure_processes(patterns, args.startup_timeout)
        if not processes:
            raise RuntimeError(f'No matching processes found for patterns: {patterns}')

        process_cpu_stats: Dict[int, RunningStats] = {}
        process_rss_stats: Dict[int, RunningStats] = {}
        process_ctx_stats: Dict[int, RunningStats] = {}
        thread_cpu_stats: Dict[str, Dict[str, object]] = {}
        total_cpu_stats = RunningStats()
        total_rss_stats = RunningStats()

        start_time = time.monotonic()
        next_sample_time = start_time + args.interval
        prev_snapshot = snapshot_processes(processes)
        prev_sample_time = start_time

        while True:
            now = time.monotonic()
            if now - start_time >= args.duration:
                break

            timeout_sec = max(0.0, min(0.1, next_sample_time - now))
            executor.spin_once(timeout_sec=timeout_sec)

            now = time.monotonic()
            if now < next_sample_time:
                continue

            current_processes = discover_processes(patterns)
            processes.update(current_processes)

            current_snapshot = snapshot_processes(processes)
            elapsed = now - prev_sample_time
            if elapsed <= 0.0:
                prev_snapshot = current_snapshot
                prev_sample_time = now
                next_sample_time += args.interval
                continue

            total_cpu_percent = 0.0
            total_rss_mb = 0.0

            for pid, current in current_snapshot.items():
                previous = prev_snapshot.get(pid)
                if previous is None:
                    continue

                cpu_percent = max(
                    0.0,
                    ((current['cpu_time_sec'] - previous['cpu_time_sec']) / elapsed) * 100.0,
                )
                rss_mb = current['rss_bytes'] / (1024.0 * 1024.0)
                ctx_delta = (
                    (current['ctx_voluntary'] - previous['ctx_voluntary']) +
                    (current['ctx_involuntary'] - previous['ctx_involuntary'])
                )
                ctx_per_sec = max(0.0, ctx_delta / elapsed)

                total_cpu_percent += cpu_percent
                total_rss_mb += rss_mb

                process_cpu_stats.setdefault(pid, RunningStats()).add(cpu_percent)
                process_rss_stats.setdefault(pid, RunningStats()).add(rss_mb)
                process_ctx_stats.setdefault(pid, RunningStats()).add(ctx_per_sec)

                previous_threads = previous['threads']
                current_threads = current['threads']
                for tid, current_thread_cpu in current_threads.items():
                    if tid not in previous_threads:
                        continue
                    thread_cpu_percent = max(
                        0.0,
                        ((current_thread_cpu - previous_threads[tid]) / elapsed) * 100.0,
                    )
                    key = f'{pid}:{tid}'
                    if key not in thread_cpu_stats:
                        thread_cpu_stats[key] = {
                            'pid': pid,
                            'tid': tid,
                            'process_name': current['name'],
                            'thread_name': read_thread_name(pid, tid),
                            'cpu_percent': RunningStats(),
                        }
                    thread_cpu_stats[key]['cpu_percent'].add(thread_cpu_percent)

            total_cpu_stats.add(total_cpu_percent)
            total_rss_stats.add(total_rss_mb)

            prev_snapshot = current_snapshot
            prev_sample_time = now
            next_sample_time += args.interval

        wall_duration_sec = time.monotonic() - start_time
        topic_summary = node.summary(wall_duration_sec)

        process_summaries = []
        final_snapshot = snapshot_processes(processes)
        for pid, snapshot in sorted(final_snapshot.items()):
            process_summaries.append({
                'pid': pid,
                'name': snapshot['name'],
                'cmdline': snapshot['cmdline'],
                'cpu_percent': process_cpu_stats.get(pid, RunningStats()).summary(),
                'rss_mb': process_rss_stats.get(pid, RunningStats()).summary(),
                'ctx_switches_per_sec': process_ctx_stats.get(pid, RunningStats()).summary(),
                'thread_count': len(snapshot['threads']),
            })

        ranked_threads = sorted(
            thread_cpu_stats.values(),
            key=lambda item: item['cpu_percent'].total / max(item['cpu_percent'].count, 1),
            reverse=True,
        )
        top_threads = []
        for item in ranked_threads[:args.top_threads]:
            top_threads.append({
                'pid': item['pid'],
                'tid': item['tid'],
                'process_name': item['process_name'],
                'thread_name': item['thread_name'],
                'cpu_percent': item['cpu_percent'].summary(),
            })

        summary = {
            'mode_label': mode_label,
            'profile': args.profile,
            'patterns': patterns,
            'duration_sec': wall_duration_sec,
            'sample_interval_sec': args.interval,
            'topic': topic_summary,
            'totals': {
                'cpu_percent': total_cpu_stats.summary(),
                'rss_mb': total_rss_stats.summary(),
            },
            'processes': process_summaries,
            'top_threads': top_threads,
        }

        output_path = args.output
        if output_path is None:
            timestamp = time.strftime('%Y%m%d_%H%M%S')
            output_path = os.path.abspath(f'livox_perf_{mode_label}_{timestamp}.json')

        with open(output_path, 'w', encoding='utf-8') as handle:
            json.dump(summary, handle, indent=2, ensure_ascii=False)

        print(f'Wrote summary to {output_path}')
        print(
            json.dumps(
                {
                    'mode_label': mode_label,
                    'topic_effective_hz': topic_summary['effective_hz'],
                    'topic_latency_ms_p95': topic_summary['latency_ms_p95'],
                    'total_cpu_percent_avg': summary['totals']['cpu_percent']['avg'],
                    'total_rss_mb_avg': summary['totals']['rss_mb']['avg'],
                },
                indent=2,
                ensure_ascii=False,
            )
        )
        return 0
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
