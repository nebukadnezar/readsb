#!/usr/bin/env python3
"""
Beast dump file playback tool for readsb.
(c) 2025 Balthasar Indermuehle <balt@inside.net>

// This file is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// any later version.
//
// This file is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

Reads zstd-compressed beast dump files and plays them back in real-time
using embedded synthetic timestamps to pace the data.

Usage:
    python beast_playback.py [OPTIONS] <pattern>

Example:
    python beast_playback.py --host localhost --port 30015 "*.zst"
    python beast_playback.py --speed 2.0 "/path/to/dumps/*.zst"
"""

import argparse
import glob
import socket
import struct
import sys
import time
from pathlib import Path
from typing import Iterator, Tuple, List, Optional

try:
    import zstandard as zstd
except ImportError:
    print("Error: zstandard library required. Install with: pip install zstandard")
    sys.exit(1)


# Beast protocol constants
BEAST_ESCAPE = 0x1A
MSG_TYPE_MODEAC = ord('1')      # 2 bytes message
MSG_TYPE_SHORT = ord('2')       # 7 bytes message
MSG_TYPE_LONG = ord('3')        # 14 bytes message
MSG_TYPE_RECEIVER_ID = 0xE3     # 8 bytes receiver ID
MSG_TYPE_SYNTHETIC_TS = 0xE8    # 8 bytes synthetic timestamp (ms epoch)

# Message lengths (excluding escape sequences)
MSG_LENGTHS = {
    MSG_TYPE_MODEAC: 2,
    MSG_TYPE_SHORT: 7,
    MSG_TYPE_LONG: 14,
}


class BeastParser:
    """Parser for Beast binary format with escape handling."""

    def __init__(self, data: bytes):
        self.data = data
        self.pos = 0
        self.current_timestamp_ms: Optional[int] = None

    def read_byte_escaped(self) -> Optional[int]:
        """Read a byte, handling 0x1A escape sequences. Returns None at EOF."""
        if self.pos >= len(self.data):
            return None
        b = self.data[self.pos]
        self.pos += 1
        if b == BEAST_ESCAPE and self.pos < len(self.data):
            next_b = self.data[self.pos]
            if next_b == BEAST_ESCAPE:
                # Escaped 0x1A, consume the second one
                self.pos += 1
        return b

    def read_bytes_escaped(self, count: int) -> Optional[bytes]:
        """Read count bytes with escape handling. Returns None if not enough data."""
        result = []
        for _ in range(count):
            b = self.read_byte_escaped()
            if b is None:
                return None
            result.append(b)
        return bytes(result)

    def parse_messages(self) -> Iterator[Tuple[Optional[int], bytes]]:
        """
        Parse beast messages from the data.

        Yields tuples of (timestamp_ms, raw_message_bytes).
        timestamp_ms is the synthetic timestamp if one was seen before this message.
        raw_message_bytes includes the 0x1A prefix and all escaped bytes.
        """
        while self.pos < len(self.data):
            # Find next 0x1A
            start_pos = self.pos
            while self.pos < len(self.data) and self.data[self.pos] != BEAST_ESCAPE:
                self.pos += 1

            if self.pos >= len(self.data):
                break

            msg_start = self.pos
            self.pos += 1  # Skip the 0x1A

            if self.pos >= len(self.data):
                break

            msg_type = self.data[self.pos]
            self.pos += 1

            if msg_type == MSG_TYPE_SYNTHETIC_TS:
                # Synthetic timestamp: 8 bytes int64_t (native byte order, no escaping)
                # Written raw by readsb without escape handling
                if self.pos + 8 > len(self.data):
                    break
                ts_bytes = self.data[self.pos:self.pos + 8]
                self.pos += 8
                # Use native byte order (little-endian on x86)
                self.current_timestamp_ms = struct.unpack('=q', ts_bytes)[0]
                continue

            elif msg_type == MSG_TYPE_RECEIVER_ID:
                # Receiver ID: 8 bytes (with escape handling)
                receiver_data = self.read_bytes_escaped(8)
                if receiver_data is None:
                    break
                # Yield the receiver ID message as-is
                msg_end = self.pos
                raw_msg = self.data[msg_start:msg_end]
                yield (self.current_timestamp_ms, raw_msg)

            elif msg_type in MSG_LENGTHS:
                # Regular beast message: 6 byte timestamp + 1 byte signal + N bytes data
                msg_len = MSG_LENGTHS[msg_type]
                total_payload = 6 + 1 + msg_len  # timestamp + signal + message

                payload = self.read_bytes_escaped(total_payload)
                if payload is None:
                    break

                # Calculate raw message bounds for sending
                msg_end = self.pos
                raw_msg = self.data[msg_start:msg_end]
                yield (self.current_timestamp_ms, raw_msg)

            else:
                # Unknown message type, skip
                continue


def read_dump_file(filepath: Path) -> bytes:
    """Read and decompress a zstd-compressed dump file."""
    dctx = zstd.ZstdDecompressor()
    with open(filepath, 'rb') as f:
        return dctx.decompress(f.read(), max_output_size=500 * 1024 * 1024)


def get_sorted_files(patterns: List[str]) -> List[Path]:
    """Get sorted list of files matching the given glob patterns."""
    files = []
    for pattern in patterns:
        files.extend(Path(p) for p in glob.glob(pattern))

    # Sort by filename (assumes HHMMSS format like 092422Z.zst)
    files = sorted(set(files), key=lambda p: p.name)
    return files


def collect_messages(files: List[Path], verbose: bool = False) -> List[Tuple[int, bytes]]:
    """
    Collect all messages from multiple dump files with their timestamps.
    Returns list of (timestamp_ms, raw_bytes) sorted by timestamp.
    """
    messages = []

    for filepath in files:
        if verbose:
            print(f"Reading {filepath}...", file=sys.stderr)

        try:
            data = read_dump_file(filepath)
        except Exception as e:
            print(f"Error reading {filepath}: {e}", file=sys.stderr)
            continue

        parser = BeastParser(data)
        file_messages = 0

        for ts_ms, raw_msg in parser.parse_messages():
            if ts_ms is not None:
                messages.append((ts_ms, raw_msg))
                file_messages += 1

        if verbose:
            print(f"  Found {file_messages} messages", file=sys.stderr)

    # Sort by timestamp
    messages.sort(key=lambda x: x[0])
    return messages


def stream_messages(files: List[Path], verbose: bool = False) -> Iterator[Tuple[int, bytes]]:
    """
    Stream messages from multiple dump files without loading all into memory.
    Files should be in chronological order by filename.
    Yields (timestamp_ms, raw_bytes) tuples.
    """
    for filepath in files:
        if verbose:
            print(f"Streaming {filepath}...", file=sys.stderr)

        try:
            data = read_dump_file(filepath)
        except Exception as e:
            print(f"Error reading {filepath}: {e}", file=sys.stderr)
            continue

        parser = BeastParser(data)

        for ts_ms, raw_msg in parser.parse_messages():
            if ts_ms is not None:
                yield (ts_ms, raw_msg)


def create_synthetic_timestamp_msg(ts_ms: int) -> bytes:
    """Create a synthetic timestamp message (native byte order to match readsb)."""
    return bytes([BEAST_ESCAPE, MSG_TYPE_SYNTHETIC_TS]) + struct.pack('=q', ts_ms)


def playback_realtime(
    messages: List[Tuple[int, bytes]],
    sock: socket.socket,
    speed: float = 1.0,
    verbose: bool = False,
    skip_seconds: float = 0
) -> None:
    """
    Play back messages in real-time, using timestamps to pace delivery.

    Args:
        messages: List of (timestamp_ms, raw_bytes) tuples, sorted by timestamp
        sock: Connected socket to send data to
        speed: Playback speed multiplier (2.0 = 2x speed)
        verbose: Print progress information
        skip_seconds: Skip this many seconds from the start
    """
    if not messages:
        print("No messages to play back", file=sys.stderr)
        return

    first_ts = messages[0][0]
    last_ts = messages[-1][0]
    duration_sec = (last_ts - first_ts) / 1000.0

    # Calculate skip offset
    skip_ms = int(skip_seconds * 1000)
    start_ts = first_ts + skip_ms

    if verbose:
        print(f"Playback starting:", file=sys.stderr)
        print(f"  Messages: {len(messages)}", file=sys.stderr)
        print(f"  Duration: {duration_sec:.1f} seconds", file=sys.stderr)
        print(f"  Speed: {speed}x", file=sys.stderr)
        if skip_seconds > 0:
            print(f"  Skipping: {skip_seconds:.1f} seconds", file=sys.stderr)
        print(f"  First timestamp: {first_ts} ms", file=sys.stderr)
        print(f"  Last timestamp: {last_ts} ms", file=sys.stderr)

    playback_start = time.time()
    data_start_ts = start_ts

    last_sent_ts = None
    sent_count = 0
    bytes_sent = 0
    skipped_count = 0

    try:
        for msg_ts, raw_msg in messages:
            # Skip messages before start time
            if msg_ts < start_ts:
                skipped_count += 1
                continue

            # Calculate when this message should be sent
            msg_offset_ms = msg_ts - data_start_ts
            target_time = playback_start + (msg_offset_ms / 1000.0 / speed)

            # Wait until it's time to send
            now = time.time()
            if target_time > now:
                time.sleep(target_time - now)

            # Send synthetic timestamp if it changed
            if last_sent_ts != msg_ts:
                ts_msg = create_synthetic_timestamp_msg(msg_ts)
                sock.sendall(ts_msg)
                bytes_sent += len(ts_msg)
                last_sent_ts = msg_ts

            # Send the message
            sock.sendall(raw_msg)
            bytes_sent += len(raw_msg)
            sent_count += 1

            # Progress update every 10 seconds of data
            if verbose and sent_count % 10000 == 0:
                elapsed = time.time() - playback_start
                data_elapsed = (msg_ts - data_start_ts) / 1000.0
                print(f"  Progress: {data_elapsed:.1f}s data, {elapsed:.1f}s elapsed, "
                      f"{sent_count} msgs, {bytes_sent/1024:.1f} KB sent", file=sys.stderr)

    except KeyboardInterrupt:
        print("\nPlayback interrupted", file=sys.stderr)
    except BrokenPipeError:
        print("\nConnection closed by remote", file=sys.stderr)

    if verbose:
        elapsed = time.time() - playback_start
        if skipped_count > 0:
            print(f"Skipped {skipped_count} messages before start offset", file=sys.stderr)
        print(f"Playback complete: {sent_count} messages, {bytes_sent/1024:.1f} KB "
              f"in {elapsed:.1f} seconds", file=sys.stderr)


def playback_streaming(
    files: List[Path],
    sock: socket.socket,
    speed: float = 1.0,
    verbose: bool = False,
    skip_seconds: float = 0
) -> None:
    """
    Play back messages in streaming mode without loading all into memory.

    Args:
        files: List of dump files in chronological order
        sock: Connected socket to send data to
        speed: Playback speed multiplier
        verbose: Print progress information
        skip_seconds: Skip this many seconds from the start
    """
    skip_ms = int(skip_seconds * 1000)

    playback_start = None
    data_start_ts = None
    last_sent_ts = None
    sent_count = 0
    bytes_sent = 0
    skipped_count = 0
    first_ts = None

    if verbose:
        print(f"Streaming playback starting...", file=sys.stderr)
        print(f"  Speed: {speed}x", file=sys.stderr)
        if skip_seconds > 0:
            print(f"  Skipping: {skip_seconds:.1f} seconds", file=sys.stderr)

    try:
        for msg_ts, raw_msg in stream_messages(files, verbose=verbose):
            if first_ts is None:
                first_ts = msg_ts
                data_start_ts = first_ts + skip_ms
                playback_start = time.time()
                if verbose:
                    print(f"  First timestamp: {first_ts} ms", file=sys.stderr)

            # Skip messages before start time
            if msg_ts < data_start_ts:
                skipped_count += 1
                continue

            # Calculate when this message should be sent
            msg_offset_ms = msg_ts - data_start_ts
            target_time = playback_start + (msg_offset_ms / 1000.0 / speed)

            # Wait until it's time to send
            now = time.time()
            if target_time > now:
                time.sleep(target_time - now)

            # Send synthetic timestamp if it changed
            if last_sent_ts != msg_ts:
                ts_msg = create_synthetic_timestamp_msg(msg_ts)
                sock.sendall(ts_msg)
                bytes_sent += len(ts_msg)
                last_sent_ts = msg_ts

            # Send the message
            sock.sendall(raw_msg)
            bytes_sent += len(raw_msg)
            sent_count += 1

            # Progress update
            if verbose and sent_count % 10000 == 0:
                elapsed = time.time() - playback_start
                data_elapsed = (msg_ts - data_start_ts) / 1000.0
                print(f"  Progress: {data_elapsed:.1f}s data, {elapsed:.1f}s elapsed, "
                      f"{sent_count} msgs, {bytes_sent/1024:.1f} KB sent", file=sys.stderr)

    except KeyboardInterrupt:
        print("\nPlayback interrupted", file=sys.stderr)
    except BrokenPipeError:
        print("\nConnection closed by remote", file=sys.stderr)

    if verbose and playback_start:
        elapsed = time.time() - playback_start
        if skipped_count > 0:
            print(f"Skipped {skipped_count} messages before start offset", file=sys.stderr)
        print(f"Playback complete: {sent_count} messages, {bytes_sent/1024:.1f} KB "
              f"in {elapsed:.1f} seconds", file=sys.stderr)


def main():
    parser = argparse.ArgumentParser(
        description='Play back readsb beast dump files in real-time',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Examples:
  %(prog)s "*.zst"
  %(prog)s --host localhost --port 30015 "/data/dumps/092*.zst"
  %(prog)s --speed 2.0 --verbose "dump_*.zst"
  %(prog)s --stream --skip 60 "*.zst"  # Skip first 60 seconds, stream mode

The script reads zstd-compressed beast dump files and sends them to readsb
at the original recording rate, using embedded synthetic timestamps.

Run readsb with:
  readsb --devel=accept_synthetic --net-only --net-bi-port 30015
'''
    )

    parser.add_argument('files', nargs='+', metavar='PATTERN',
                        help='Glob pattern(s) for dump files (e.g., "*.zst")')
    parser.add_argument('--host', '-H', default='localhost',
                        help='Target host (default: localhost)')
    parser.add_argument('--port', '-p', type=int, default=30015,
                        help='Target port (default: 30015)')
    parser.add_argument('--speed', '-s', type=float, default=1.0,
                        help='Playback speed multiplier (default: 1.0)')
    parser.add_argument('--skip', '-k', type=float, default=0,
                        help='Skip first N seconds of data (default: 0)')
    parser.add_argument('--stream', action='store_true',
                        help='Stream mode: process files one at a time (saves memory)')
    parser.add_argument('--verbose', '-v', action='store_true',
                        help='Show progress information')
    parser.add_argument('--list', '-l', action='store_true',
                        help='List matching files and exit')

    args = parser.parse_args()

    # Find matching files
    files = get_sorted_files(args.files)

    if not files:
        print(f"No files matching: {' '.join(args.patterns)}", file=sys.stderr)
        sys.exit(1)

    if args.list:
        print(f"Found {len(files)} files:")
        for f in files:
            print(f"  {f}")
        sys.exit(0)

    if args.verbose:
        print(f"Found {len(files)} dump files", file=sys.stderr)
        for f in files:
            print(f"  {f}", file=sys.stderr)

    # Connect to readsb
    if args.verbose:
        print(f"Connecting to {args.host}:{args.port}...", file=sys.stderr)

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((args.host, args.port))
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
    except Exception as e:
        print(f"Connection failed: {e}", file=sys.stderr)
        sys.exit(1)

    if args.verbose:
        print("Connected, starting playback...", file=sys.stderr)

    try:
        if args.stream:
            # Streaming mode - process files one at a time
            playback_streaming(files, sock, speed=args.speed, verbose=args.verbose,
                             skip_seconds=args.skip)
        else:
            # Load all messages first, then play back
            if args.verbose:
                print("Loading messages...", file=sys.stderr)

            messages = collect_messages(files, verbose=args.verbose)

            if not messages:
                print("No valid messages found in dump files", file=sys.stderr)
                sys.exit(1)

            if args.verbose:
                print(f"Total: {len(messages)} messages loaded", file=sys.stderr)

            playback_realtime(messages, sock, speed=args.speed, verbose=args.verbose,
                            skip_seconds=args.skip)
    finally:
        sock.close()


if __name__ == '__main__':
    main()