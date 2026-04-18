import math
import json
import socket
import struct
from datetime import datetime
import io
import csv
import time
from collections import deque
from pathlib import Path
from threading import Event, Lock, Thread
from typing import Any

from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse, Response


app = FastAPI()
BASE_DIR = Path(__file__).resolve().parent
DASHBOARD_FILE = BASE_DIR / "dashboard.html"
MAX_HISTORY = 300
UDP_HOST = "0.0.0.0"
UDP_PORT = 8001
UDP_BUFFER_SIZE = 65535
UDP_PACKET_MAGIC = 0x5349
UDP_PACKET_VERSION = 1
UDP_HEADER_BYTES = 16
UDP_SAMPLE_STRUCT = struct.Struct("<Hhhhhhh")
ACCEL_SCALE = 1000.0
GYRO_SCALE = 100.0
SERVER_DISCOVERY_REQUEST = b"SI_DISCOVER_V1"
SERVER_DISCOVERY_RESPONSE = b"SI_SERVER_V1"


latest_data = {
    "device": "none",
    "timestamp_ms": 0,
    "ax": 0.0,
    "ay": 0.0,
    "az": 0.0,
    "gx": 0.0,
    "gy": 0.0,
    "gz": 0.0,
    "rssi_dbm": -127,
    "server_received_at": 0.0,
    "packet_seq": None,
}
history = deque(maxlen=MAX_HISTORY)
sample_seq = 0
recording_active = False
recording_started_at = 0.0
recorded_samples: list[dict[str, Any]] = []
lock = Lock()
udp_stop_event = Event()
udp_thread: Thread | None = None
last_packet_seq: int | None = None
lost_packet_count = 0


def normalize_samples(payload: dict[str, Any]) -> tuple[str, int, list[dict[str, Any]]]:
    device = str(payload.get("device", "unknown"))
    rssi_dbm = int(payload.get("rssi_dbm", -127))

    if isinstance(payload.get("samples"), list):
        raw_samples = payload["samples"]
    else:
        raw_samples = [payload]

    samples: list[dict[str, Any]] = []
    for raw in raw_samples:
        if not isinstance(raw, dict):
            raise HTTPException(status_code=400, detail="Each sample must be an object")

        samples.append(
            {
                "device": device,
                "timestamp_ms": int(raw.get("timestamp_ms", 0)),
                "ax": float(raw.get("ax", 0.0)),
                "ay": float(raw.get("ay", 0.0)),
                "az": float(raw.get("az", 0.0)),
                "gx": float(raw.get("gx", 0.0) or 0.0),
                "gy": float(raw.get("gy", 0.0) or 0.0),
                "gz": float(raw.get("gz", 0.0) or 0.0),
            }
        )

    return device, rssi_dbm, samples


def compute_orientation(data: dict[str, Any]) -> dict[str, Any]:
    ax = float(data["ax"])
    ay = float(data["ay"])
    az = float(data["az"])

    gravity = math.sqrt(ax * ax + ay * ay + az * az) or 1.0
    roll_deg = math.degrees(math.atan2(ay, az))
    pitch_deg = math.degrees(math.atan2(-ax, math.sqrt(ay * ay + az * az)))

    horizontal_x = ax / gravity
    horizontal_y = ay / gravity
    tilt_strength = min(math.sqrt(horizontal_x * horizontal_x + horizontal_y * horizontal_y), 1.0)
    heading_deg = (math.degrees(math.atan2(horizontal_y, horizontal_x)) + 360.0) % 360.0

    return {
        "roll_deg": round(roll_deg, 1),
        "pitch_deg": round(pitch_deg, 1),
        "tilt_strength": round(tilt_strength, 3),
        "heading_deg": round(heading_deg, 1),
        "top_view_vector": {
            "x": round(horizontal_x, 4),
            "y": round(horizontal_y, 4),
        },
    }


def decode_udp_packet(packet: bytes) -> tuple[int, str, int, list[dict[str, Any]]]:
    if len(packet) < UDP_HEADER_BYTES:
        raise ValueError("packet too short")

    magic = int.from_bytes(packet[0:2], "little")
    if magic != UDP_PACKET_MAGIC:
        raise ValueError("invalid magic")

    version = packet[2]
    if version != UDP_PACKET_VERSION:
        raise ValueError(f"unsupported version: {version}")

    packet_seq = int.from_bytes(packet[4:8], "little")
    base_time_ms = int.from_bytes(packet[8:12], "little")
    rssi_dbm = int.from_bytes(packet[12:14], "little", signed=True)
    sample_count = packet[14]
    expected_size = UDP_HEADER_BYTES + sample_count * UDP_SAMPLE_STRUCT.size
    if len(packet) != expected_size:
        raise ValueError(f"invalid packet size: expected {expected_size}, got {len(packet)}")

    samples: list[dict[str, Any]] = []
    for index in range(sample_count):
        offset = UDP_HEADER_BYTES + index * UDP_SAMPLE_STRUCT.size
        dt_ms, ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw = UDP_SAMPLE_STRUCT.unpack_from(packet, offset)
        samples.append(
            {
                "device": "atom-s3",
                "timestamp_ms": base_time_ms + dt_ms,
                "ax": ax_raw / ACCEL_SCALE,
                "ay": ay_raw / ACCEL_SCALE,
                "az": az_raw / ACCEL_SCALE,
                "gx": gx_raw / GYRO_SCALE,
                "gy": gy_raw / GYRO_SCALE,
                "gz": gz_raw / GYRO_SCALE,
                "packet_seq": packet_seq,
            }
        )

    return packet_seq, "atom-s3", rssi_dbm, samples


def handle_discovery_packet(sock: socket.socket, packet: bytes, addr: tuple[str, int]) -> bool:
    if packet != SERVER_DISCOVERY_REQUEST:
        return False

    sock.sendto(SERVER_DISCOVERY_RESPONSE, addr)
    return True


def store_samples(device: str, rssi_dbm: int, samples: list[dict[str, Any]], now: float) -> dict[str, Any]:
    global sample_seq

    with lock:
        for sample in samples:
            sample_seq += 1
            latest_data.update(sample)
            latest_data["device"] = device
            latest_data["rssi_dbm"] = rssi_dbm
            latest_data["server_received_at"] = now
            if "packet_seq" in sample:
                latest_data["packet_seq"] = sample["packet_seq"]

            history.append(
                {
                    "seq": sample_seq,
                    "device": device,
                    "timestamp_ms": sample["timestamp_ms"],
                    "ax": sample["ax"],
                    "ay": sample["ay"],
                    "az": sample["az"],
                    "gx": sample["gx"],
                    "gy": sample["gy"],
                    "gz": sample["gz"],
                    "rssi_dbm": rssi_dbm,
                    "server_received_at": now,
                    "packet_seq": sample.get("packet_seq"),
                }
            )

            if recording_active:
                recorded_samples.append(
                    {
                        "seq": sample_seq,
                        "device": device,
                        "timestamp_ms": sample["timestamp_ms"],
                        "ax": sample["ax"],
                        "ay": sample["ay"],
                        "az": sample["az"],
                        "gx": sample["gx"],
                        "gy": sample["gy"],
                        "gz": sample["gz"],
                        "rssi_dbm": rssi_dbm,
                        "server_received_at": now,
                        "packet_seq": sample.get("packet_seq"),
                    }
                )

        latest = dict(latest_data)
        count = len(history)
        recording_count = len(recorded_samples)

    return {
        "accepted": len(samples),
        "history_size": count,
        "recording_count": recording_count,
        "latest": latest,
    }


def udp_listener() -> None:
    global last_packet_seq, lost_packet_count

    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind((UDP_HOST, UDP_PORT))
        sock.settimeout(0.5)
        print(f"UDP listening on {UDP_HOST}:{UDP_PORT}")

        while not udp_stop_event.is_set():
            try:
                packet, addr = sock.recvfrom(UDP_BUFFER_SIZE)
            except socket.timeout:
                continue
            except OSError:
                break

            try:
                if handle_discovery_packet(sock, packet, addr):
                    continue
                packet_seq, device, rssi_dbm, samples = decode_udp_packet(packet)
                with lock:
                    if last_packet_seq is not None and packet_seq > last_packet_seq + 1:
                        lost_packet_count += packet_seq - last_packet_seq - 1
                    last_packet_seq = packet_seq
                store_samples(device, rssi_dbm, samples, time.time())
            except Exception as exc:
                print(f"UDP packet ignored from {addr}: {exc}")


@app.on_event("startup")
def startup_event() -> None:
    global udp_thread

    udp_stop_event.clear()
    udp_thread = Thread(target=udp_listener, name="imu-udp-listener", daemon=True)
    udp_thread.start()


@app.on_event("shutdown")
def shutdown_event() -> None:
    udp_stop_event.set()


@app.post("/api/imu")
def receive_imu(payload: dict[str, Any]):
    device, rssi_dbm, samples = normalize_samples(payload)
    result = store_samples(device, rssi_dbm, samples, time.time())
    return {"status": "ok", **result}


@app.get("/api/latest")
def get_latest():
    with lock:
        return dict(latest_data)


@app.get("/api/state")
def get_state():
    with lock:
        latest = dict(latest_data)
        records = list(history)
        recording = {
            "active": recording_active,
            "count": len(recorded_samples),
            "started_at": recording_started_at,
        }
        udp_stats = {
            "last_packet_seq": last_packet_seq,
            "lost_packet_count": lost_packet_count,
        }

    orientation = compute_orientation(latest) if records else None
    return {
        "device": latest["device"],
        "count": len(records),
        "latest": latest,
        "orientation": orientation,
        "history": records,
        "recording": recording,
        "udp": udp_stats,
    }


@app.post("/api/recording/start")
def start_recording():
    global recording_active, recording_started_at, recorded_samples

    with lock:
        recording_active = True
        recording_started_at = time.time()
        recorded_samples = []
        recording = {
            "active": recording_active,
            "count": 0,
            "started_at": recording_started_at,
        }

    return {"status": "ok", "recording": recording}


@app.post("/api/recording/stop")
def stop_recording():
    global recording_active, recording_started_at, recorded_samples

    with lock:
        export_samples = list(recorded_samples)
        started_at = recording_started_at
        recording_active = False
        recording_started_at = 0.0
        recorded_samples = []

    if not export_samples:
        raise HTTPException(status_code=400, detail="No recorded samples to export")

    buffer = io.StringIO()
    writer = csv.DictWriter(
        buffer,
        fieldnames=[
            "seq",
            "packet_seq",
            "device",
            "timestamp_ms",
            "ax",
            "ay",
            "az",
            "gx",
            "gy",
            "gz",
            "rssi_dbm",
            "server_received_at",
        ],
    )
    writer.writeheader()
    writer.writerows(export_samples)

    timestamp = datetime.fromtimestamp(started_at or time.time()).strftime("%Y%m%d_%H%M%S")
    filename = f"imu_recording_{timestamp}.csv"
    return Response(
        content=buffer.getvalue(),
        media_type="text/csv; charset=utf-8",
        headers={"Content-Disposition": f'attachment; filename="{filename}"'},
    )


@app.get("/")
def index():
    return FileResponse(DASHBOARD_FILE)
