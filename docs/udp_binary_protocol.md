# IMU UDP Binary Protocol

## Overview

ATOM-S3 sends IMU batches to the Python server over UDP using a fixed-length binary packet.

Design goals:

- Reduce payload size versus JSON
- Keep each UDP packet independently decodable
- Avoid inter-packet delta encoding
- Allow packet loss detection with a sequence number
- Keep decoding simple on both ESP32 and Python

## Byte Order

All multi-byte fields are little-endian.

## Scaling Rules

- Acceleration values are encoded as `int16` with `1 LSB = 0.001 g`
- Gyro values are encoded as `int16` with `1 LSB = 0.01 dps`
- RSSI is encoded as `int16` in dBm
- Sample delta time is encoded as `uint16` in milliseconds

Decode formulas:

- `ax_g = raw_ax / 1000.0`
- `ay_g = raw_ay / 1000.0`
- `az_g = raw_az / 1000.0`
- `gx_dps = raw_gx / 100.0`
- `gy_dps = raw_gy / 100.0`
- `gz_dps = raw_gz / 100.0`

## Packet Layout

### Header

| Offset | Size | Type | Name | Description |
| --- | --- | --- | --- | --- |
| 0 | 2 | `uint16` | `magic` | Constant `0x5349` (`'I''S'`) |
| 2 | 1 | `uint8` | `version` | Protocol version, currently `1` |
| 3 | 1 | `uint8` | `flags` | Reserved, currently `0` |
| 4 | 4 | `uint32` | `seq` | Packet sequence number, increments per packet |
| 8 | 4 | `uint32` | `base_time_ms` | Timestamp of the first sample in the batch |
| 12 | 2 | `int16` | `rssi_dbm` | RSSI for the batch |
| 14 | 1 | `uint8` | `sample_count` | Number of samples in the packet |
| 15 | 1 | `uint8` | `reserved` | Reserved, currently `0` |

Header size: `16 bytes`

### Sample Record

| Offset in Record | Size | Type | Name | Description |
| --- | --- | --- | --- | --- |
| 0 | 2 | `uint16` | `dt_ms` | Milliseconds from `base_time_ms` |
| 2 | 2 | `int16` | `ax` | Acceleration X |
| 4 | 2 | `int16` | `ay` | Acceleration Y |
| 6 | 2 | `int16` | `az` | Acceleration Z |
| 8 | 2 | `int16` | `gx` | Gyro X |
| 10 | 2 | `int16` | `gy` | Gyro Y |
| 12 | 2 | `int16` | `gz` | Gyro Z |

Sample size: `14 bytes`

## Packet Size

Total packet size:

`16 + sample_count * 14`

Examples:

- `10 samples -> 156 bytes`
- `20 samples -> 296 bytes`

This is comfortably below the typical ESP32 UDP payload constraint that caused JSON packets to fail.

## Timestamp Reconstruction

For each sample:

`timestamp_ms = base_time_ms + dt_ms`

Because time is relative only within the packet, packets remain independent and can be decoded even if earlier packets were lost.

## Packet Loss Detection

The server tracks the `seq` field.

- If the current `seq` is exactly `last_seq + 1`, no gap was detected
- If the current `seq` is larger, one or more UDP packets were lost
- If the current `seq` is smaller, the packet may be duplicated, delayed, or the device may have restarted

No inter-packet delta state is required for decoding.

## Notes

- This protocol is intentionally simple and stable
- Reserved fields are kept for future extension
- If more compression is ever needed, the next step should be a version bump with optional flags, not an incompatible silent change
