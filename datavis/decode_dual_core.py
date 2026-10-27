#!/usr/bin/env python3
import math
import os
import struct
import sys


CSV_HEADER = "ts,x,y,yaw,x_des,y_des,yaw_des,v_ff,w_ff,v_actual,w_actual,omega_l_cmd,omega_r_cmd,omega_l_meas,omega_r_meas,duty_l,duty_r,x_err,y_err,yaw_err,x_raw,y_raw,yaw_raw,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z\n"
MAGIC_HEADER = b"\xaa\xbb\xcc\xdd"
DUAL_CORE_MAGIC = b"DCLG"
RECORD_FORMATS = {
    1: (129, struct.Struct("<IQBI25f3I")),
    2: (141, struct.Struct("<IQBI28f3I")),
}

LOG_VALID_POSE = 1 << 1
LOG_VALID_SETPOINT = 1 << 2
LOG_VALID_WHEEL_TARGET = 1 << 3
LOG_VALID_WHEEL_MEASUREMENT = 1 << 4
LOG_VALID_MOTOR = 1 << 5
LOG_VALID_TRACKING_ERROR = 1 << 6
LOG_VALID_IMU = 1 << 7
LOG_VALID_MOCAP = 1 << 9


def format_value(value):
    if math.isnan(value):
        return ""
    return f"{value:.6f}"


def new_csv_row(timestamp_ms):
    row = [""] * 29
    row[0] = str(timestamp_ms)
    return row


def map_frame_to_row(frame, version):
    _, timestamp_us, _, valid_mask, *payload = frame
    row = new_csv_row(timestamp_us // 1000)

    if valid_mask & LOG_VALID_POSE:
        for column, value in zip((1, 2, 3), payload[2:5]):
            row[column] = format_value(value)

    if valid_mask & LOG_VALID_SETPOINT:
        for column, value in zip((4, 5, 6, 7, 8), payload[5:10]):
            row[column] = format_value(value)

    if valid_mask & LOG_VALID_WHEEL_TARGET:
        for column, value in zip((11, 12), payload[10:12]):
            row[column] = format_value(value)

    if valid_mask & LOG_VALID_WHEEL_MEASUREMENT:
        for column, value in zip((13, 14), payload[12:14]):
            row[column] = format_value(value)

    if valid_mask & LOG_VALID_MOTOR:
        for column, value in zip((15, 16), payload[14:16]):
            row[column] = format_value(value)

    if valid_mask & LOG_VALID_TRACKING_ERROR:
        for column, value in zip((17, 18, 19), payload[16:19]):
            row[column] = format_value(value)

    if version >= 2 and valid_mask & LOG_VALID_MOCAP:
        for column, value in zip((20, 21, 22), payload[19:22]):
            row[column] = format_value(value)

    if valid_mask & LOG_VALID_IMU:
        imu_offset = 22 if version >= 2 else 19
        for column, value in zip(range(23, 29), payload[imu_offset : imu_offset + 6]):
            row[column] = format_value(value)

    return row


def csv_output_path(input_path, output_dir=None):
    filename = os.path.basename(input_path) + ".csv"
    if output_dir is not None:
        return os.path.join(output_dir, filename)
    return os.path.join(os.path.dirname(input_path), filename)


def decode_file(input_path, output_path=None):
    if not os.path.exists(input_path):
        print(f"Error: File '{input_path}' does not exist.")
        return False
    if output_path is None:
        output_path = csv_output_path(input_path)

    print(f"Reading binary file: {input_path}")
    with open(input_path, "rb") as bin_file:
        magic = bin_file.read(4)
        if magic != MAGIC_HEADER:
            print(
                f"Error: Invalid file format (expected magic header "
                f"{MAGIC_HEADER.hex()}, got {magic.hex()})"
            )
            return False

        dual_core_magic = bin_file.read(4)
        if dual_core_magic != DUAL_CORE_MAGIC:
            print(
                "Error: Missing DCLG header; this is not a dual-core log."
            )
            return False

        header = bin_file.read(4)
        if len(header) != 4:
            print("Error: Truncated DCLG format header.")
            return False

        version, record_size = struct.unpack("<HH", header)
        if version not in RECORD_FORMATS:
            print(f"Error: Unsupported DCLG version {version}.")
            return False
        expected_size, record_struct = RECORD_FORMATS[version]
        if record_size != expected_size:
            print(
                f"Error: Unexpected record size {record_size}; "
                f"expected {expected_size}."
            )
            return False

        print(f"Detected DCLG v{version} UnifiedLogFrame format")
        rows = []
        while True:
            record_bytes = bin_file.read(record_size)
            if not record_bytes:
                break
            if len(record_bytes) != record_size:
                print("Warning: Trailing partial record ignored")
                break
            rows.append(map_frame_to_row(record_struct.unpack(record_bytes), version))

    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
    print(f"Writing CSV data: {output_path}")
    with open(output_path, "w") as csv_file:
        csv_file.write(CSV_HEADER)
        for row in rows:
            csv_file.write(",".join(row) + "\n")

    print(f"Successfully decoded {len(rows)} log entries.")
    return True


def main():
    if len(sys.argv) < 2:
        print("Usage: python decode_dual_core.py <path_or_dir_1> [<path_or_dir_2> ...]")
        sys.exit(1)

    success = True
    decode_jobs = []

    for arg in sys.argv[1:]:
        if os.path.isdir(arg):
            print(f"Scanning directory: {arg}")
            output_dir = os.path.join(arg, "decoded")
            for entry in sorted(os.listdir(arg)):
                full_path = os.path.join(arg, entry)
                if (
                    os.path.isfile(full_path)
                    and entry.startswith("TR")
                    and not entry.endswith((".png", ".csv"))
                ):
                    try:
                        with open(full_path, "rb") as bin_file:
                            if bin_file.read(4) == MAGIC_HEADER:
                                decode_jobs.append(
                                    (full_path, csv_output_path(full_path, output_dir))
                                )
                    except OSError:
                        pass
        else:
            decode_jobs.append((arg, csv_output_path(arg)))

    if not decode_jobs:
        print("No matching binary log files found to decode.")
        return

    for input_path, output_path in decode_jobs:
        if not decode_file(input_path, output_path):
            success = False

    if not success:
        sys.exit(1)


if __name__ == "__main__":
    main()