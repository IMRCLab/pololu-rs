#!/usr/bin/env python3
import struct
import sys
import os
import math

CSV_HEADER = "ts,x,y,yaw,x_des,y_des,yaw_des,v_ff,w_ff,v_actual,w_actual,omega_l_cmd,omega_r_cmd,omega_l_meas,omega_r_meas,duty_l,duty_r,x_err,y_err,yaw_err,x_raw,y_raw,yaw_raw,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z\n"
MAGIC_HEADER = b"\xaa\xbb\xcc\xdd"


def format_value(val):
    """Format a float value, preserving NaN as empty string for CSV compatibility."""
    if math.isnan(val):
        return ""
    return f"{val:.6f}"


def map_tag_to_row(tag, unpacked, row):
    if tag == 1: # EkfState (x, y, z, roll, pitch, yaw)
        row[1] = format_value(unpacked[0]) # x
        row[2] = format_value(unpacked[1]) # y
        row[3] = format_value(unpacked[5]) # yaw
    elif tag == 2: # Setpoint (x_des, y_des, yaw_des, v_ff, w_ff)
        row[4] = format_value(unpacked[0]) # x_des
        row[5] = format_value(unpacked[1]) # y_des
        row[6] = format_value(unpacked[2]) # yaw_des
        row[7] = format_value(unpacked[3]) # v_ff
        row[8] = format_value(unpacked[4]) # w_ff
    elif tag == 3: # WheelCmd (omega_l, omega_r)
        row[11] = format_value(unpacked[0]) # omega_l_cmd
        row[12] = format_value(unpacked[1]) # omega_r_cmd
    elif tag == 4: # TrackingError (x_err, y_err, yaw_err)
        row[17] = format_value(unpacked[0]) # x_err
        row[18] = format_value(unpacked[1]) # y_err
        row[19] = format_value(unpacked[2]) # yaw_err
    elif tag == 5: # Motor (left, right)
        row[15] = format_value(unpacked[0]) # duty_l
        row[16] = format_value(unpacked[1]) # duty_r
    elif tag == 6: # Encoder (omega_l, omega_r)
        row[13] = format_value(unpacked[0]) # omega_l_meas
        row[14] = format_value(unpacked[1]) # omega_r_meas
    elif tag == 7: # Mocap (x, y, z, roll, pitch, yaw)
        row[20] = format_value(unpacked[0]) # x_raw
        row[21] = format_value(unpacked[1]) # y_raw
        row[22] = format_value(unpacked[5]) # yaw_raw
    elif tag == 8: # Odom (x, y, theta, v, w)
        row[9] = format_value(unpacked[3])  # v_actual
        row[10] = format_value(unpacked[4]) # w_actual
    elif tag == 9: # Imu (acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z)
        row[23] = format_value(unpacked[0]) # acc_x
        row[24] = format_value(unpacked[1]) # acc_y
        row[25] = format_value(unpacked[2]) # acc_z
        row[26] = format_value(unpacked[3]) # gyro_x
        row[27] = format_value(unpacked[4]) # gyro_y
        row[28] = format_value(unpacked[5]) # gyro_z


def is_compact_format(bin_file, start_pos):
    bin_file.seek(start_pos)
    float_counts = {1: 6, 2: 5, 3: 2, 4: 3, 5: 2, 6: 2, 7: 6, 8: 5, 9: 6}
    count = 0
    while count < 10:
        header = bin_file.read(5)
        if not header:
            return count > 0
        if len(header) < 5:
            return count > 0
        t_ms, tag = struct.unpack("<IB", header)
        if tag not in float_counts:
            return False
        num_floats = float_counts[tag]
        payload = bin_file.read(num_floats * 4)
        if len(payload) < num_floats * 4:
            return count > 0
        count += 1
    return True


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
        # Verify magic header
        magic = bin_file.read(4)
        if magic != MAGIC_HEADER:
            print(f"Error: Invalid file format (expected magic header {MAGIC_HEADER.hex()}, got {magic.hex()})")
            return False
        
        current_pos = bin_file.tell()
        is_compact = is_compact_format(bin_file, current_pos)
        bin_file.seek(current_pos)  # seek back
        
        records = []
        if is_compact:
            print("Detected Compact Tagged Binary format")
            csv_header = CSV_HEADER
            float_counts = {1: 6, 2: 5, 3: 2, 4: 3, 5: 2, 6: 2, 7: 6, 8: 5, 9: 6}
            while True:
                header_chunk = bin_file.read(5)
                if not header_chunk:
                    break
                if len(header_chunk) < 5:
                    print("Warning: Trailing partial header ignored")
                    break
                t_ms, tag = struct.unpack("<IB", header_chunk)
                if tag not in float_counts:
                    print(f"Warning: Invalid tag {tag} at timestamp {t_ms}, skipping rest of file")
                    break
                num_floats = float_counts[tag]
                payload_chunk = bin_file.read(num_floats * 4)
                if len(payload_chunk) < num_floats * 4:
                    print("Warning: Trailing partial payload ignored")
                    break
                unpacked = struct.unpack(f"<{num_floats}f", payload_chunk)

                current_row = [""] * 29
                current_row[0] = str(t_ms)
                map_tag_to_row(tag, unpacked, current_row)
                records.append(",".join(current_row))
        else:
            print(f"Error: Unsupported legacy binary format. Only Compact Tagged Binary logs can be decoded.")
            return False
            
    os.makedirs(os.path.dirname(output_path) or ".", exist_ok=True)
    print(f"Writing CSV data: {output_path}")
    with open(output_path, "w") as csv_file:
        csv_file.write(csv_header)
        for record in records:
            csv_file.write(record + "\n")
            
    print(f"Successfully decoded {len(records)} log entries.")
    return True

def main():
    if len(sys.argv) < 2:
        print("Usage: python decode_binary.py <path_or_dir_1> [<path_or_dir_2> ...]")
        sys.exit(1)
        
    success = True
    decode_jobs = []
    
    for arg in sys.argv[1:]:
        if os.path.isdir(arg):
            print(f"Scanning directory: {arg}")
            output_dir = os.path.join(arg, "decoded")
            for entry in sorted(os.listdir(arg)):
                full_path = os.path.join(arg, entry)
                if os.path.isfile(full_path) and entry.startswith("TR") and not entry.endswith(".png") and not entry.endswith(".csv"):
                    try:
                        with open(full_path, "rb") as f:
                            magic = f.read(4)
                            if magic == MAGIC_HEADER:
                                decode_jobs.append((full_path, csv_output_path(full_path, output_dir)))
                    except Exception:
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
