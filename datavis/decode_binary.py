#!/usr/bin/env python3
import struct
import sys
import os
import math

# Expected columns matching the BinaryLogRecord layout exactly
# Original 20 fields + 3 raw mocap + 6 IMU = 29 float fields + 1 u32 timestamp
CSV_HEADER = "ts,x,y,yaw,x_des,y_des,yaw_des,v_ff,w_ff,v_actual,w_actual,omega_l_cmd,omega_r_cmd,omega_l_meas,omega_r_meas,duty_l,duty_r,x_err,y_err,yaw_err,x_raw,y_raw,yaw_raw,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z\n"
RECORD_FORMAT = "<I28f"
RECORD_SIZE = struct.calcsize(RECORD_FORMAT)  # 116 bytes
MAGIC_HEADER = b"\xaa\xbb\xcc\xdd"

# Also support the legacy 80-byte format (20 fields: 1 u32 + 19 f32)
LEGACY_RECORD_FORMAT = "<I19f"
LEGACY_RECORD_SIZE = struct.calcsize(LEGACY_RECORD_FORMAT)  # 80 bytes
LEGACY_CSV_HEADER = "ts,x,y,yaw,x_des,y_des,yaw_des,v_ff,w_ff,v_actual,w_actual,omega_l_cmd,omega_r_cmd,omega_l_meas,omega_r_meas,duty_l,duty_r,x_err,y_err,yaw_err\n"


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


def decode_file(input_path):
    if not os.path.exists(input_path):
        print(f"Error: File '{input_path}' does not exist.")
        return False
        
    print(f"Reading binary file: {input_path}")
    with open(input_path, "rb") as bin_file:
        # Verify magic header
        magic = bin_file.read(4)
        if magic != MAGIC_HEADER:
            print(f"Error: Invalid file format (expected magic header {MAGIC_HEADER.hex()}, got {magic.hex()})")
            return False
        
        # Detect format by checking file size after header
        current_pos = bin_file.tell()
        bin_file.seek(0, 2)  # seek to end
        data_size = bin_file.tell() - current_pos
        
        # Check if compact format
        is_compact = is_compact_format(bin_file, current_pos)
        bin_file.seek(current_pos)  # seek back
        
        records = []
        if is_compact:
            print("Detected Compact Tagged Binary format")
            csv_header = CSV_HEADER
            float_counts = {1: 6, 2: 5, 3: 2, 4: 3, 5: 2, 6: 2, 7: 6, 8: 5, 9: 6}
            current_row = [""] * 29
            current_ts = None
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
                
                # Coalesce rows within a 2ms window
                if current_ts is None or abs(t_ms - current_ts) > 2:
                    if current_ts is not None:
                        records.append(",".join(current_row))
                    current_row = [""] * 29
                    current_row[0] = str(t_ms)
                    current_ts = t_ms
                
                map_tag_to_row(tag, unpacked, current_row)
                
            if current_ts is not None:
                records.append(",".join(current_row))
        else:
            # Determine record size: try new format first, fall back to legacy
            if data_size > 0 and data_size % RECORD_SIZE == 0:
                record_format = RECORD_FORMAT
                record_size = RECORD_SIZE
                csv_header = CSV_HEADER
                print(f"Detected fixed 116-byte format ({record_size} bytes/record, {data_size // record_size} records)")
            elif data_size > 0 and data_size % LEGACY_RECORD_SIZE == 0:
                record_format = LEGACY_RECORD_FORMAT
                record_size = LEGACY_RECORD_SIZE
                csv_header = LEGACY_CSV_HEADER
                print(f"Detected legacy 80-byte format ({record_size} bytes/record, {data_size // record_size} records)")
            else:
                # Try new format first (may have trailing partial record)
                record_format = RECORD_FORMAT
                record_size = RECORD_SIZE
                csv_header = CSV_HEADER
                print(f"Warning: Data size {data_size} is not a clean multiple of record size. Trying fixed 116-byte format.")
                
            while True:
                chunk = bin_file.read(record_size)
                if not chunk:
                    break
                if len(chunk) < record_size:
                    print(f"Warning: Trailing partial record ignored ({len(chunk)} bytes)")
                    break
                    
                # Unpack record
                unpacked = struct.unpack(record_format, chunk)
                # unpacked[0] is t_ms (u32), rest are f32
                # Format: ts is integer, floats preserve NaN as empty
                row_parts = [str(unpacked[0])]
                for val in unpacked[1:]:
                    row_parts.append(format_value(val))
                row_str = ",".join(row_parts)
                records.append(row_str)
            
    # Overwrite the original binary file with the CSV content
    print(f"Overwriting binary file with CSV data: {input_path}")
    with open(input_path, "w") as csv_file:
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
    paths_to_decode = []
    
    for arg in sys.argv[1:]:
        if os.path.isdir(arg):
            print(f"Scanning directory: {arg}")
            for entry in sorted(os.listdir(arg)):
                full_path = os.path.join(arg, entry)
                if os.path.isfile(full_path) and entry.startswith("TR") and not entry.endswith(".png") and not entry.endswith(".csv"):
                    try:
                        with open(full_path, "rb") as f:
                            magic = f.read(4)
                            if magic == MAGIC_HEADER:
                                paths_to_decode.append(full_path)
                    except Exception:
                        pass
        else:
            paths_to_decode.append(arg)
            
    if not paths_to_decode:
        print("No matching binary log files found to decode.")
        return
        
    for path in paths_to_decode:
        if not decode_file(path):
            success = False
            
    if not success:
        sys.exit(1)

if __name__ == "__main__":
    main()
