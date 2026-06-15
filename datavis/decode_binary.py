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
        bin_file.seek(current_pos)  # seek back
        
        # Determine record size: try new format first, fall back to legacy
        if data_size > 0 and data_size % RECORD_SIZE == 0:
            record_format = RECORD_FORMAT
            record_size = RECORD_SIZE
            csv_header = CSV_HEADER
            num_floats = 28
            print(f"Detected new format ({record_size} bytes/record, {data_size // record_size} records)")
        elif data_size > 0 and data_size % LEGACY_RECORD_SIZE == 0:
            record_format = LEGACY_RECORD_FORMAT
            record_size = LEGACY_RECORD_SIZE
            csv_header = LEGACY_CSV_HEADER
            num_floats = 19
            print(f"Detected legacy format ({record_size} bytes/record, {data_size // record_size} records)")
        else:
            # Try new format first (may have trailing partial record)
            record_format = RECORD_FORMAT
            record_size = RECORD_SIZE
            csv_header = CSV_HEADER
            num_floats = 28
            print(f"Warning: Data size {data_size} is not a clean multiple of record size. Trying new format ({record_size} bytes).")
            
        records = []
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
        print("Usage: python decode_binary.py <binary_log_file_1> [<binary_log_file_2> ...]")
        sys.exit(1)
        
    success = True
    for path in sys.argv[1:]:
        if not decode_file(path):
            success = False
            
    if not success:
        sys.exit(1)

if __name__ == "__main__":
    main()
