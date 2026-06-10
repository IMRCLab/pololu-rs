#!/usr/bin/env python3
import struct
import sys
import os

# Expected columns matching the original CSV header exactly
CSV_HEADER = "ts,x,y,yaw,x_des,y_des,yaw_des,v_ff,w_ff,v_actual,w_actual,omega_l_cmd,omega_r_cmd,omega_l_meas,omega_r_meas,duty_l,duty_r,x_err,y_err,yaw_err\n"
RECORD_FORMAT = "<I19f"
RECORD_SIZE = struct.calcsize(RECORD_FORMAT) # 80 bytes
MAGIC_HEADER = b"\xaa\xbb\xcc\xdd"

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
            
        records = []
        while True:
            chunk = bin_file.read(RECORD_SIZE)
            if not chunk:
                break
            if len(chunk) < RECORD_SIZE:
                print(f"Warning: Trailing partial record ignored ({len(chunk)} bytes)")
                break
                
            # Unpack record
            unpacked = struct.unpack(RECORD_FORMAT, chunk)
            # unpacked is a tuple: (t_ms, x, y, yaw, ..., yaw_err)
            # Format numbers to make sure they are readable and clear
            # ts is u32 (int), others are f32 (float)
            row_str = f"{unpacked[0]}" + "".join(f",{val:.6f}" for val in unpacked[1:])
            records.append(row_str)
            
    # Output path is original name + ".csv"
    output_path = input_path + ".csv"
    
    print(f"Writing CSV data to: {output_path}")
    with open(output_path, "w") as csv_file:
        csv_file.write(CSV_HEADER)
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
