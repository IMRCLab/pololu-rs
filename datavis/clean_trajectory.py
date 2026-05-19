#!/usr/bin/env python3
"""
Clean individual trajectory data.
This script processes a single trajectory file (TR00 or TR02).
"""

import re
from pathlib import Path

def clean_single_trajectory(trajectory_name):
    """Clean a single trajectory file and create the final CSV."""
    script_dir = Path(__file__).parent
    input_file = script_dir / trajectory_name
    output_file = script_dir / f"{trajectory_name}_final.csv"
    
    if not input_file.exists():
        print(f"Error: File '{trajectory_name}' not found!")
        return False
    
    try:
        with open(input_file, 'r') as f:
            content = f.read().strip()
        
        print(f"Processing {trajectory_name}...")
        
        lines = content.split('\n')
        header = lines[0]
        
        if len(lines) == 2:
            data_line = lines[1]
            print(f"Data line length: {len(data_line)} characters")
            
            # Generate the expected timestamp sequence
            timestamps = ['100']  # First timestamp
            for i in range(200, 10300, 100):  # 0200, 0300, ..., 010200
                timestamps.append(f'0{i}' if i < 10000 else f'0{i}')
            
            print(f"Looking for {len(timestamps)} timestamps")
            
            # Split the data by timestamp boundaries
            rows = []
            remaining_data = data_line
            
            for i, ts in enumerate(timestamps):
                if i == 0:
                    # First timestamp should be at the beginning
                    if remaining_data.startswith(ts + ','):
                        # Find where the next timestamp starts
                        next_ts = timestamps[i + 1] if i + 1 < len(timestamps) else None
                        if next_ts:
                            next_pos = remaining_data.find(',' + next_ts + ',')
                            if next_pos == -1:
                                next_pos = remaining_data.find(next_ts + ',', 1)
                            if next_pos != -1:
                                row_data = remaining_data[:next_pos]
                                remaining_data = remaining_data[next_pos + 1:]
                            else:
                                row_data = remaining_data
                                remaining_data = ""
                        else:
                            row_data = remaining_data
                            remaining_data = ""
                        
                        # Validate that we have correct number of values
                        values = row_data.split(',')
                        if len(values) == 19:  # Missing motor_r, add it
                            row_data += ',0'
                            values.append('0')
                        
                        if len(values) == 20:
                            rows.append(row_data)
                else:
                    # Subsequent timestamps
                    if remaining_data.startswith(ts + ','):
                        next_ts = timestamps[i + 1] if i + 1 < len(timestamps) else None
                        if next_ts:
                            next_pos = remaining_data.find(',' + next_ts + ',')
                            if next_pos == -1:
                                next_pos = remaining_data.find(next_ts + ',', 1)
                            if next_pos != -1:
                                row_data = remaining_data[:next_pos]
                                remaining_data = remaining_data[next_pos + 1:]
                            else:
                                row_data = remaining_data
                                remaining_data = ""
                        else:
                            row_data = remaining_data
                            remaining_data = ""
                        
                        values = row_data.split(',')
                        if len(values) == 19:  # Missing motor_r, add it
                            row_data += ',0'
                            values.append('0')
                            
                        if len(values) == 20:
                            rows.append(row_data)
                    else:
                        break
            
            print(f"Successfully extracted {len(rows)} rows")
            
            # Write the cleaned data
            with open(output_file, 'w') as f:
                f.write(header + '\n')
                for row in rows:
                    f.write(row + '\n')
            
            print(f"Successfully wrote {len(rows)} rows to {output_file}")
            return True
        else:
            print(f"Unexpected format: {len(lines)} lines")
            return False
            
    except Exception as e:
        print(f"Error processing {trajectory_name}: {e}")
        return False

if __name__ == "__main__":
    import sys
    if len(sys.argv) != 2:
        print("Usage: python3 clean_trajectory.py <trajectory_name>")
        print("Example: python3 clean_trajectory.py TR00")
        sys.exit(1)
    
    trajectory = sys.argv[1]
    success = clean_single_trajectory(trajectory)
    if not success:
        sys.exit(1)
