#!/usr/bin/env bash
set -euo pipefail

if [ "$#" -lt 1 ]; then
    echo "Usage: $0 <binary-log-directory> [plot-script-args...]"
    exit 1
fi

log_dir=$1
shift

if [ ! -d "$log_dir" ]; then
    echo "Error: '$log_dir' is not a directory." >&2
    exit 1
fi

script_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
decoded_dir="$log_dir/decoded"
plots_dir="$log_dir/plots"

python3 "$script_dir/decode_binary.py" "$log_dir"
python3 "$script_dir/plot_pololu_log_standalone.py" --log "$decoded_dir" --output "$plots_dir" "$@"

echo "Decoded CSV files: $decoded_dir"
echo "Plots: $plots_dir"
