Got it — so we’ll remove both `cmd_v` **and** `cmd_omega` from the plots.
The subplots will now only show:

* Error
* u\_k
* Target RPM
* Raw RPM
* Filtered RPM

Here’s the final plotting function for your script:

```python
def plot_subplots(df: pd.DataFrame, out_png: Path | None):
    fig, axes = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    # Left wheel subplot
    ax = axes[0]
    if "left_error" in df:       ax.plot(df["t"], df["left_error"], label="Error")
    if "left_u_k" in df:         ax.plot(df["t"], df["left_u_k"], label="u_k")
    if "left_target" in df:      ax.plot(df["t"], df["left_target"], label="Target RPM")
    if "left_raw" in df:         ax.plot(df["t"], df["left_raw"], label="Raw RPM")
    if "left_filtered" in df:    ax.plot(df["t"], df["left_filtered"], label="Filtered RPM")
    ax.set_title("Left Wheel")
    ax.set_ylabel("Value")
    ax.grid(True)
    ax.legend()

    # Right wheel subplot
    ax = axes[1]
    if "right_error" in df:      ax.plot(df["t"], df["right_error"], label="Error")
    if "right_u_k" in df:        ax.plot(df["t"], df["right_u_k"], label="u_k")
    if "right_target" in df:     ax.plot(df["t"], df["right_target"], label="Target RPM")
    if "right_raw" in df:        ax.plot(df["t"], df["right_raw"], label="Raw RPM")
    if "right_filtered" in df:   ax.plot(df["t"], df["right_filtered"], label="Filtered RPM")
    ax.set_title("Right Wheel")
    ax.set_xlabel("t (log step)")
    ax.set_ylabel("Value")
    ax.grid(True)
    ax.legend()

    plt.tight_layout()
    if out_png is not None:
        plt.savefig(out_png, dpi=150)
    plt.show()
```

Do you want me to update the **full script** I gave you earlier with this change so you have the final cleaned version in one place?
