"""
analyze_comparison.py — Compare NoloClientAPI vs hidAPI data from a compare_output.csv.

Usage:
    python test/analyze_comparison.py test/compare_output.csv [--out-dir test/analysis]

Outputs:
    - Correlation table (stdout)
    - Top button-byte candidates (stdout)
    - PNG plots: axis alignment and button-byte correlations
    - analysis_report.txt with full findings
"""

import argparse
import os
import sys
import warnings
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

warnings.filterwarnings("ignore", category=RuntimeWarning, module="numpy")

HID_BYTE_COLS = [f"hid_b{i:02d}" for i in range(64)]

KNOWN_BUTTONS = {
    0x01: "touchpad_click",
    0x02: "trigger",
    0x04: "menu",
    0x08: "system",
    0x10: "grip",
    0x20: "touchpad_touch",
}

POSITION_COLS = ["pos_x", "pos_y", "pos_z"]
ORIENT_COLS   = ["rot_w", "rot_x", "rot_y", "rot_z"]


def load_csv(path: str) -> pd.DataFrame:
    df = pd.read_csv(path)
    df["timestamp_ms"] = pd.to_numeric(df["timestamp_ms"], errors="coerce")
    df = df.dropna(subset=["timestamp_ms"])
    df = df.sort_values("timestamp_ms").reset_index(drop=True)
    return df


def split_sources(df: pd.DataFrame):
    capi = df[df["source_api"] == "client_api"].copy()
    hid  = df[df["source_api"] == "hid"].copy()
    return capi, hid


def align_nearest(df_a: pd.DataFrame, df_b: pd.DataFrame, max_gap_ms: int = 50) -> pd.DataFrame:
    """Merge two per-device DataFrames on nearest timestamp within max_gap_ms."""
    merged = pd.merge_asof(
        df_a.sort_values("timestamp_ms"),
        df_b.sort_values("timestamp_ms"),
        on="timestamp_ms",
        suffixes=("_ca", "_hid"),
        direction="nearest",
        tolerance=int(max_gap_ms),
    )
    return merged.dropna(subset=[f"{POSITION_COLS[0]}_ca", f"{POSITION_COLS[0]}_hid"])


def axis_correlation_table(aligned: pd.DataFrame) -> pd.DataFrame:
    """3×3 Pearson correlation between client_api axes and hid axes, plus negated."""
    results = []
    for ca_ax in POSITION_COLS:
        for hid_ax in POSITION_COLS:
            ca_col  = f"{ca_ax}_ca"
            hid_col = f"{hid_ax}_hid"
            if ca_col not in aligned.columns or hid_col not in aligned.columns:
                continue
            ca_vals  = aligned[ca_col].values.astype(float)
            hid_vals = aligned[hid_col].values.astype(float)
            if len(ca_vals) < 2 or ca_vals.std() == 0 or hid_vals.std() == 0:
                continue
            corr_pos = float(np.corrcoef(ca_vals, hid_vals)[0, 1])
            corr_neg = float(np.corrcoef(ca_vals, -hid_vals)[0, 1])
            if np.isnan(corr_pos): corr_pos = 0.0
            if np.isnan(corr_neg): corr_neg = 0.0
            results.append({
                "ca_axis": ca_ax, "hid_axis": hid_ax,
                "corr": round(corr_pos, 4), "corr_negated": round(corr_neg, 4),
                "best": round(max(corr_pos, corr_neg, key=abs), 4),
                "best_sign": "+" if abs(corr_pos) >= abs(corr_neg) else "-",
            })
    return pd.DataFrame(results)


def button_byte_candidates(
    capi: pd.DataFrame,
    hid: pd.DataFrame,
    device: str,
    out_dir: str,
    report_lines: list,
) -> pd.DataFrame:
    """
    For each known button bit, find HID bytes whose value changes when the button is pressed.
    Returns a DataFrame ranking byte positions by mean absolute difference (pressed vs. not pressed).
    """
    ca_dev  = capi[capi["device"] == device].copy()
    hid_dev = hid[hid["device"] == device].copy()

    if ca_dev.empty or hid_dev.empty:
        report_lines.append(f"\n[{device}] no data for button analysis")
        return pd.DataFrame()

    # Ensure hid_byte columns are numeric
    for col in HID_BYTE_COLS:
        if col in hid_dev.columns:
            hid_dev[col] = pd.to_numeric(hid_dev[col], errors="coerce").fillna(0)

    # align on nearest timestamp
    aligned = pd.merge_asof(
        ca_dev.sort_values("timestamp_ms")[["timestamp_ms", "buttons"]],
        hid_dev.sort_values("timestamp_ms")[["timestamp_ms"] + HID_BYTE_COLS],
        on="timestamp_ms",
        direction="nearest",
        tolerance=50,    ).dropna(subset=["buttons"])

    if aligned.empty:
        report_lines.append(f"\n[{device}] could not align client_api and hid data (max gap 50 ms)")
        return pd.DataFrame()

    all_results = []
    for bit, name in KNOWN_BUTTONS.items():
        pressed     = aligned["buttons"].astype(int) & bit != 0
        n_pressed   = int(pressed.sum())
        n_released  = int((~pressed).sum())
        if n_pressed < 3 or n_released < 3:
            report_lines.append(f"\n[{device}] {name} (0x{bit:02x}): too few samples "
                                  f"(pressed={n_pressed}, released={n_released}) — press the button during capture")
            continue

        row_results = {"button": name, "bit": f"0x{bit:02x}"}
        for col in HID_BYTE_COLS:
            if col not in aligned.columns:
                row_results[col] = 0.0
                continue
            vals = aligned[col].values.astype(float)
            mean_pressed  = vals[pressed].mean()
            mean_released = vals[~pressed].mean()
            row_results[col] = abs(mean_pressed - mean_released)
        all_results.append(row_results)

    if not all_results:
        report_lines.append(f"\n[{device}] no button presses detected — press buttons during capture")
        return pd.DataFrame()

    results_df = pd.DataFrame(all_results)
    # For each button, rank byte columns by mean-abs-difference
    report_lines.append(f"\n=== Button-byte candidates [{device}] ===")
    byte_cols = [c for c in results_df.columns if c.startswith("hid_b")]
    for _, row in results_df.iterrows():
        name = row["button"]
        bit  = row["bit"]
        diffs = row[byte_cols].sort_values(ascending=False)
        top5  = diffs.head(5)
        report_lines.append(f"\n  {name} ({bit}):")
        for col, val in top5.items():
            byte_idx = int(col[5:])
            report_lines.append(f"    {col} (byte {byte_idx}): mean |pressed-released| = {val:.2f}")

    # Plot heatmap: buttons × top 20 byte positions
    top_bytes = results_df.set_index("button")[byte_cols].max(axis=0).nlargest(20).index.tolist()
    plot_data = results_df.set_index("button")[top_bytes]
    fig, ax = plt.subplots(figsize=(14, max(3, len(all_results) + 1)))
    im = ax.imshow(plot_data.values, aspect="auto", cmap="hot")
    ax.set_xticks(range(len(top_bytes)))
    ax.set_xticklabels([f"b{int(c[5:]):02d}" for c in top_bytes], rotation=90, fontsize=8)
    ax.set_yticks(range(len(plot_data)))
    ax.set_yticklabels(plot_data.index, fontsize=9)
    ax.set_title(f"Button-byte correlation [{device}] — higher = more correlated")
    plt.colorbar(im, ax=ax, label="|mean(pressed) - mean(released)|")
    plt.tight_layout()
    out_path = os.path.join(out_dir, f"button_bytes_{device}.png")
    plt.savefig(out_path, dpi=120)
    plt.close()
    print(f"  Saved: {out_path}")

    return results_df


def plot_axis_alignment(aligned: pd.DataFrame, device: str, out_dir: str):
    """3×3 grid of scatter plots: each client_api axis vs each hid axis."""
    fig, axes = plt.subplots(3, 3, figsize=(12, 12))
    for row, ca_ax in enumerate(POSITION_COLS):
        for col, hid_ax in enumerate(POSITION_COLS):
            ca_col  = f"{ca_ax}_ca"
            hid_col = f"{hid_ax}_hid"
            ax = axes[row][col]
            if ca_col in aligned.columns and hid_col in aligned.columns:
                ax.scatter(aligned[ca_col], aligned[hid_col], s=2, alpha=0.3)
                corr = np.corrcoef(aligned[ca_col].values, aligned[hid_col].values)[0, 1]
                ax.set_title(f"ca:{ca_ax} vs hid:{hid_ax}\nr={corr:.3f}", fontsize=8)
            ax.set_xlabel(f"ca {ca_ax}", fontsize=7)
            ax.set_ylabel(f"hid {hid_ax}", fontsize=7)
    fig.suptitle(f"Axis alignment scatter [{device}]", fontsize=12)
    plt.tight_layout()
    out_path = os.path.join(out_dir, f"axis_scatter_{device}.png")
    plt.savefig(out_path, dpi=100)
    plt.close()
    print(f"  Saved: {out_path}")


def plot_time_series(aligned: pd.DataFrame, device: str, out_dir: str):
    """Time series of each axis from both sources, for visual alignment check."""
    if aligned.empty:
        return
    t = aligned["timestamp_ms"].values
    fig, axes = plt.subplots(3, 1, figsize=(14, 8), sharex=True)
    for idx, ax_name in enumerate(POSITION_COLS):
        ca_col  = f"{ax_name}_ca"
        hid_x   = f"pos_x_hid"
        hid_y   = f"pos_y_hid"
        hid_z   = f"pos_z_hid"
        ax = axes[idx]
        if ca_col in aligned.columns:
            ax.plot(t, aligned[ca_col].values, label=f"ca {ax_name}", linewidth=0.8)
        for hid_col in [hid_x, hid_y, hid_z]:
            if hid_col in aligned.columns:
                ax.plot(t, aligned[hid_col].values, label=f"hid {hid_col[4:9]}", alpha=0.6, linewidth=0.6)
        ax.set_ylabel(ax_name)
        ax.legend(fontsize=7)
        ax.grid(True, alpha=0.3)
    axes[-1].set_xlabel("timestamp_ms")
    fig.suptitle(f"Position time series [{device}]", fontsize=12)
    plt.tight_layout()
    out_path = os.path.join(out_dir, f"timeseries_{device}.png")
    plt.savefig(out_path, dpi=100)
    plt.close()
    print(f"  Saved: {out_path}")


def main():
    parser = argparse.ArgumentParser(description="Analyze NoloClientAPI vs hidAPI comparison CSV")
    parser.add_argument("csv", help="Path to compare_output.csv")
    parser.add_argument("--out-dir", default="", help="Directory for PNG outputs (default: same dir as CSV)")
    args = parser.parse_args()

    if not os.path.exists(args.csv):
        print(f"error: file not found: {args.csv}")
        sys.exit(1)

    out_dir = args.out_dir or os.path.dirname(os.path.abspath(args.csv))
    os.makedirs(out_dir, exist_ok=True)

    print(f"Loading {args.csv} ...")
    df = load_csv(args.csv)
    print(f"  {len(df)} rows loaded  ({df['source_api'].value_counts().to_dict()})")

    capi, hid = split_sources(df)
    print(f"  client_api rows: {len(capi)}   hid rows: {len(hid)}")

    if capi.empty:
        print("WARNING: no client_api rows found — was --client-api server running?")
    if hid.empty:
        print("WARNING: no hid rows found — was hidAPI server running?")

    report_lines = [f"NoloVR API Comparison Report", f"Source: {args.csv}", f"Rows: {len(df)} (capi={len(capi)}, hid={len(hid)})"]

    devices = ["left", "right"]

    # ── Axis alignment ──────────────────────────────────────────────────────
    print("\n--- Axis alignment ---")
    report_lines.append("\n=== Axis alignment ===")
    for device in devices:
        ca_dev  = capi[capi["device"] == device]
        hid_dev = hid[hid["device"] == device]
        if ca_dev.empty or hid_dev.empty:
            print(f"  [{device}] not enough data (ca={len(ca_dev)}, hid={len(hid_dev)})")
            continue
        aligned = align_nearest(ca_dev, hid_dev)
        if len(aligned) < 10:
            print(f"  [{device}] too few aligned pairs ({len(aligned)})")
            continue
        print(f"  [{device}] {len(aligned)} aligned pairs")
        corr_df = axis_correlation_table(aligned)
        report_lines.append(f"\n[{device}] axis correlation table:")
        report_lines.append(corr_df.to_string(index=False))
        print(corr_df.to_string(index=False))
        # Suggest best mapping
        report_lines.append(f"\n[{device}] best axis mapping (highest |corr|):")
        print(f"\n  [{device}] best axis mapping:")
        for ca_ax in POSITION_COLS:
            sub = corr_df[corr_df["ca_axis"] == ca_ax].sort_values("best", key=abs, ascending=False)
            if not sub.empty:
                best = sub.iloc[0]
                line = f"    ca:{ca_ax} -> hid:{best['hid_axis']} {best['best_sign']}  (r={best['best']:.4f})"
                print(line)
                report_lines.append(line)
        plot_axis_alignment(aligned, device, out_dir)
        plot_time_series(aligned, device, out_dir)

    # ── Button-byte correlation ─────────────────────────────────────────────
    print("\n--- Button-byte analysis ---")
    for device in devices:
        button_byte_candidates(capi, hid, device, out_dir, report_lines)

    # ── Save report ──────────────────────────────────────────────────────────
    report_path = os.path.join(out_dir, "analysis_report.txt")
    with open(report_path, "w") as f:
        f.write("\n".join(report_lines))
    print(f"\nReport saved: {report_path}")


if __name__ == "__main__":
    main()
