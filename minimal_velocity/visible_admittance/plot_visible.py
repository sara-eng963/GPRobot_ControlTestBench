"""Render response figures from the visible-admittance batch, without simulation."""
import csv
from html import escape
from pathlib import Path
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from scipy.io import loadmat

JOINTS = [f"J{i}" for i in range(1, 7)]


def response(folder, source, case):
    run = loadmat(source / f"{case}.mat", simplify_cells=True)
    s, fixture = run["s"], run["a"]
    fig, axes = plt.subplots(5, 1, figsize=(10, 11), sharex=True, layout="constrained")
    fig.suptitle(f'{case.replace("_", " ")} | R2024b measured simulation response')
    for j, labels in enumerate((["Fx", "Fy", "Fz"], ["Mx", "My", "Mz"])):
        cols = slice(3*j, 3*j+3)
        ax = axes[j]
        applied = s["applied_wrench"]
        filtered = s["ema_wrench"]
        ax.step(applied["t"], applied["x"][:, cols], where="post", label=labels, lw=1.6)
        ax.set_prop_cycle(None)
        ax.plot(filtered["t"], filtered["x"][:, cols], "--", lw=.85)
        ax.set_title("Physical TCP wrench (solid) and EMA measurement (dashed)")
        ax.set_ylabel(["N", "N·m"][j])
        ax.legend(loc="upper right", ncol=3, fontsize=8)
    ax = axes[2]
    qd = s["qd"]
    command = s["selected_qd"]
    ax.plot(qd["t"], qd["x"], label=JOINTS, lw=1.45)
    ax.set_prop_cycle(None)
    ax.step(command["t"], command["x"], "--", where="post", lw=.85)
    ax.set_title("Joint velocity | measured (solid) and selected command (dashed)")
    ax.set_ylabel("rad/s")
    ax.legend(loc="upper right", ncol=3, fontsize=8)
    ax = axes[3]
    q = s["q"]
    ax.plot(q["t"], np.rad2deg(q["x"] - q["x"][0]))
    ax.set_title("Measured joint displacement from initial pose")
    ax.set_ylabel("deg")
    ax = axes[4]
    tcp = s["tcp"]
    ax.plot(tcp["t"], 1000 * (tcp["x"] - tcp["x"][0]), label=["X", "Y", "Z"])
    ax.set_title("Measured TCP displacement in base axes")
    ax.set_ylabel("mm")
    ax.legend(loc="upper right", ncol=3, fontsize=8)
    ax.set_xlabel("Simulation time (s)")
    for ax in axes:
        if np.any(fixture["w"]):
            ax.axvspan(fixture["start"], fixture["release"], color="0.93", zorder=0)
        ax.grid(alpha=.22)
    fig.savefig(folder / f"{case}_response.png", dpi=125)
    plt.close(fig)


def limits(folder, source, case, cfg):
    raw = loadmat(source / f"{case}.mat", simplify_cells=True)["raw"]
    t, v = raw["t"], raw["twist"]
    cap = cfg["demo"]["admittanceSpeedLimit"]
    delta_cap = cfg["demo"]["admittanceAccelerationLimit"]
    accel = np.diff(v, axis=0, prepend=np.zeros((1, 6))) / cfg["sampleTime_s"]

    fig, axes = plt.subplots(2, 2, figsize=(11, 6), sharex=True, layout="constrained")
    fig.suptitle(f'{case.replace("_", " ")} | visible blocks vs original native C')
    for col, names in enumerate((["X", "Y", "Z"], ["about X", "about Y", "about Z"])):
        sl = slice(3*col, 3*col+3)
        ax = axes[0, col]
        ax.plot(t, v[:, sl], label=names, lw=1.5)
        ax.set_prop_cycle(None)
        ax.plot(t[::8], raw["oracle"][::8, sl], "o", ms=2)
        ax.hlines([-cap[3*col], cap[3*col]], t[0], t[-1], color=".45", ls=":")
        ax.set_title(["Linear velocity", "Angular velocity"][col]+" | block lines and C dots")
        ax.set_ylabel(["m/s", "rad/s"][col])
        ax.legend(ncol=3, fontsize=8)
        ax = axes[1, col]
        ax.plot(t, accel[:, sl], lw=1.3)
        ax.hlines([-delta_cap[3*col], delta_cap[3*col]], t[0], t[-1], color=".45", ls=":")
        ax.set_title("Applied per-tick change divided by the 2 ms period")
        ax.set_ylabel(["m/s²", "rad/s²"][col])
        ax.set_xlabel("Test time (s)")
    for ax in axes.flat:
        ax.axvspan(.10, .54, color=".94", zorder=0)
        ax.grid(alpha=.22)
    fig.savefig(folder / f"{case}_response.png", dpi=125)
    plt.close(fig)


def summary(folder):
    with (folder / "summary.csv").open(encoding="utf-8-sig", newline="") as f:
        return list(csv.DictReader(f))


def main(folder, raw_source=None, isolated_source=None):
    config = loadmat(folder / "configuration.mat", simplify_cells=True)["cfg"]
    raw_source = raw_source or folder
    sources = []
    if isolated_source:
        sources.extend((r, isolated_source) for r in summary(isolated_source))
    sources.extend((r, raw_source) for r in summary(folder))
    with (folder / "qualified_summary.csv").open("w", encoding="utf-8", newline="") as f:
        columns = ("name", "controller", "plant", "evidence", "raw_folder")
        writer = csv.DictWriter(f, fieldnames=columns)
        writer.writeheader()
        for row, source in sources:
            writer.writerow({**row, "raw_folder": source.name})
    for row, source in sources:
        case = row["name"]
        if not (source / f"{case}.mat").is_file():
            continue
        if case in ("isolated_positive_limits", "isolated_negative_limits"):
            limits(folder, source, case, config)
        elif case in ("zero_wrench", "Fx_positive", "Mz_negative",
                      "push_sustain_release", "standalone_selector"):
            response(folder, source, case)
    body = []
    for row, source in sources:
        case = row["name"]
        links = []
        for label, path in (("Response", folder / f"{case}_response.png"),
                            ("MAT", source / f"{case}.mat"),
                            ("Rescored MAT", folder / f"{case}_rescore.mat"),
                            ("Exception", source / f"{case}_exception.mat")):
            if path.is_file():
                file = path.name if path.parent == folder else f"../{path.parent.name}/{path.name}"
                links.append(f'<a href="{file}">{label}</a>')
        cells = [escape(row[key]) for key in ("name", "controller", "plant", "evidence")]
        cells.append(" · ".join(links))
        body.append("<tr>" + "".join(f"<td>{entry}</td>" for entry in cells) + "</tr>")
    history = ""
    if raw_source != folder:
        history = (f'<p>Live raw evidence retained in the <a href="../{raw_source.name}/summary.csv">'
                   'initial batch</a>. Live controller results shown here are rescored on the mapper clock. '
                   'Isolated cases use the corrected input fixture.</p>')
    (folder / "index.html").write_text("""<!doctype html><meta charset="utf-8">
<title>Visible admittance validation</title>
<style>body{font:16px system-ui;margin:2rem;max-width:92rem}table{border-collapse:collapse}
th,td{text-align:left;vertical-align:top;padding:.5em .7em;border-bottom:1px solid #cbd5e1}
td:last-child{white-space:nowrap}a{color:#075985}</style>
<h1>Visible admittance validation</h1>
<p>MATLAB R2024b. Isolated limiter trials were disconnected from the robot.
Measured plant data are from the torque-driven simulated robot.</p>
<p><a href="qualified_summary.csv">All cases CSV</a> · <a href="PREREGISTRATION.txt">Acceptance criteria</a>
 · <a href="configuration.mat">Configuration</a></p>
""" + history + """<table><thead><tr><th>Case</th><th>Controller</th><th>Plant</th><th>Evidence</th><th>Files</th>
</tr></thead><tbody>""" + "".join(body) + "</tbody></table>", encoding="utf-8")


if __name__ == "__main__":
    main(*(Path(arg).resolve() for arg in sys.argv[1:]))
