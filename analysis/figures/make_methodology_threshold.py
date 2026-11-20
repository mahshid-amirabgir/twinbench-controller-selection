"""Figure 1: the fidelity threshold (a) and purpose-dependent fidelity (b)."""

from __future__ import annotations

import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch, FancyBboxPatch

ROOT = Path(__file__).resolve().parents[2]
for _p in (ROOT, ROOT / "src"):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from analysis.generate_tables import table_coupled, table_fidelity_by_uncertainty

BLUE_FC, BLUE_EC = "#dce6f1", "#3a6ea5"
ORANGE_FC, ORANGE_EC = "#f6e3d3", "#c0651f"
GREY_FC, GREY_EC = "#f0f0f0", "#666666"
RED = "#7b2a2a"
GREEN = "#2e7d4f"
CROSS = "#b03a48"
STYLE = {"font.family": "serif", "font.serif": ["Times New Roman", "Liberation Serif", "TeX Gyre Termes", "DejaVu Serif"],
         "mathtext.fontset": "stix"}


def build_figure(kr6_tau_b, tau_nominal, tau_heavy):
    fig, ax = plt.subplots(figsize=(8.4, 6.0))
    ax.set_xlim(0, 100)
    ax.set_ylim(0, 72)
    ax.axis("off")

    def box(xc, yc, w, h, txt, fc, ec, fs=9, bold=False):
        ax.add_patch(FancyBboxPatch((xc - w / 2, yc - h / 2), w, h,
                                    boxstyle="round,pad=0.3,rounding_size=1.0",
                                    fc=fc, ec=ec, lw=1.4))
        ax.text(xc, yc, txt, ha="center", va="center", fontsize=fs, linespacing=1.3,
                fontweight="bold" if bold else "normal", color="#111111")

    def down(x, y0, y1, color):
        ax.add_patch(FancyArrowPatch((x, y0), (x, y1), arrowstyle="-|>", mutation_scale=11,
                                     lw=1.6, color=color))

    # (a) fidelity threshold
    xa = 27
    box(xa, 64, 31, 6.2, "Datasheet-only proxy\ndecoupled  $I\\ddot{q} + B\\dot{q} + G\\sin q$",
        BLUE_FC, BLUE_EC)
    down(xa, 60.3, 55.8, "#333333")
    ax.text(xa + 1.8, 58.0, "add prescribed-form coupling", fontsize=7.5, style="italic",
            color="#777777", va="center")
    box(xa, 51.5, 31, 6.2, "Coupled proxy\noff-diag. inertia + prescribed Coriolis",
        BLUE_FC, BLUE_EC)
    ax.text(xa, 46.3, f"ranking preserved  $\\tau_b = {kr6_tau_b:.3f}$ (KR6)", ha="center",
            fontsize=8, color=BLUE_EC, fontweight="bold")
    down(xa, 44.3, 40.6, RED)

    ax.plot([7, 47], [38.5, 38.5], ls=(0, (5, 4)), lw=1.6, color=RED, zorder=1)
    ax.add_patch(FancyBboxPatch((14.5, 36.6), 25, 3.8, boxstyle="round,pad=0.2,rounding_size=0.5",
                                fc=RED, ec=RED, zorder=2))
    ax.text(xa, 38.5, "FIDELITY THRESHOLD", ha="center", va="center", fontsize=8.5,
            fontweight="bold", color="white", zorder=3)

    box(xa, 29, 34, 7.6, "URDF + articulated-body dynamics\n"
        "$M(q)\\ddot{q} + C(q,\\dot{q})\\dot{q} + g(q)$\nconfiguration-dependent",
        ORANGE_FC, ORANGE_EC)
    ax.text(xa, 22.4, f"exact model  $\\tau_b = {tau_nominal:.3f}$;   "
            f"heavy mismatch  $\\tau_b = {tau_heavy:.3f}$",
            ha="center", fontsize=8, color=ORANGE_EC, fontweight="bold")
    down(xa, 21.3, 18.2, ORANGE_EC)
    box(xa, 14.5, 31, 4.6, "Reliable controller selection", GREY_FC, GREY_EC, fs=9.5, bold=True)
    ax.text(xa, 5.5, "(a) Prescribed coupling alone does not cross the\nthreshold; under parametric "
            "uncertainty, the full\nconfiguration-dependent dynamics do.",
            ha="center", va="center", fontsize=8.5, style="italic", linespacing=1.3)

    # (b) purpose-dependent fidelity
    xb = 69
    ax.text(xb + 6, 69, "Purpose-dependent fidelity", ha="center", fontsize=10, fontweight="bold")
    rows = [
        (59, "Torque-magnitude\nsanity check", True),
        (46, "Trajectory\nfeasibility", True),
        (33, "Controller ranking,\nexact model", True),
        (20, "Controller ranking\nunder uncertainty", False),
    ]
    for y, txt, ok in rows:
        box(xb, y, 16, 7.2, txt, BLUE_FC if ok else ORANGE_FC, BLUE_EC if ok else ORANGE_EC, fs=8.5)
        ax.text(xb + 11, y, "✓" if ok else "✗", ha="center", va="center", fontsize=16,
                color=GREEN if ok else CROSS, fontfamily="DejaVu Sans")
        ax.text(xb + 14.5, y, "proxy may\nsuffice" if ok else "may require\nconfig-dependent\ndynamics",
                ha="left", va="center", fontsize=8, fontweight="bold", linespacing=1.15,
                color=GREEN if ok else CROSS)
    ax.text(xb + 4, 7.5, "(b) A single fidelity level is not uniformly\nsufficient: the purpose sets "
            "the requirement.", ha="center", va="center", fontsize=8.5, style="italic", linespacing=1.3)
    return fig


def main():
    coupled_computed, _ = table_coupled()
    by_level, _ = table_fidelity_by_uncertainty()

    kr6_tau_b = coupled_computed["tau_by_eps"][0.09]
    tau_nominal = by_level["nominal"]["subset_tau_b"]
    tau_heavy = by_level["heavy"]["subset_tau_b"]

    with plt.rc_context(STYLE):
        fig = build_figure(kr6_tau_b, tau_nominal, tau_heavy)
        for name, kw in (("methodology_threshold.png", {"dpi": 300}),
                         ("methodology_threshold_serif.pdf", {"metadata": {"CreationDate": None}})):
            out = ROOT / "figures" / name
            fig.savefig(out, bbox_inches="tight", **kw)
            print(f"SAVED {out}")
        plt.close(fig)

    print(f"  KR6 coupled-proxy tau_b (eps_c=0.09): {kr6_tau_b:.4f}")
    print(f"  URDF subset tau_b: nominal {tau_nominal:.4f}, heavy {tau_heavy:.4f}")


if __name__ == "__main__":
    main()
