"""Mismatch figure: RMSE_cart_norm against link-mass mismatch, with 95% CIs.

    python analysis/figures/make_mismatch_robustness.py
"""
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

T5 = 2.571  # t-critical, 5 df, two-sided 95%, as in the mismatch table (tab:mismatch)

ORDER = ["pdg", "ctc_pd", "ctc_smc", "pbc"]
COL = {"pdg": "#3b82f6", "ctc_pd": "#ef4444", "ctc_smc": "#a855f7", "pbc": "#f59e0b"}
LAB = {"pdg": "PD+g", "ctc_pd": "CTC-PD", "ctc_smc": "CTC-SMC", "pbc": "PBC"}

ALL = pd.read_csv("data/d1_mismatch_all.csv")


def draw(df, out, title_extra=""):
    """One figure: mean RMSE vs mismatch magnitude, t5 band over the six realizations."""
    pcts = sorted(df.pct.unique())

    # realization mean per (pct, controller) = mean over (robot, trajectory, params, seed);
    # then mean +/- t5*sd/sqrt(6) across the 6 realization means, as in tab_mismatch.
    means = {c: [] for c in ORDER}
    los = {c: [] for c in ORDER}
    his = {c: [] for c in ORDER}
    for pct in pcts:
        d = df[df.pct == pct]
        rm = d.groupby(["realization", "controller"]).RMSE_cart_norm.mean().unstack()[ORDER]
        n = len(rm)
        mu = rm.mean()
        ci = T5 * rm.std(ddof=1) / np.sqrt(n)
        for c in ORDER:
            means[c].append(mu[c])
            los[c].append(mu[c] - ci[c])
            his[c].append(mu[c] + ci[c])

    x = [p * 100 for p in pcts]

    fig, ax = plt.subplots(figsize=(5.2, 4.0))
    for c in ORDER:
        ax.plot(x, means[c], "-o", color=COL[c], label=LAB[c], linewidth=1.8, markersize=5)
        ax.fill_between(x, los[c], his[c], color=COL[c], alpha=0.18, linewidth=0)

    ax.set_xlabel("Link-mass mismatch magnitude (%)")
    ax.set_ylabel(r"Mean RMSE$_{cart,norm}$")
    ax.set_xticks(x)
    ax.grid(axis="y", alpha=0.25, lw=0.6)
    ax.set_axisbelow(True)
    ax.legend(frameon=False, fontsize=9)
    fig.suptitle("Controller robustness to parametric mismatch" + title_extra + "\n(shaded band: 95% CI, "
                     r"$t_5$ over 6 realizations)", fontsize=10.5)
    fig.tight_layout()

    fig.savefig(out, dpi=300, bbox_inches="tight")
    print("SAVED", out)

    for i, pct in enumerate(pcts):
        row = "  ".join(f"{LAB[c]}={means[c][i]:.4f}[{los[c][i]:.4f},{his[c][i]:.4f}]" for c in ORDER)
        print(f"+/-{int(pct*100)}%: {row}")
    plt.close(fig)


# All three uncertainty levels pooled, and the nominal + heavy levels only (at
# the light level CTC-PD and CTC-SMC diverge on the URDF robots).
draw(ALL, "figures/figC_mismatch_robustness.png")
draw(ALL[ALL.params.isin(["nominal", "heavy"])], "figures/figC_mismatch_robustness_stable.png",
     title_extra=", nominal and heavy levels")
