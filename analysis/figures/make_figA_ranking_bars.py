import numpy as np, pandas as pd
import matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import Patch

df = pd.read_csv("data/all_results_main.csv")
ORDER = ['pdg','ctc_pd','ctc_smc','pbc']
COL   = {'pdg':'#3b82f6','ctc_pd':'#ef4444','ctc_smc':'#a855f7','pbc':'#f59e0b'}
LAB   = {'pdg':'PD+g','ctc_pd':'CTC-PD','ctc_smc':'CTC-SMC','pbc':'PBC'}
UNC   = ['light','nominal','heavy']
TIER  = ['proxy','rigid_body']; TLAB={'proxy':'Proxy tier','rigid_body':'Rigid-body tier'}

def cell(tier,unc):
    d = df[(df.fidelity==tier)&(df.params==unc)]
    g = d.groupby('controller').RMSE_cart_norm
    m = g.mean(); se = g.std(ddof=1)/np.sqrt(g.count())
    return m.reindex(ORDER), se.reindex(ORDER)

# print numbers for verification
print("=== Figure A underlying numbers (mean +/- 1.96*SE; n=240/cell) ===")
for tier in TIER:
    for unc in UNC:
        m,se=cell(tier,unc)
        rank={c:int(r) for c,r in m.rank().items()}
        print(f"{tier:11s} {unc:7s}: "+"  ".join(f"{LAB[c]}={m[c]:.4f}(#{rank[c]})" for c in m.sort_values().index))

fig,ax=plt.subplots(2,3,figsize=(7.2,4.8),sharex=True)
rowmax={'proxy':0.40,'rigid_body':0.255}
x=np.arange(4)
for r,tier in enumerate(TIER):
    for c_i,unc in enumerate(UNC):
        a=ax[r,c_i]; m,se=cell(tier,unc); ci=1.96*se
        ranks=m.rank(); ymax=rowmax[tier]
        # A cell is flagged as divergent when its runs saturate torque or sit on the
        # velocity limit for most of the motion (see tables/tab_stability.tex).
        g=df[(df.fidelity==tier)&(df.params==unc)].groupby('controller')
        div=(g.Sat_frac_any.mean()>0.5)|(g.Vel_violation_frac.mean()>0.9)
        over={c: m[c]+ci[c] >= 0.93*ymax for c in ORDER}
        bars=a.bar(x,[min(m[c],ymax) for c in ORDER],yerr=[0 if over[c] else ci[c] for c in ORDER],
                   color=[COL[c] for c in ORDER],edgecolor='#333333',linewidth=0.7,
                   width=0.72,capsize=2.5,error_kw=dict(lw=0.9, ecolor='#333333'))
        for i,c in enumerate(ORDER):
            if div[c]: bars[i].set_hatch('////')
            if over[c]:   # bar runs off the panel: print rank and value inside it
                a.text(i,0.96*ymax,f"{int(ranks[c])}\n{m[c]:.2f}",ha='center',va='top',
                       fontsize=7.5,fontweight='bold',color='white',
                       bbox=dict(facecolor=COL[c],edgecolor='none',pad=1.0))
            else:
                a.text(i,m[c]+ci[c]+ymax*0.03,f"{int(ranks[c])}",ha='center',va='bottom',
                       fontsize=8,fontweight='bold',color=COL[c])
        a.set_ylim(0,rowmax[tier]); a.set_xticks(x); a.set_xticklabels([])
        a.grid(axis='y',alpha=0.25,lw=0.6); a.set_axisbelow(True)
        a.tick_params(labelsize=8)
        if r==0: a.set_title(unc.capitalize(),fontsize=10,fontweight='bold')
        if c_i==0: a.set_ylabel(f"{TLAB[tier]}\nmean RMSE$_{{cart,norm}}$",fontsize=9)
handles=[Patch(facecolor=COL[c],edgecolor='#333333',lw=0.6,label=LAB[c]) for c in ORDER]
handles.append(Patch(facecolor='white',edgecolor='#333333',lw=0.6,hatch='////',label='joints diverge'))
fig.legend(handles=handles,loc='lower center',ncol=5,fontsize=9,frameon=False,bbox_to_anchor=(0.5,-0.02))
fig.suptitle("Four-controller ranking by tier and uncertainty\nrank 1–4 annotated above each bar",
             fontsize=10.5,y=1.0)
fig.tight_layout(rect=[0,0.03,1,0.96])
out="figures/figA_ranking_bars.png"; fig.savefig(out,dpi=300,bbox_inches='tight')
print("SAVED",out)
print("\nsaturation any panel:", df[(df.params.isin(UNC))].Sat_frac_any.max())
