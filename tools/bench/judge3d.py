#!/usr/bin/env python3
# usage: judge.py metrics.csv [baseline metrics.csv]  -> per scene: PASS/FAIL with violated metrics, and the key numbers
import sys,csv,collections
L={
"pyramid-3d-10":{"maxSpeed":0.1,"maxPenetration":0.01,"jitter":0.01,"restKE":0.0001,"finalPenetration":0.005,"topDrop":0.06,"topLateral":0.05},
"pyramid-3d-30":{"maxSpeed":1,"maxPenetration":0.05,"jitter":0.01,"restKE":0.0001,"finalPenetration":0.005,"topDrop":0.1,"topLateral":0.25},
"large-pile-3d":{"maxSpeed":15,"maxPenetration":0.5,"jitter":0.01,"restKE":0.0001,"finalPenetration":0.05},
"joint-chains-3d":{"maxSpeed":45,"jointError":1.0},
"joint-grid-3d":{"maxSpeed":5,"jointError":0.1},
"ragdoll-pile-3d":{"maxSpeed":30,"maxPenetration":0.4,"jointError":0.3,"restKE":0.05,"finalPenetration":0.05},
"mesh-terrain-3d":{"maxSpeed":15,"maxPenetration":0.15,"finalPenetration":0.03},
"heightfield-terrain-3d":{"maxSpeed":15,"maxPenetration":0.15,"finalPenetration":0.05},
"sleep-wake-3d":{"maxSpeed":45,"maxPenetration":1.5,"finalPenetration":0.3},
"queries-3d":{"maxSpeed":25},
"cloth-drape-3d":{"softMaxSpeed":10,"softPenetration":0.07,"softFinalPenetration":0.015,"softRestKE":0.0007,"edgeStrain":0.15},
"soft-pile-3d":{"softMaxSpeed":10,"softPenetration":0.02,"softFinalPenetration":0.01,"softRestKE":0.02,"volumeError":0.05},
"soft-rigid-3d":{"softMaxSpeed":10,"softPenetration":0.07,"softFinalPenetration":0.01,"softRestKE":0.001,"volumeError":0.03,"maxSpeed":10,"restKE":0.1},
"soft-mesh-3d":{"softMaxSpeed":10,"softPenetration":0.02,"softFinalPenetration":0.01,"softRestKE":0.01,"volumeError":0.03},
"ragdoll-powered-3d":{"maxSpeed":10,"poseErrorP95":0.10,"fallen":0,"jointError":0.05},
"vehicles-3d":{"maxSpeed":25,"maxPenetration":0.05,"rollovers":0,"wheelThroughGround":0,"suspensionJitter":0.01},
"characters-3d":{"maxSpeed":25,"maxPenetration":0.05,"charJitter":0.01,"slopeViolations":0},
"buoyancy-3d":{"maxSpeed":8,"equilibriumDepthError":0.02,"sinkersAfloat":0},
"tracked-3d":{"maxSpeed":8,"maxPenetration":0.05,"suspensionJitter":0.01,"rollovers":0},
"motorcycle-3d":{"maxSpeed":12,"maxPenetration":0.05,"suspensionJitter":0.01,"fallen":0,"leanError":0.05},
}
ZERO=["nan","escapes","tunnelled","wheelThroughGround","softNan","softEscapes","softTunnelled"]
def load(f):
    m=collections.defaultdict(dict)
    for r in csv.DictReader(open(f)):
        e=r.get('engine','onephysics'); sc=r['scene'] if e=='onephysics' else r['scene']+'@'+e
        m[sc][r['metric']]=float(r['value'])
    return m
cur=load(sys.argv[1]); base=load(sys.argv[2]) if len(sys.argv)>2 else {}
for key0 in sorted(cur):
    sc=key0.split('@')[0]
    if sc not in L: continue
    bad=[]
    for k,v in cur[key0].items():
        if k in ZERO and v!=0 and not (sc=="sleep-wake-3d" and k=="escapes" and v<=10): bad.append(f"{k}={v:g}")
        elif k in L[sc] and v>L[sc][k]: bad.append(f"{k}={v:g}>{L[sc][k]:g}")
        if k=="stairSuccess" and v<1: bad.append(f"stairSuccess={v:g}")
    key=[k for k in L[sc] if k in cur[key0]]
    nums=" ".join(f"{k}={cur[key0][k]:.4g}"+(f"({base[sc][k]:.4g})" if sc in base and k in base[sc] else "") for k in key)
    print(("PASS " if not bad else "FAIL ")+key0.ljust(24)+nums+("   !! "+", ".join(bad) if bad else ""))
