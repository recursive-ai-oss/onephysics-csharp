#!/bin/bash
# stab.sh NAME [ENV=VAL ...] : stability run of all 3D scenes for ours (Release bin in ~/onephysics/u71/bin-$BIN), 1 thread
HERE=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
export PATH=~/dotnet10:$PATH DOTNET_ROOT=~/dotnet10
cd ~/onephysics/u71
name=$1; shift
env "$@" nice -n 10 taskset -c ${CPUS:-2-5} dotnet bin-${BIN:-new}/Bar.dll run --mode stability --scene-dir scenes --state $HOME/onephysics/u71 --scenes ${SCENES:-pyramid-3d-10,pyramid-3d-30,large-pile-3d,joint-chains-3d,joint-grid-3d,ragdoll-pile-3d,mesh-terrain-3d,heightfield-terrain-3d,sleep-wake-3d,ragdoll-powered-3d,vehicles-3d,characters-3d,buoyancy-3d,tracked-3d,motorcycle-3d} --engines onephysics --threads 1 --out runs/$name > runs/$name.log 2>&1
python3 $HERE/judge3d.py runs/$name/metrics.csv ${BASE:+runs/$BASE/metrics.csv} > runs/$name.judge
