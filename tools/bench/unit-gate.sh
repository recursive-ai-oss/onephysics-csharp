#!/usr/bin/env bash
# Runs chosen generated scenes in a non-metre world and judges them in metres.
#   tools/bench/unit-gate.sh OUTDIR UNITS_PER_METRE scene1,scene2,...
# The scene is rescaled by bench/Bar/UnitScaling.cs (BAR_UNITS_PER_METRE), the driver converts its dumps back to metres, and the
# thresholds apply unchanged. Needs BAR (Bar.dll) and SCENES (the generated scene directory) in the environment.
out=$1; k=$2; scenes=$3
mkdir -p "$out/items"
dotnet "$BAR" gate-plan --scene-dir "$SCENES" --threads 1 > "$out/plan_all.tsv" 2>/dev/null
IFS=, read -ra arr <<< "$scenes"
: > "$out/plan.tsv"
for s in "${arr[@]}"; do grep -P "^run-$s\t" "$out/plan_all.tsv" >> "$out/plan.tsv"; done
while IFS=$'\t' read -r id args; do
  item="$out/items/$id"; a="${args//\{ITEM\}/$item}"
  BAR_UNITS_PER_METRE=$k dotnet "$BAR" $a > "$out/items/$id.log" 2>&1; echo $? > "$out/items/$id.rc"
done < "$out/plan.tsv"
dotnet "$BAR" gate-table --in "$out" --scene-dir "$SCENES" --out "$out/stability.tsv" > "$out/table.log" 2>&1
cat "$out/stability.tsv"
