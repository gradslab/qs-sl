#!/usr/bin/env bash
# Ground-up reproduction: install -> verification gates -> the runs -> numbers -> figures -> page.
# Run from sim/:  bash reproduce.sh      (add "anim" as the first argument to also render the animations)
set -euo pipefail
cd "$(dirname "$0")"

echo "== 1/6 install =="
if [ ! -d .venv ]; then python3 -m venv .venv; fi
source .venv/bin/activate
pip install --quiet -r requirements.txt

echo "== 2/6 verification gates =="
python3 model/test_chains.py
python3 drake/verify_plant.py

echo "== 3/6 the runs =="
python3 model/simulate_port.py
python3 model/simulate_port.py --tight
python3 model/compare_matlab.py
python3 drake/sim_drake.py
python3 drake/sim_drake.py --start on-path --tag _onpath
python3 drake/sim_drake.py --point-mass --tag _pointmass
python3 drake/sim_drake.py --hz 400 --tag _400hz

echo "== 4/6 numbers =="
python3 metrics.py

echo "== 5/6 figures =="
python3 paper_figures.py
python3 paper_figures.py --web
python3 drake/render_drake.py drake_run --snapshots 0,1,1.5,2,5,6.5,8,10.5 --cam wide

if [ "${1:-}" = "anim" ]; then
  echo "== animations =="
  for c in follow side orbit; do python3 drake/render_drake.py drake_run --cam $c; done
  python3 drake/render_drake.py drake_run_onpath --cam follow
  python3 web/anim_web.py
fi

echo "== 6/6 page =="
python3 web/page_numbers.py
python3 web/build_page.py

echo
echo "Done. Numbers: results/web/numbers.json   Figures: results/figs/   Page: ../docs/index.html"
