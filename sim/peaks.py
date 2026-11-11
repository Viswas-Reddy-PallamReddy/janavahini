"""Print the peak density per zone for each saved uncontrolled scenario run.

Usage:  python peaks.py
"""

import csv
from pathlib import Path

RES = Path(__file__).resolve().parent / "results"

for name in ["normal", "surge", "blocked"]:
    path = RES / f"{name}_uncontrolled.csv"
    if not path.exists():
        continue
    with open(path) as f:
        rows = list(csv.DictReader(f))
    cols = [c for c in rows[0] if c.startswith("rho_")]
    print(name, {c[4:]: round(max(float(r[c]) for r in rows), 2) for c in cols})
