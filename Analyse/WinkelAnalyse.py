import re
import sys
from collections import defaultdict

import matplotlib.pyplot as plt

DATEI = sys.argv[1] if len(sys.argv) > 1 else "Data.txt"

MUSTER = re.compile(
    r"leg=(\d+),\s*phase=([-\d.]+),\s*x=([-\d.]+),\s*y=([-\d.]+),\s*z=([-\d.]+)"
)

# Rohdaten pro Bein einlesen
beine = defaultdict(lambda: {"x": [], "y": [], "z": []})

with open(DATEI, encoding="utf-8") as f:
    for zeile in f:
        m = MUSTER.search(zeile)
        if not m:
            continue
        leg = int(m.group(1))
        beine[leg]["x"].append(float(m.group(3)))
        beine[leg]["y"].append(float(m.group(4)))
        beine[leg]["z"].append(float(m.group(5)))

# Ein Diagramm pro Bein, nur die Rohdaten x, y, z
fig, achsen = plt.subplots(len(beine), 1, sharex=True, figsize=(12, 3 * len(beine)))
if len(beine) == 1:
    achsen = [achsen]

for ax, leg in zip(achsen, sorted(beine)):
    for name in ("x", "y", "z"):
        ax.plot(beine[leg][name], label=name)
    ax.set_ylabel(f"Bein {leg}")
    ax.legend(loc="upper right")

achsen[-1].set_xlabel("Update")
plt.tight_layout()
plt.show()
