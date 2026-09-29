import re
import sys
from collections import defaultdict

import matplotlib.pyplot as plt
from matplotlib.ticker import AutoMinorLocator, MaxNLocator
from matplotlib.widgets import MultiCursor

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

# Ein Diagramm pro Bein
fig, achsen = plt.subplots(len(beine), 1, sharex=True, figsize=(14, 4 * len(beine)))
if len(beine) == 1:
    achsen = [achsen]

for ax, leg in zip(achsen, sorted(beine)):
    for name in ("x", "y", "z"):
        werte = beine[leg][name]
        # X-Achse = laufende Nummer des Wertes (1 = erster Wert, 2 = zweiter ...)
        ax.plot(range(1, len(werte) + 1), werte, label=name)

    ax.set_ylabel(f"Bein {leg}\nWinkel (°)")
    ax.legend(loc="upper right")

    # X-Achse: viele beschriftete Ticks, Anzahl der Werte an jeder Stelle sichtbar
    ax.xaxis.set_major_locator(MaxNLocator(nbins=25, integer=True))
    ax.xaxis.set_minor_locator(AutoMinorLocator())
    ax.tick_params(axis="x", labelbottom=True, rotation=45)

    # Y-Achse: feinere Teilung, damit der Winkel genau ablesbar ist
    ax.yaxis.set_major_locator(MaxNLocator(nbins=15))
    ax.yaxis.set_minor_locator(AutoMinorLocator(5))

    ax.grid(which="major", alpha=0.5)
    ax.grid(which="minor", alpha=0.2, linestyle=":")

achsen[-1].set_xlabel("Anzahl Werte (Nummer des Updates)")

# Fadenkreuz ueber alle Diagramme, zeigt Position beim Bewegen der Maus
cursor = MultiCursor(fig.canvas, achsen, color="red", lw=0.8, horizOn=True, vertOn=True)


# Aktueller Wert unter der Maus in der Fensterleiste
def anzeige(x, y):
    return f"Wert Nr. {x:.0f}   Winkel {y:.2f}°"


for ax in achsen:
    ax.format_coord = anzeige

plt.tight_layout()
plt.show()
