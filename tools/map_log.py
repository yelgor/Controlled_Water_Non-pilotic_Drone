#!/usr/bin/env python3
"""Карта заїзду з логу: трек на супутниковому знімку, колір = швидкість.

    python3 tools/map_log.py log000.csv            → log000_map.html (відкрити в браузері)

Потрібні колонки lat, lon (прошивка пише їх з листопада 2026). Тільки стандартна
бібліотека Python. Для підкладки карти браузеру потрібен інтернет.
"""
import csv
import html
import json
import math
import sys
from pathlib import Path


def speed_color(v: float, vmax: float) -> str:
    """0 → синій, середня → зелений/жовтий, максимум → червоний."""
    x = 0.0 if vmax <= 0 else max(0.0, min(1.0, v / vmax))
    hue = 240 * (1 - x)  # 240 = синій, 0 = червоний
    return f"hsl({hue:.0f},90%,50%)"


def main() -> int:
    if len(sys.argv) != 2:
        print(__doc__)
        return 1
    src = Path(sys.argv[1])
    rows = list(csv.DictReader(src.open(encoding="utf-8")))
    if not rows or "lat" not in rows[0]:
        print("У лозі немає колонок lat/lon — його записала стара прошивка. Перепроший і зроби новий заїзд.")
        return 1

    pts = []
    t0 = float(rows[0]["t_ms"])
    for r in rows:
        try:
            lat, lon = float(r["lat"]), float(r["lon"])
        except ValueError:
            continue
        if r.get("gps_ok") != "1" or (lat == 0 and lon == 0):
            continue
        pts.append({
            "lat": lat, "lon": lon,
            "t": (float(r["t_ms"]) - t0) / 1000,
            "v": float(r["spd"]), "thr": float(r["throttle"]),
            "rud": float(r["rudder"]), "hdg": float(r["hdg"]), "mode": r["mode"],
        })
    if len(pts) < 2:
        print("Замало точок з GPS-фіксом (gps_ok=1). Дочекайся фіксу перед заїздом.")
        return 1

    vmax = max(p["v"] for p in pts)
    segs = [
        {"a": [a["lat"], a["lon"]], "b": [b["lat"], b["lon"]], "c": speed_color((a["v"] + b["v"]) / 2, vmax)}
        for a, b in zip(pts, pts[1:])
    ]
    # Загальна довжина треку, м (плоске наближення — для коротких відстаней точно).
    dist = 0.0
    for a, b in zip(pts, pts[1:]):
        dn = math.radians(b["lat"] - a["lat"]) * 6371000
        de = math.radians(b["lon"] - a["lon"]) * 6371000 * math.cos(math.radians(a["lat"]))
        dist += math.hypot(dn, de)
    dur = pts[-1]["t"] - pts[0]["t"]
    title = f"Заїзд {src.stem}"
    summary = (f"{len(pts)} точок · {dur / 60:.1f} хв · {dist:.0f} м · "
               f"макс {vmax:.1f} м/с ({vmax * 3.6:.1f} км/год)")

    out = src.with_name(src.stem + "_map.html")
    out.write_text(PAGE.replace("__TITLE__", html.escape(title))
                       .replace("__SUMMARY__", html.escape(summary))
                       .replace("__VMAX__", f"{vmax:.1f}")
                       .replace("__PTS__", json.dumps(pts))
                       .replace("__SEGS__", json.dumps(segs)), encoding="utf-8")
    print(f"Збережено {out}  ({summary})")
    return 0


PAGE = """<!doctype html>
<html lang="uk"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>__TITLE__</title>
<link rel="stylesheet" href="https://unpkg.com/leaflet@1.9.4/dist/leaflet.css">
<script src="https://unpkg.com/leaflet@1.9.4/dist/leaflet.js"></script>
<style>
  html,body{margin:0;height:100%;font:14px system-ui,sans-serif}
  #map{position:absolute;inset:0}
  .box{position:absolute;z-index:1000;left:12px;top:12px;background:#fff;padding:8px 12px;
       border-radius:8px;box-shadow:0 1px 4px #0004;max-width:calc(100% - 48px)}
  .bar{height:10px;border-radius:5px;margin:6px 0 2px;
       background:linear-gradient(90deg,hsl(240,90%,50%),hsl(120,90%,50%),hsl(60,90%,50%),hsl(0,90%,50%))}
  .lg{display:flex;justify-content:space-between;font-size:12px;color:#555}
</style></head><body>
<div id="map"></div>
<div class="box"><b>__TITLE__</b><br>__SUMMARY__
  <div class="bar"></div><div class="lg"><span>0 м/с</span><span>швидкість</span><span>__VMAX__ м/с</span></div>
  <span style="font-size:12px;color:#555">🟢 старт · 🔴 фініш · клік по треку — деталі</span></div>
<script>
const pts = __PTS__, segs = __SEGS__;
const sat = L.tileLayer('https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/{z}/{y}/{x}',
  {maxZoom: 20, attribution: 'Esri World Imagery'});
const osm = L.tileLayer('https://tile.openstreetmap.org/{z}/{x}/{y}.png',
  {maxZoom: 19, attribution: '&copy; OpenStreetMap'});
const map = L.map('map', {layers: [sat]});
L.control.layers({'Супутник': sat, 'Карта': osm}).addTo(map);
for (const s of segs) L.polyline([s.a, s.b], {color: s.c, weight: 5, opacity: .9}).addTo(map);
const all = L.polyline(pts.map(p => [p.lat, p.lon]), {opacity: 0, weight: 18}).addTo(map);
all.on('click', e => {
  let best = pts[0], bd = Infinity;
  for (const p of pts) { const d = map.distance(e.latlng, [p.lat, p.lon]); if (d < bd) { bd = d; best = p; } }
  L.popup().setLatLng([best.lat, best.lon]).setContent(
    `<b>${best.t.toFixed(1)} с</b> · ${best.mode}<br>швидкість ${best.v.toFixed(2)} м/с<br>` +
    `газ ${(best.thr*100).toFixed(0)}% · кермо ${best.rud.toFixed(2)}<br>курс ${best.hdg.toFixed(0)}°`).openOn(map);
});
const a = pts[0], b = pts[pts.length - 1];
L.circleMarker([a.lat, a.lon], {radius: 8, color: '#fff', fillColor: '#1a7f37', fillOpacity: 1}).addTo(map).bindTooltip('старт');
L.circleMarker([b.lat, b.lon], {radius: 8, color: '#fff', fillColor: '#cf222e', fillOpacity: 1}).addTo(map).bindTooltip('фініш');
map.fitBounds(all.getBounds(), {padding: [40, 40]});
</script></body></html>
"""

if __name__ == "__main__":
    sys.exit(main())
