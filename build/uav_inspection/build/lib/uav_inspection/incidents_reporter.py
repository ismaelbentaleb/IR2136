#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import csv
import json

from std_msgs.msg import String


class IncidentsReporter:
    """
    Keeps your CSV/HTML + JSON publishing exactly as before, but out of the node.
    """

    def __init__(self, incidents_dir: str, incident_pub, logger):
        self.incidents_dir = incidents_dir
        self.incident_pub = incident_pub
        self.logger = logger

        os.makedirs(self.incidents_dir, exist_ok=True)

    def publish_incident(self, payload: dict):
        msg = String()
        msg.data = json.dumps(payload)
        self.incident_pub.publish(msg)

    def append_incident_row(self, payload: dict):
        """
        Guarda un CSV y un HTML en incidents_dir con SOLO las incidencias.
        """
        csv_path = os.path.join(self.incidents_dir, "incidents.csv")
        new_file = not os.path.exists(csv_path)

        with open(csv_path, "a", newline="") as f:
            w = csv.writer(f)
            if new_file:
                w.writerow(["timestamp", "photo_file", "score_percent", "local_x", "local_y", "local_z"])
            w.writerow([
                f"{payload['timestamp']:.3f}",
                payload["photo_file"],
                f"{payload['score_percent']:.2f}",
                f"{payload['local']['x']:.3f}",
                f"{payload['local']['y']:.3f}",
                f"{payload['local']['z']:.3f}",
            ])

        self.write_incidents_index_html()

    def write_incidents_index_html(self):
        """
        Regenera incidents/index.html leyendo incidents.csv.
        """
        csv_path = os.path.join(self.incidents_dir, "incidents.csv")
        html_path = os.path.join(self.incidents_dir, "index.html")

        if not os.path.exists(csv_path):
            return

        rows = []
        with open(csv_path, "r", newline="") as f:
            r = csv.reader(f)
            _header = next(r, None)
            for row in r:
                if len(row) >= 6:
                    rows.append(row)

        html = []
        html.append("<!doctype html>")
        html.append("<html><head><meta charset='utf-8'><title>Incidents</title>")
        html.append("<style>")
        html.append("body{font-family:Arial, sans-serif; margin:20px;}")
        html.append("table{border-collapse:collapse; width:100%;}")
        html.append("th,td{border:1px solid #ddd; padding:8px; text-align:left; vertical-align:top;}")
        html.append("th{background:#f4f4f4;}")
        html.append("img{max-width:320px; height:auto; display:block;}")
        html.append("</style></head><body>")
        html.append("<h2>Detected incidents</h2>")
        html.append("<p>Only images where an incident was detected.</p>")
        html.append("<table>")
        html.append("<tr><th>Image</th><th>photo_file</th><th>score%</th><th>timestamp</th><th>local (x,y,z)</th></tr>")

        # row = [timestamp, photo_file, score, x, y, z]
        for ts, photo_file, score, x, y, z in reversed(rows):
            html.append("<tr>")
            html.append(f"<td><a href='{photo_file}'><img src='{photo_file}'></a></td>")
            html.append(f"<td>{photo_file}</td>")
            html.append(f"<td>{score}</td>")
            html.append(f"<td>{ts}</td>")
            html.append(f"<td>({x}, {y}, {z})</td>")
            html.append("</tr>")

        html.append("</table></body></html>")

        with open(html_path, "w") as f:
            f.write("\n".join(html))
