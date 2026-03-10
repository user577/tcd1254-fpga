"""Fast real-time CCD waveform display using pyqtgraph."""

from __future__ import annotations

import numpy as np
import pyqtgraph as pg
from PySide6.QtCore import Qt

from ccd_inspector.comm.protocol import ADC_MAX, EFFECTIVE_END, EFFECTIVE_START, TOTAL_PIXELS


class WaveformWidget(pg.PlotWidget):
    """Real-time CCD waveform plotter."""

    def __init__(self, parent=None):
        super().__init__(parent)

        self.setBackground('#1e1e1e')
        self.setLabel('bottom', 'Pixel Index')
        self.setLabel('left', 'ADC Value (12-bit)')
        self.setYRange(0, ADC_MAX, padding=0.02)
        self.setXRange(0, TOTAL_PIXELS, padding=0)
        self.showGrid(x=True, y=True, alpha=0.15)

        # Main waveform curve
        self._curve = self.plot(pen=pg.mkPen('#00ff88', width=1))

        # Effective pixel region shading
        self._eff_region = pg.LinearRegionItem(
            values=[EFFECTIVE_START, EFFECTIVE_END],
            movable=False,
            brush=pg.mkBrush(255, 255, 255, 8),
        )
        self._eff_region.setZValue(-10)
        self.addItem(self._eff_region)

        # Edge markers
        self._edge_lines: list[pg.InfiniteLine] = []

        # Peak / mean text
        self._stats_text = pg.TextItem(anchor=(1, 0), color='#aaaaaa')
        self._stats_text.setPos(TOTAL_PIXELS - 10, ADC_MAX - 50)
        self.addItem(self._stats_text)

        # Saturation line
        self._sat_line = pg.InfiniteLine(
            pos=ADC_MAX, angle=0,
            pen=pg.mkPen('#ff4444', width=1, style=Qt.PenStyle.DashLine),
        )
        self.addItem(self._sat_line)

    def update_frame(self, pixels: np.ndarray):
        """Update the waveform with new pixel data."""
        self._curve.setData(pixels)

        eff = pixels[EFFECTIVE_START:EFFECTIVE_END]
        peak = int(np.max(eff))
        mean = float(np.mean(eff))
        self._stats_text.setText(f"Peak: {peak}  Mean: {mean:.0f}")

    def set_edges(self, edges: list[dict]):
        """Show detected edge positions.

        Each edge dict: {'position': float, 'polarity': 'rising'|'falling', 'strength': float}
        """
        # Clear old markers
        for line in self._edge_lines:
            self.removeItem(line)
        self._edge_lines.clear()

        for edge in edges:
            color = '#ff6644' if edge.get('polarity') == 'rising' else '#4488ff'
            line = pg.InfiniteLine(
                pos=edge['position'],
                angle=90,
                pen=pg.mkPen(color, width=2),
                label=f"{edge['position']:.1f}",
                labelOpts={'position': 0.95, 'color': color},
            )
            self.addItem(line)
            self._edge_lines.append(line)

    def clear_edges(self):
        for line in self._edge_lines:
            self.removeItem(line)
        self._edge_lines.clear()
