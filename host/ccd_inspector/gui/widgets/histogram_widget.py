"""ADC value histogram widget."""

from __future__ import annotations

import numpy as np
import pyqtgraph as pg

from ccd_inspector.comm.protocol import ADC_MAX, EFFECTIVE_END, EFFECTIVE_START


class HistogramWidget(pg.PlotWidget):
    """Histogram of ADC values for the effective pixel region."""

    BINS = 128

    def __init__(self, parent=None):
        super().__init__(parent)

        self.setBackground('#1e1e1e')
        self.setLabel('bottom', 'ADC Value')
        self.setLabel('left', 'Count')
        self.setXRange(0, ADC_MAX, padding=0.02)
        self.showGrid(x=True, y=True, alpha=0.15)

        self._bar = pg.BarGraphItem(x=[], height=[], width=ADC_MAX / self.BINS,
                                     brush='#00aa66')
        self.addItem(self._bar)

    def update_frame(self, pixels: np.ndarray):
        eff = pixels[EFFECTIVE_START:EFFECTIVE_END]
        counts, edges = np.histogram(eff, bins=self.BINS, range=(0, ADC_MAX))
        centers = (edges[:-1] + edges[1:]) / 2
        self._bar.setOpts(x=centers, height=counts, width=edges[1] - edges[0])
        self.setYRange(0, max(int(counts.max() * 1.1), 1))
