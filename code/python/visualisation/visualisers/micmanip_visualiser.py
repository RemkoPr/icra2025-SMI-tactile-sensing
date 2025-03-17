import numpy as np
from cyclonedds.util import duration
from loguru import logger
import pyqtgraph as pg
from threading import Thread
from visualisation.visualisers.visualiser import Visualiser
from communication.data_classes.sequence import Sequence
from loguru import logger
import os


class MicManipVisualiser(Visualiser):
    def __init__(self):
        super().__init__(topic_data_type=Sequence, description="Visualise data from a MicManip sensor.")
        self.fs = 16000  # sample rate for input
        self.samples_per_frame = 1000
        self.plotted_values = [0 for _ in range(self.samples_per_frame)]
        self.refresh_rate_hz = self.fs / self.samples_per_frame
        self.timebase = np.arange(self.samples_per_frame) / self.fs
        self.yrange = None  # (-900000, 900000)
        self.xrange = None

        self.plotWidget = pg.plot(title="Mic data")
        self.plot_data_item_ch1 = self.plotWidget.plot(pen=1)
        self.plotItem = self.plotWidget.getPlotItem()
        if self.xrange:
            self.plotItem.setXRange(self.xrange[0], self.xrange[1])
        if self.yrange:
            self.plotItem.setYRange(self.yrange[0], self.yrange[1])

    def run(self):
        for sample in self.reader.take_iter(timeout=duration(seconds=10)):
            values = sample.values
            self.plotted_values = self.plotted_values[len(values):] + values
            # logger.debug(values)
            self.plot_data_item_ch1.setData(self.timebase, self.plotted_values)


if __name__ == "__main__":
    logger.info(f"Running {os.path.basename(__file__)}")
    visualiser = MicManipVisualiser()

    thread = Thread(target=visualiser.run, args=())
    thread.start()
    pg.QtGui.QGuiApplication.exec()
