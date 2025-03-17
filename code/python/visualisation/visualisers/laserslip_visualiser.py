import numpy as np
from loguru import logger
from cyclonedds.util import duration
import pyqtgraph as pg
from PyQt5 import QtCore
from PyQt5.QtWidgets import QApplication
from visualisation.visualisers.visualiser import Visualiser
from communication.data_classes.sequence import Sequence
from communication.readers.data_publisher import DataPublisher
import os
import time


samples_per_block = 3*100  # must agree with readout microcontroller firmware


class SampleReaderThread(QtCore.QThread):

    sample_signal = QtCore.pyqtSignal(object)

    def __init__(self, cyclone_reader):
        QtCore.QThread.__init__(self)
        self.cyclone_reader = cyclone_reader

    def run(self):
        logger.info('Running SampleReaderThread')
        ctr = 0
        t_start = time.time()
        for sample in self.cyclone_reader.take_iter(timeout=duration(seconds=10)):
            self.sample_signal.emit(sample)
            ctr += 1
            if ctr == 200:
                logger.info(f'LaserSlipVisualiser readout frequency: {200 * samples_per_block/(time.time() - t_start)}')
                ctr = 0
                t_start = time.time()


class LaserSlipVisualiser(Visualiser):
    def __init__(self):
        super().__init__(topic_data_type=Sequence, description="Visualise data from a LaserSlip sensor.")
        self.samples_per_frame = 20000
        self.plotted_values = np.zeros((self.samples_per_frame,))
        #self.timebase = np.arange(self.samples_per_frame) / self.fs
        self.timebase = np.arange(self.samples_per_frame)

        self.plotWidget = pg.plot(title="Laser output voltage")
        self.plot_data_item_ch1 = self.plotWidget.plot(pen=1)
        self.plotItem = self.plotWidget.getPlotItem()
        self.yrange = None
        self.xrange = None
        if self.xrange:
            self.plotItem.setXRange(self.xrange[0], self.xrange[1])
        if self.yrange:
            self.plotItem.setYRange(self.yrange[0], self.yrange[1])

        self.sample_reader_thread = SampleReaderThread(self.reader)
        self.sample_reader_thread.sample_signal.connect(self.update_graph)
        self.sample_reader_thread.start()

        self.frame_publisher = DataPublisher(topic_name="LaserSlipFrame", topic_data_type=Sequence)

    def update_graph(self, sample):
        self.plotted_values = np.roll(self.plotted_values, -samples_per_block, axis=0)  # moves all samples up a block
        self.plotted_values[-samples_per_block:] = np.array(sample.values)
        self.plot_data_item_ch1.setData(self.timebase, self.plotted_values)

        frame_to_publish = Sequence(values=list(self.plotted_values))
        self.frame_publisher.publish_sensor_data(frame_to_publish)


if __name__ == "__main__":
    logger.info(f"Running {os.path.basename(__file__)}")
    app = QApplication([])
    visualiser = LaserSlipVisualiser()
    app.exec()
