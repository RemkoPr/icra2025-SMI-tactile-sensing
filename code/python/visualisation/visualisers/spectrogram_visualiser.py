import sys
import os
import time

import numpy as np
from PyQt5.QtCore import pyqtSignal, QObject, Qt, QThread
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
import pyqtgraph as pg
from scipy.signal import spectrogram
from communication.readers.data_publisher import DataPublisher
from communication.data_classes.sequence import Sequence
from visualisation.visualisers.visualiser import Visualiser
from cyclonedds.util import duration
from threading import Thread
from loguru import logger


class SpectrogramUpdater(QObject):
    spectrogram_data_signal = pyqtSignal(object)


class SpectrogramVisualiser(Visualiser):
    def __init__(self, topic_name_prefix="", parent=None, description="Visualise timeseries data as a spectrogram."):
        super().__init__(topic_data_type=Sequence, description=description)

        # Create a QWidget as the container
        self.container = QWidget(parent)
        self.layout = QVBoxLayout(self.container)

        # Create a PlotWidget for plotting the spectrogram
        self.plot_widget = pg.PlotWidget()
        self.layout.addWidget(self.plot_widget)

        self.fs = 20000  # Sampling frequency
        self.samples_per_frame = self.fs  # Frame is what is viewed on screen. The visualisation will show a one-second window (if self.fs is correct)
        self.samples_per_block = 50  # Block is the unit that is published by the sensor reader. The value here must agree with the reader script.
        self.blocks_per_frame = self.samples_per_frame/self.samples_per_block
        self.plotted_values = [0 for _ in range(self.samples_per_frame)]

        # Create an instance of SpectrogramUpdater to manage signals
        self.updater = SpectrogramUpdater()
        self.updater.spectrogram_data_signal.connect(self.update_spectrogram)

        # Publish spectrogram data
        self.frame_data_publisher = DataPublisher(topic_name=f"{topic_name_prefix}Frame", topic_data_type=Sequence)
        self.spectrogram_data_publisher = DataPublisher(topic_name=f"{topic_name_prefix}Spectrogram", topic_data_type=Sequence)

        self.db_levels = [-100, 0]

    def update_spectrogram(self, signal):
        # Calculate the spectrogram
        f, t, Sxx = spectrogram(np.array(signal), self.fs)
        #norm_factor = np.sum(Sxx, axis=1, keepdims=True)
        #Sxx /= norm_factor
        Sxx = 10 * np.log10(Sxx + np.finfo(float).eps)  # Convert to dB, small offset to avoid log(0)
        spectrogram_to_publish = Sequence(values=[Sxx.shape[0], Sxx.shape[1]] + list(Sxx.flatten()))  # first two values are shape
        self.spectrogram_data_publisher.publish_sensor_data(spectrogram_to_publish)
        # Clear the plot
        self.plot_widget.clear()

        # Update the image in the plot widget
        #logger.debug(f'min {np.min(Sxx):.2f}, max {np.max(Sxx):.2f}')
        img = pg.ImageItem(np.transpose(Sxx))
        img.setRect(pg.QtCore.QRectF(t[0], f[0], t[-1] - t[0], f[-1] - f[0]))
        # Set a fixed color mapping using a lookup table
        cmap = pg.colormap.get('viridis')  # You can choose another colormap if desired
        lut = cmap.getLookupTable(0.0, 1.0, 256)
        img.setLookupTable(lut)
        # Set fixed levels for color mapping: values outside this range will be clipped.
        # This ensures that, for example, -80 dB always maps to the same color.
        img.setLevels(self.db_levels)

        # Update the image in the plot widget
        self.plot_widget.addItem(img)
        self.plot_widget.setLabel('left', 'Frequency', units='Hz')
        self.plot_widget.setLabel('bottom', 'Time', units='s')
        self.plot_widget.setTitle('Spectrogram')

    def run(self):
        frame_to_publish = Sequence(values=[0 for _ in range(self.samples_per_frame)])
        block_ctr = 0
        t_start = time.time()
        for sample_block in self.reader.take_iter(timeout=duration(seconds=10)):
            values = sample_block.values
            self.plotted_values = self.plotted_values[len(values):] + values
            frame_to_publish.values = self.plotted_values
            self.frame_data_publisher.publish_sensor_data(frame_to_publish)

            # Emit the signal with the new data
            self.updater.spectrogram_data_signal.emit(self.plotted_values)

            block_ctr += 1
            if block_ctr == 200:
                self.fs = 200 * self.samples_per_block/(time.time() - t_start)
                logger.info(f'SpectrogramVisualiser readout frequency: {self.fs}')
                block_ctr = 0
                t_start = time.time()


class MainWindow(QMainWindow):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)

        # Set the window title and geometry
        self.setWindowTitle('Spectrogram Viewer')
        self.setGeometry(100, 100, 800, 600)

        # Create the main widget
        self.spectrogram_widget = SpectrogramVisualiser()
        self.setCentralWidget(self.spectrogram_widget.container)


if __name__ == '__main__':
    logger.info(f"Running {os.path.basename(__file__)}")
    # Create the Qt Application
    app = QApplication(sys.argv)

    # Create and show the main window
    main_window = MainWindow()
    main_window.show()

    # Start the data visualization in a separate thread
    visualiser = main_window.spectrogram_widget
    thread = QThread(target=visualiser.run, args=())
    thread.start()

    # Run the main Qt loop
    sys.exit(app.exec_())
