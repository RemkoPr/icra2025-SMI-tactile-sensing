import sys
import os
import time

import numpy as np
import torch
import torchaudio.transforms as T
from collections import deque
from PyQt5.QtCore import pyqtSignal, QObject, Qt, QThread
from PyQt5.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
import pyqtgraph as pg
from communication.readers.data_publisher import DataPublisher
from communication.data_classes.sequence import Sequence
from visualisation.visualisers.visualiser import Visualiser
from cyclonedds.util import duration
from loguru import logger


class SpectrogramUpdater(QObject):
    spectrogram_data_signal = pyqtSignal(object)


class SpectrogramVisualiser(Visualiser):
    def __init__(self, topic_name_prefix="", parent=None, description="Visualise timeseries data as a mel spectrogram."):
        super().__init__(topic_data_type=Sequence, description=description)

        self.container = QWidget(parent)
        self.layout = QVBoxLayout(self.container)

        self.plot_widget = pg.PlotWidget()
        self.layout.addWidget(self.plot_widget)
        self.img = pg.ImageItem()
        cmap = pg.colormap.get('viridis')
        self.img.setLookupTable(cmap.getLookupTable(0.0, 1.0, 256))
        self.plot_widget.addItem(self.img)

        self.fs = 20000  # Sampling frequency
        self.samples_per_frame = self.fs  # One-second window
        self.samples_per_block = 50  # Block size
        self.blocks_per_frame = self.samples_per_frame / self.samples_per_block
        self.plotted_values = deque([0] * self.samples_per_frame, maxlen=self.samples_per_frame)

        self.updater = SpectrogramUpdater()
        self.updater.spectrogram_data_signal.connect(self.update_spectrogram)

        self.frame_data_publisher = DataPublisher(topic_name=f"{topic_name_prefix}Frame", topic_data_type=Sequence)
        self.spectrogram_data_publisher = DataPublisher(topic_name=f"{topic_name_prefix}MelSpectrogram", topic_data_type=Sequence)

        self.db_levels = [-100, 0]
        self.mel_transform = T.MelSpectrogram(sample_rate=self.fs, n_fft=400, win_length=400, hop_length=160, n_mels=64, normalized=False)

    def update_spectrogram(self, signal):
        signal = np.array(signal) - np.mean(np.array(signal))
        signal_tensor = torch.as_tensor(signal, dtype=torch.float32).unsqueeze(0)
        mel_spectrogram = self.mel_transform(signal_tensor).squeeze().numpy()
        mel_spectrogram_db = 10 * np.log10(mel_spectrogram + np.finfo(float).eps)
        #logger.debug(f'min {np.min(mel_spectrogram_db):.2f} ; max {np.max(mel_spectrogram_db):.2f}')
        spectrogram_to_publish = Sequence(values=[mel_spectrogram_db.shape[0], mel_spectrogram_db.shape[1]] + list(mel_spectrogram_db.flatten()))
        self.spectrogram_data_publisher.publish_sensor_data(spectrogram_to_publish)

        #self.img.setRect(pg.QtCore.QRectF(0, 0, mel_spectrogram_db.shape[1], mel_spectrogram_db.shape[0]))
        self.img.setImage(np.transpose(mel_spectrogram_db))
        self.img.setLevels(self.db_levels)
        
        self.plot_widget.addItem(self.img)
        self.plot_widget.setLabel('left', 'Mel Filter Banks')
        self.plot_widget.setLabel('bottom', 'Time Frames')
        self.plot_widget.setTitle('Mel Spectrogram')

    def run(self):
        frame_to_publish = Sequence(values=[0 for _ in range(self.samples_per_frame)])
        block_ctr = 0
        t_start = time.time()
        for sample_block in self.reader.take_iter(timeout=duration(seconds=10)):
            values = sample_block.values
            self.plotted_values.extend(values)
            frame_to_publish.values = self.plotted_values
            self.frame_data_publisher.publish_sensor_data(frame_to_publish)

            self.updater.spectrogram_data_signal.emit(self.plotted_values)

            block_ctr += 1
            if block_ctr == 200:
                self.fs = 200 * self.samples_per_block / (time.time() - t_start)
                self.mel_transform.fs = self.fs
                logger.info(f'MelSpectrogramVisualiser readout frequency: {self.fs}')
                block_ctr = 0
                t_start = time.time()


class MainWindow(QMainWindow):
    def __init__(self, parent=None):
        super(MainWindow, self).__init__(parent)
        self.setWindowTitle('Mel Spectrogram Viewer')
        self.setGeometry(100, 100, 800, 600)
        self.spectrogram_widget = SpectrogramVisualiser()
        self.setCentralWidget(self.spectrogram_widget.container)


if __name__ == '__main__':
    logger.info(f"Running {os.path.basename(__file__)}")
    app = QApplication(sys.argv)
    main_window = MainWindow()
    main_window.show()

    visualiser = main_window.spectrogram_widget
    thread = QThread(target=visualiser.run, args=())
    thread.start()

    sys.exit(app.exec_())
