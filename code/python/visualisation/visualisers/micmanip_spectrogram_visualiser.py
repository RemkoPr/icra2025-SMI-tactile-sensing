import sys
import os
from PyQt5.QtCore import pyqtSignal, QObject
from PyQt5.QtWidgets import QApplication, QMainWindow
from visualisation.visualisers.mel_spectrogram_visualiser import SpectrogramVisualiser
from threading import Thread
from loguru import logger


class SpectrogramUpdater(QObject):
    spectrogram_data_signal = pyqtSignal(object)


class MicManipSpectrogramVisualiser(SpectrogramVisualiser):
    def __init__(self):
        super().__init__(topic_name_prefix="MicManip", description="Visualise data from a MicManip sensor in a spectrogram.")
        self.db_levels = [50, 130]


class MicManipMainWindow(QMainWindow):
    def __init__(self, parent=None):
        super(MicManipMainWindow, self).__init__(parent)

        # Set the window title and geometry
        self.setWindowTitle('Spectrogram Viewer')
        self.setGeometry(100, 100, 800, 600)

        # Create the main widget
        self.spectrogram_widget = MicManipSpectrogramVisualiser()
        self.setCentralWidget(self.spectrogram_widget.container)


if __name__ == '__main__':
    logger.info(f"Running {os.path.basename(__file__)}")
    # Create the Qt Application
    app = QApplication(sys.argv)

    # Create and show the main window
    main_window = MicManipMainWindow()
    main_window.show()

    # Start the data visualization in a separate thread
    visualiser = main_window.spectrogram_widget
    thread = Thread(target=visualiser.run, args=())
    thread.start()

    # Run the main Qt loop
    sys.exit(app.exec_())
