import argparse
import dill
import numpy as np
import os
import serial
import time
from loguru import logger
import matplotlib.pyplot as plt

from sensor_comm_dds.communication.config.cyclone_config import CycloneConfig
from sensor_comm_dds.communication.readers.websocket import Websocket
from sensor_comm_dds.communication.data_classes.sequence import Sequence
from sensor_comm_dds.communication.config.serial_config import SerialConfig
from sensor_comm_dds.communication.config.websocket_config import WebsocketConfig
from scipy import signal
from dataclasses import dataclass
from sensor_comm_dds.communication.readers.data_publisher import DataPublisher


@dataclass
class LaserSlipSerialReaderConfig(SerialConfig, WebsocketConfig, CycloneConfig):
    _ = None


class LaserSlipSerialReader:
    def __init__(self, config: LaserSlipSerialReaderConfig):
        self.config = config
        self.data_publisher = DataPublisher(topic_name="LaserSlip", topic_data_type=Sequence)
        self.websocket = Websocket(websocket_server_url=self.config.websocket_server_url)

        self.ser = serial.Serial(self.config.COM, self.config.BAUD)

    def run(self, samples_per_block=3*100, sample_size_in_bytes=2, voltage_ref=5):
        samples = Sequence(values=list(np.zeros((samples_per_block,))))
        voltage_values = [0 for _ in range(samples_per_block)]
        while True:
            logger.info("Waiting for start sequence")
            if True:  #int.from_bytes(self.ser.read(9), byteorder='big', signed=True) == 0:
                logger.info("Read start sequence")
                while True:
                    t_start = time.time()
                    data_bytes = self.ser.read(sample_size_in_bytes * samples_per_block)
                    #logger.debug(data)
                    for i in range(samples_per_block):
                        start_index = i * sample_size_in_bytes
                        sample_bytes = data_bytes[start_index:start_index + sample_size_in_bytes]
                        two_complement_integer = int.from_bytes(sample_bytes, byteorder='big', signed=True)
                        voltage_value = voltage_ref * two_complement_integer/4096
                        #voltage_values[i] = voltage_value
                        #logger.debug(voltage_value)
                        samples.values[i] = voltage_value
                    fs = int(samples_per_block / (time.time() - t_start))
                    if fs < 3000:
                        logger.warning(f'Low sampling frequency: {fs} Hz')
                        continue
                    last_raw_sample = samples.values[-1]
                    #sos = signal.butter(10, 50, 'hp', fs=fs, output='sos')
                    #samples.values = list(signal.sosfilt(sos, samples.values))
                    self.data_publisher.publish_sensor_data(samples)
                    logger.debug(f'{int(samples_per_block / (time.time() - t_start))} Hz, last sample value: {last_raw_sample:.3f}')
                    #time.sleep(0.1)
                    # Send data to WebSocket as JSON
                    '''if self.config.ENABLE_WS:
                        for i in range(samples_per_block):
                            v = {'v': samples.values[i]}
                            self.websocket.send_data_to_websocket(v)'''


if __name__ == "__main__":
    logger.info(f"Running {os.path.basename(__file__)}")
    parser = argparse.ArgumentParser(description='Read data from a LaserSlip sensor over a serial connection.')

    mic_reader = LaserSlipSerialReader(config=LaserSlipSerialReaderConfig(ENABLE_WS=False,
                                                                  topic_names=np.array(["LaserSlip"]),
                                                                  COM='/dev/serial/by-id/usb-Arduino_IO_Coupling_C6E76762B4D1E02A-if00',
                                                                  BAUD=115200))

    mic_reader.run()
