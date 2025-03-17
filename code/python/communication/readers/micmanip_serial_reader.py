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
from dataclasses import dataclass
from sensor_comm_dds.communication.readers.data_publisher import DataPublisher


@dataclass
class MicManipSerialReaderConfig(SerialConfig, WebsocketConfig, CycloneConfig):
    _ = None


class MicManipReader:
    def __init__(self, config: MicManipSerialReaderConfig):
        self.config = config
        self.data_publisher = DataPublisher(topic_name="MicManip", topic_data_type=Sequence)
        self.websocket = Websocket(websocket_server_url=self.config.websocket_server_url)

        self.ser = serial.Serial(self.config.COM, self.config.BAUD)

    def read_num_samples(self, num_samples):
        samples = np.zeros((num_samples,))
        while True:
            data = self.ser.read(1)
            if data == b'\xAA':
                for i in range(num_samples):
                    bytes_ = self.ser.read(4)
                    two_complement_integer = int.from_bytes(bytes_, byteorder='big', signed=True)
                    samples[i] = two_complement_integer
                return samples

    def read_x_seconds(self, seconds, num_samples=1000):
        samples = np.zeros((seconds * 10000,))
        ctr = 0
        start_time = time.time()
        while time.time() - start_time < seconds:
            for i in range(num_samples):
                bytes_ = self.ser.read(4)
                two_complement_integer = int.from_bytes(bytes_, byteorder='big', signed=True)
                samples[ctr * num_samples + i] = two_complement_integer
            ctr += 1
        return samples

    def run(self, samples_per_block=50, sample_size=3):
        # WARNING: samples_per_block must be set to the same value in your visualiser script
        samples = Sequence(values=list(np.zeros((samples_per_block,))))
        while True:
            logger.info("Waiting for start sequence")
            if True:  #int.from_bytes(self.ser.read(9), byteorder='big', signed=True) == 0:
                logger.info("Read start sequence")
                while True:
                    t_start = time.time()
                    data = self.ser.read(sample_size * samples_per_block)
                    # logger.debug(data)
                    for i in range(samples_per_block):
                        start_index = i * sample_size
                        sample_bytes = data[start_index:start_index + sample_size]
                        two_complement_integer = int.from_bytes(sample_bytes, byteorder='big', signed=True)

                        # logger.debug(two_complement_integer)
                        samples.values[i] = two_complement_integer
                    self.data_publisher.publish_sensor_data(samples)
                    logger.debug(f'{samples_per_block / (time.time() - t_start)} Hz')
                    #time.sleep(0.1)
                    # Send data to WebSocket as JSON
                    # if self.config.ENABLE_WS:
                    #    self.websocket.send_data_to_websocket(samples)


if __name__ == "__main__":
    logger.info(f"Running {os.path.basename(__file__)}")
    parser = argparse.ArgumentParser(description='Read data from a MicManip sensor over a serial connection.')

    mic_reader = MicManipReader(config=MicManipSerialReaderConfig(ENABLE_WS=False,
                                                                  topic_names=np.array(["MicManip"]),
                                                                  COM='/dev/serial/by-id/usb-Arduino_LLC_Arduino_MKR1000_A1AF5F405150435437202020FF17163A-if00',
                                                                  BAUD=115200))

    mic_reader.run()
