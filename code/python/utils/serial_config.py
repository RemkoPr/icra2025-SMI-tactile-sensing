from dataclasses import dataclass


@dataclass
class SerialConfig:
    BAUD: int = 115200  # Baud rate
    COM: str = '/dev/ttyACM0'  # Serial port
