from cyclonedds.core import Listener
from loguru import logger


class LivelinessListener(Listener):
    def __init__(self, topic_name, **kwargs):
        super().__init__(**kwargs)
        self.topic_name = topic_name

    def on_liveliness_changed(self, reader, status):
        logger.info(f'>> "{self.topic_name}" Topic Liveliness event')