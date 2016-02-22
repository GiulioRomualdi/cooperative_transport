import rospy
from threading import Lock

class Subscriber:
    """Topic subscription with locks."""

    def __init__(self, topic_name, msg_type):

        rospy.Subscriber(topic_name, msg_type, self.callback)
        self._is_ready = False
        self.lock = Lock()

    @property
    def is_ready(self):
        self.lock.acquire()
        value = self._is_ready
        self.lock.release()
        return value
        
    @is_ready.setter
    def is_ready(self, value):
        self.lock.acquire()
        self._is_ready = value
        self.lock.release()

    @property
    def data(self):
        self.lock.acquire()
        value = self._data
        self.lock.release()
        return value
        
    @data.setter
    def data(self, value):
        self.lock.acquire()
        self._is_ready = value
        self.lock.release()

    def callback(self, data):
        self.is_ready = True
        self.data = data
