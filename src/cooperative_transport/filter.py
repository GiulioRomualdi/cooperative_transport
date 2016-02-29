import collections

class LowPassFilter():
    def __init__(self, init_value, length):
        self.length = length
        self.data_buffer = collections.deque(self.length*[init_value], self.length)

    def filtering(self, x):
        self.data_buffer.append(x)
        return sum(self.data_buffer) / self.length
