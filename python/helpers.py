import serial
import serial.tools.list_ports
from threading import Timer

def get_serial():
    return list(serial.tools.list_ports.comports())


def get_ArduinoUno_ports():
    ports = []
    for p in get_serial():
        if "Arduino Uno" in p.description:
            ports.append(p.name)
    return ports

class ResettableTimer(object):
    def __init__(self, interval, function):
        self.interval = interval
        self.function = function
        self.timer = Timer(self.interval, self.function)

    def run(self):
        if not self.timer.is_alive():
            self.timer.start()

    def reset(self):
        #print("reset")
        self.timer.cancel()
        self.timer = Timer(self.interval, self.function)
        self.timer.start()

    def pause(self):
        #print("pause")
        self.timer.cancel()

    def set_interval(self, interval):
        self.interval = interval
        self.reset()

    def stop(self):
        self.timer.cancel()


class File():
    def __init__(self):
        self._l = []

    def enqueue(self, v):
        self._l.append(v)

    def dequeue(self):
        return self._l.pop(0)

    def is_empty(self):
        return len(self._l) == 0

if __name__ == "__main__":
    print(get_ArduinoUno_ports())


