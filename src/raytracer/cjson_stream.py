import cjson
import time

debug_speed = False


class CJSONStream(object):

    def __init__(self, input_stream):
        self.input = input_stream

    def read_next(self):
        while True:
            t0 = time.time()
            c0 = time.clock()
            line = self.input.readline()
            deltat = time.time() - t0
            deltac = time.clock() - c0
            if debug_speed:
                print('Read line[%s] in %d ms, %d clock' % 
                      (len(line), deltat * 1000, deltac * 1000))
            if line == '':
                return None
            line = line.strip()
            if line:
                return cjson.decode(line)  # @UndefinedVariable

