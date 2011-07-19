import cjson

class CJSONStream(object):
    def __init__(self, input):
        self.input = input
        
    # def read_next(self):
    #     for line in self.input:
    #         # line = self.input.readline()
    #         print('REad %r' % line)
    #         if line == '':
    #             return None
    #         line = line.strip()
    #         if line:
    #             return cjson.decode(line)
    #     
    def read_next(self):
        while True:
            line = self.input.readline()
            if line == '':
                return None
            line = line.strip()
            if line:
                return cjson.decode(line)
        
