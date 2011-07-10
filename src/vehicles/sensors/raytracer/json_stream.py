

# Part of code from http://www.mikealrogers.com/archives/291

class JSONStream(object):
    def __init__(self, stream, decoder=None):
        self.sbuffer = ''
        if decoder is None:
            import simplejson #@UnresolvedImport
            self.decoder = simplejson.JSONDecoder()
        else:
            self.decoder = decoder
        self.stream = stream
        self.object_buffer = []
        
    def process_read(self, data):
        """Parse out json objects and fire callbacks."""
        self.sbuffer += data
        self.parsing = True
        while self.parsing:
            # Remove erroneus data in front of callback object
            index = self.sbuffer.find('{')
            if index is not - 1 and index is not 0:
                self.sbuffer = self.sbuffer[index:]
            # Try to get a json object from the data stream
            try:
                obj, index = self.decoder.raw_decode(self.sbuffer)
            except Exception:
                self.parsing = False
            # If we got an object, put it in the object buffer
            if self.parsing:
                self.object_buffer.append(obj)
                self.sbuffer = self.sbuffer[index:]

    def read_next(self):
        # if the buffer is empty
        while len(self.object_buffer) == 0:
            # read next line
            line = self.stream.readline()
            # if we hit EOF
            if line == '': 
                # .. and there is trailing data in the buffer
                if len(self.sbuffer.strip()) > 0:
                    raise Exception, \
                        "JSONStream: Trailing data '%s'" % self.sbuffer
                else:
                    return None
            self.process_read(line)

        # return first object from buffer
        ob = self.object_buffer[0]
        self.object_buffer = self.object_buffer[1:]
        return ob         
   
        
