import argparse
# GSR + PPG with Shimmer
import struct
import serial

# Input arguments
#parser = argparse.ArgumentParser()
#parser.add_argument('--name', help = 'Saving location.')
#parser.add_argument('--rate', help = 'rate [Hz].')
#parser.add_argument('--gsr', help = 'GSR and PPG from Shimmer.')

#input_args = parser.parse_args()

class CollectDataShimmer(object):

    def wait_for_ack(self):
        ddata = ""
        ack = struct.pack('B', 0xff)
        while ddata != ack:
            ddata = self.ser.read(1)

    def __init__(self):
	self._rate = 100.0  # Hz
        self._plotgsr = True
        self._gsrstream = False
        self._gsrsd = True

        self._gsrsignal = [0.0] * 3

        print "GSR Setting up GSR+"
        self.ser = serial.Serial("/dev/rfcomm0", 115200)
        self.ser.flushInput()
        print "GSR port opening, done."
        self.ser.write(struct.pack('BBBB', 0x08 , 0x04, 0x01, 0x00))  #GSR and PPG
        self.wait_for_ack()
        print "GSR sensor setting, done."
        self.ser.write(struct.pack('BB', 0x5E, 0x01))
        self.wait_for_ack()
        self.ser.write(struct.pack('B', 0x03))
        self.wait_for_ack()
        ddata = ""
        numbytes = 0
        framesize = 3
        while numbytes < framesize:
            ddata += self.ser.read(framesize)
            numbytes = len(ddata)

        data = ddata[0:framesize]

        (samplingrate) = struct.unpack('H', data[1:3])
        print "Current sampling rate: %d (%.2fHz)" % (samplingrate[0], (32768.0/samplingrate[0]))   

    def activate_thread(self):
        try:
            while True:
                self._dump_data()
        except KeyboardInterrupt:
            self.clean_shutdown()

    def _dump_data(self):
	# start writing data
        self.ser.write(struct.pack('B', 0x92)) #0x70 Logging and streaming
        self.wait_for_ack()
        print "GSR start SD writing command sending, done."

    def clean_shutdown(self):
        if self._plotgsr:
            if self._gsrstream:
                self._filegsr.close()
                self.ser.write(struct.pack('B', 0x20))
                print "GSR stop command sent, waiting for ACK_COMMAND"
                self.wait_for_ack()
            elif self._gsrsd:
                self.ser.write(struct.pack('B', 0x93)) #0x97 stop logging and streaming
                print "GSR SD stop command sent, waiting for ACK_COMMAND"
                self.wait_for_ack()
            # close serial port
            #self.ser.close()

def main():
    dumper = CollectDataShimmer()
    dumper.activate_thread()


if __name__ == "__main__":
    main()
