import ctypes
import pathlib
import numpy as np
from numpy.ctypeslib import ndpointer
from time import perf_counter
from time import sleep

def sampling():
    samplerate = 48000
    chunk = 4
    adc_program = pathlib.Path().absolute() / "rpi_adc_test.so"
    adc = ctypes.CDLL(adc_program)


    adc.init.restype = None
    adc.readsamples.restype = None
    adc.terminate.restype = None
    adc.init.argtypes = [ctypes.c_int, ctypes.c_int]
    adc.readsamples.argtypes = [ndpointer(ctypes.c_uint16), ctypes.c_int]
    adc.terminate.argtypes = [ctypes.c_int]

    buffer = np.empty(chunk, dtype=np.uint16)

    adc.init(samplerate, chunk)
    adc.readsamples(buffer, chunk)

    for j in range(0, 5):
        for i in range(0, len(buffer)):
            print(buffer[i])


        # rec_buffer = np.append(rec_buffer, buffer)
    adc.terminate(0)


def reading():
    import matplotlib.pyplot as plt
    recording = np.load('recording.npy')
    plt.plot(recording)
    plt.show()


sampling()
#reading()
