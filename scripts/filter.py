# -*- coding: utf-8 -*-
import matplotlib.pyplot as plt
from scipy import signal
import scipy.signal as signal
import numpy as np
import pylab as pl
from pylab import mpl
import numpy as np

mpl.rcParams['font.sans-serif'] = ['SimHei']  # 指定默认字体

mpl.rcParams['axes.unicode_minus'] = False  # 解决保存图像是负号 '-' 显示为方块的问题


def ffileter():
    h1 = signal.remez(201, (0, 0.18,  0.2,  0.50), (0.01, 1))
    h2 = signal.remez(201, (0, 0.38,  0.4,  0.50), (1, 0.01))
    h3 = np.convolve(h1, h2)
    h4 = signal.remez(131, (0, 0.18, 0.2, 0.38, 0.4, 0.50), (0.1, 1, 0.1))

    w, h = signal.freqz(h4, 1)
    pl.plot(w/2/np.pi, 20*np.log10(np.abs(h)))

    pl.legend()
    pl.xlabel(u"正规化频率 周期/取样")
    pl.ylabel(u"幅值(dB)")
    pl.title(u"低通和高通级联为带通滤波器")
    pl.show()


def plot_response(fs, w, h, title=""):
    "Utility function to plot response functions"
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(0.5*fs*w/np.pi, 20*np.log10(np.abs(h)), 'b')
    angles = np.unwrap(np.angle(h))
    ax2 = ax.twinx()
    ax2.plot(0.5*fs*w/np.pi, angles, 'g')
    ax2.grid()
    ax2.axis('tight')
    ax.set_ylim(-80, 5)
    ax.set_xlim(0, 0.5*fs)
    ax.grid(True)
    ax.set_xlabel('Frequency (Hz)')
    ax.set_ylabel('Gain (dB)')
    ax.set_title(title)


def buff():
    b, a = signal.butter(1, [0.170, 0.300], 'bandpass', analog=False)
    w, h = signal.freqz(b, a)
    plot_response(8000, w, h, str([b, a]))
    print(b, a)
    b, a = signal.butter(3, 0.5, 'low', analog=True)
    w, h = signal.freqs(b, a)
    plot_response(8000, w, h)
    print(b, a)


def iir():
    b, a = signal.iirdesign([700/4000, 900/4000], [2/40, 14/40], 0.1,
                            60.0, ftype="cheby2", fs=8000)
    w, h = signal.freqz(b, a)
    plot_response(8000, w, h, str([b, a]))
    print(b, a)


def dd():
    fs = 8000.0         # Sample rate, Hz
    bandcenter = 1000
    band = [bandcenter * 0.9, bandcenter * 1.1]  # Desired pass band, Hz
    # Width of transition from pass band to stop band, Hz
    trans_width = bandcenter * 0.8
    numtaps = 11        # Size of the FIR filter.
    edges = [0, band[0] - trans_width, band[0], band[1],
             band[1] + trans_width, 0.5*fs]
    taps = signal.remez(numtaps, edges, [0, 1, 0], Hz=fs)
    w, h = signal.freqz(taps, [1], worN=2000)
    a = str([trans_width, band, numtaps])
    plot_response(fs, w, h, "Band-pass Filter" + a)
    buff()

if __name__ == "__main__":
    b = [0.0002]
    a = [1, b[0] - 1]
    w, h = signal.freqz(b, a)
    plot_response(16000, w, h, str([b, a]))
