import numpy as np
from scipy import fft
import scipy.signal as signal
import matplotlib.pyplot as plt

def plot_response(fs, w, h, title=""):
    "Utility function to plot response functions"
    fig = plt.figure()
    ax = fig.add_subplot(111)
    ax.plot(0.5*fs*w/np.pi, 20*np.log10(np.abs(h)))
    ax.set_ylim(-40, 5)
    ax.set_xlim(0, 0.5*fs)
    ax.grid(True)
    ax.set_xlabel('Frequency (Hz)')
    ax.set_ylabel('Gain (dB)')
    ax.set_title(title)

N=len(ll[27])
T=1/8000

def li():
    ll = read()
    showfft(ll[28])
    showfft(ll[27])
    showfft(ll[26])
    showfft(ll[25])

def showfft(data):
    plt.figure()
    nn = len(data)
    tt = 1/ 16000
    x = np.linspace(0.0, nn*tt, nn)
    yff = fft(data)
    xff = np.linspace(0.0, 1.0/(2.0*tt), nn//2)
    plt.plot(xff, 20*np.log10(2.0/nn * np.abs(yff[0:nn//2])))
    plt.grid()
    return yff

# x = np.linspace(0.0, N*T, N)
# y = ll[27]
# yf = fft(y)
# xf = np.linspace(0.0, 1.0/(2.0*T), N//2)
# import matplotlib.pyplot as plt
# plt.plot(xf, 20*np.log10(2.0/N * np.abs(yf[0:N//2])))
# plt.grid()
# plt.show()
#
# b, a = signal.butter(1, [0.100, 0.200], 'bandpass', analog=False)
# w, h = signal.freqz(b, a)
# print([b,a])
# #plot_response(8000, w, h, str([b, a]))
# filtedData = signal.filtfilt(b,a,y)
# yf = fft(filtedData)
# plt.plot(xf, 20*np.log10(2.0/N * np.abs(yf[0:N//2])))
# plt.grid()
# plt.show()


if __name__ == "__main__":
    plt.ion()