import csv
import numpy as np
import matplotlib.pyplot as plt
from scipy.io import wavfile as wf
from scipy import fft

"""
time = np.arange(0, 10, 0.1)
amplitude = np.sin(time)

plt.plot(time, amplitude)
plt.title('Sine Wave')
plt.xlabel('Time')
plt.ylabel('Amplitude = sin(time)')
plt.grid(True, which='both')
plt.axhline(y=0, color='k')
plt.show()


# Original signal
sample_length = 20 # Length in seconds
time_step = 0.02 # Time between samples
period = 5 # Cycle period
time_vec = np.arange(0, sample_length, time_step) # Evenly spaced integers {time_step} apart
sample_rate = sample_length / time_step # For reference
rng = np.random.default_rng() # Random number generator instance
# Sine function + random vertical shift
sig = (np.sin((2 * np.pi / period) * time_vec) + (0.5 * rng.standard_normal(time_vec.size)))
plt.figure(figsize=(6, 5))
plt.title(f'Sample Rate: {sample_rate}Hz')
plt.plot(time_vec, sig, label='Original Signal')

# FFT
sig_fft = fft.fft(sig) # Perfom FFT (complex object of real and imaginary values)
power = np.abs(sig_fft)**2 # Absolute FFT values
sample_freq = fft.fftfreq(sig.size, d=time_step) # Frequency of each sample
plt.figure(figsize=(6, 5))
plt.plot(sample_freq, power)
plt.xlabel('Frequency [Hz]')
plt.ylabel('Power')
pos_mask = np.where(sample_freq > 0) # Indexes for positive frequencies
freqs = sample_freq[pos_mask] # Positive frequencies
peak_freq = freqs[power[pos_mask].argmax()] # Peak frequency in positive range
np.allclose(peak_freq, 1/period)
axes = plt.axes([0.55, 0.3, 0.3, 0.5])
plt.title('Peak Frequency')
plt.plot(freqs[:8], power[pos_mask][:8])
plt.setp(axes, yticks=[])

# High frequencies removed
hi_freq_fft = sig_fft.copy()
hi_freq_fft[np.abs(sample_freq) > peak_freq] = 0
filtered_sig = fft.ifft(hi_freq_fft)
plt.figure(figsize=(6, 5))
plt.plot(time_vec, sig, label='Original Signal')
plt.plot(time_vec, filtered_sig, linewidth=3, label='Filtered Signal')
plt.xlabel('Time [s]')
plt.ylabel('Amplitude')
plt.legend(loc='best')
plt.show()
"""
"""
with open('linenoise.csv', newline='') as linenoise:
    noisedata = csv.reader(linenoise)
    for sample in noisedata:
        print(sample)

"""
noisedata = open('linenoise.csv').read().splitlines()

samplerate = 48000
saplelength = (4096-1)/samplerate
timestep = 1/samplerate

noisedata_fft = fft.fft(noisedata)
mag = np.abs(noisedata_fft)
samplefreq = fft.fftfreq(4096, d=timestep)

plt.figure(figsize=(6,5))
plt.plot(samplefreq, mag)
plt.xlabel('Frequency [Hz]')
plt.ylabel('Magnitude')

plt.show()



"""sample_rate, data = wf.read('sin_1000Hz_-3dBFS_10s.wav')

sample_length = (data.size - 1) / sample_rate # Length in seconds
time_step = 1 / sample_rate # Time between samples
time_vec = np.arange(0, sample_length, time_step) # Evenly spaced integers {time_step} apart

data_fft = fft.fft(data)
power = np.abs(data_fft) # Absolute FFT values
sample_freq = fft.fftfreq(data.size, d=time_step) # Frequency of each sample

plt.figure(figsize=(6, 5))
plt.plot(sample_freq, power)
plt.xlabel('Frequency [Hz]')
plt.ylabel('Power')

pos_mask = np.where(sample_freq > 0) # Indexes for positive frequencies
freqs = sample_freq[pos_mask] # Positive frequencies
peak_freq = freqs[power[pos_mask].argmax()] # Peak frequency in positive range
axes = plt.axes([0.55, 0.3, 0.3, 0.5])

plt.title('Peak Frequency')
plt.plot(freqs[9990:10010], power[pos_mask][9990:10010])
plt.setp(axes, yticks=[])
plt.show()
plt.close()"""