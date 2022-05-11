from ulab import numpy as np  # to get access to ulab numpy functions
import time
num = 1024 # number of samples
wave1 = np.zeros(num)
wave2 = np.zeros(num)
wave3 = np.zeros(num)
tsample = 0.05
t = np.linspace(0, num*tsample, num)
print(t)
wave1 = np.sin(2*3.14*1000*t)
wave2 = np.sin(2*3.14*2000*t)
wave3 = np.sin(2*3.14*5000*t)
x = wave1+wave2+wave3
re, im = np.fft.fft(x)
for i in re:
    print("("+str(i)+",)")  # print with plotting format
    time.sleep(0.01)
