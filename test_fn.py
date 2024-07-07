import math
import numpy as np

def _control_signal(self, amplitude, phase, duty_cycle, array_dim=100):
    '''
    create a smooth periodic function with amplitude, phase, and duty cycle,
    amplitude, phase and duty cycle are in [0, 1]
    '''
    assert(amplitude >= 0 and amplitude <= 1)
    assert(phase >= 0 and phase <= 1)
    assert(duty_cycle >= 0 and duty_cycle <= 1)
    command = np.zeros(array_dim)

    # create a 'top-hat function'
    up_time = array_dim * duty_cycle
    temp = [amplitude if i < up_time else -amplitude for i in range(0, array_dim)]

    # smoothing kernel
    kernel_size = int(array_dim / 10)
    kernel = np.zeros(int(2 * kernel_size + 1))
    sigma = kernel_size / 3
    for i in range(0, len(kernel)):
        kernel[i] =  math.exp(-(i - kernel_size) * (i - kernel_size) / (2 * sigma**2)) / (sigma * math.sqrt(math.pi))
    sum = np.sum(kernel)

    # smooth the function
    for i in range(0, array_dim):
        command[i] = 0
        for d in range(1, kernel_size + 1):
            if i - d < 0:
                command[i] += temp[array_dim + i - d] * kernel[kernel_size - d]
            else:
                command[i] += temp[i - d] * kernel[kernel_size - d]
        command[i] += temp[i] * kernel[kernel_size]
        for d in range(1, kernel_size + 1):
            if i + d >= array_dim:
                command[i] += temp[i + d - array_dim] * kernel[kernel_size + d]
            else:
                command[i] += temp[i + d] * kernel[kernel_size + d]
        command[i] /= sum

    # shift according to the phase
    final_command = np.zeros(array_dim)
    start = int(math.floor(array_dim * phase))
    current = 0
    for i in range(start, array_dim):
        final_command[current] = command[i]
        current += 1
    for i in range(0, start):
        final_command[current] = command[i]
        current += 1

    assert(len(final_command) == array_dim)
    return final_command


if __name__ == "__main__":
    import matplotlib.pyplot as plt
    amplitude = 0.5
    phase = 0.
    duty_cycle = 0.1
    NUM_POINTS = 100
    array_dim = int(NUM_POINTS/duty_cycle)
    command = _control_signal(None, amplitude, phase, duty_cycle, array_dim)[:NUM_POINTS]
    plt.plot(command)
    plt.show()