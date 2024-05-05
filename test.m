% Define frequency (in Hz)
f = 1;

% Define time range (in seconds)
t = 0:0.01:1;

% Generate sine wave
y = sin(2*pi*f*t);

% Plot the sine wave
plot(t, y)
xlabel('Time (s)')
ylabel('Amplitude')
title('Sine Wave')
grid on

% Define resistance and capacitance for the filter
R = 1000;
C = 1e-6;

% Transfer function of the low-pass filter
F = 1/(R*C);

% Create bode plot
[mag, phase, omega] = bode(F, 2*pi*f);  % convert frequency to radians/sec

% Plot magnitude (in dB) vs frequency
semilogx(omega, mag, 'b')
hold on

% Plot phase (in degrees) vs frequency (optional)
semilogx(omega, phase, 'r')

% Label the plot
xlabel('Frequency (rad/s)')
ylabel('Magnitude (dB)')
title('Bode Plot of Low-Pass Filter')
legend('Magnitude', 'Phase')  % For phase plot
grid on

hold off;  % Clear hold to plot multiple bode plots
