import serial
import matplotlib.pyplot as plt

ser = serial.Serial('COM13', 115200)

ax_vals = []
ay_vals = []
az_vals = []

plt.ion()
fig, ax = plt.subplots()

line1, = ax.plot([], [], label="Ax")
line2, = ax.plot([], [], label="Ay")
line3, = ax.plot([], [], label="Az")
ax.legend()

while True:
    line = ser.readline().decode().strip()
    data = line.split(",")

    if len(data) == 6:
        ax_vals.append(float(data[0]))  # ax
        ay_vals.append(float(data[1]))  # ay
        az_vals.append(float(data[2]))  # az
        

        line1.set_data(range(len(ax_vals)), ax_vals)
        line2.set_data(range(len(ay_vals)), ay_vals)
        line3.set_data(range(len(az_vals)), az_vals)

        ax.relim()
        ax.autoscale_view()
        plt.pause(0.01)