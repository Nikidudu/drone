import serial
import matplotlib.pyplot as plt

ser = serial.Serial('COM13', 115200)

ax_vals = []
ay_vals = []
az_vals = []
gx_vals = []
gy_vals = []
gz_vals = []


plt.ion()
fig, ax = plt.subplots()

line1, = ax.plot([], [], label="Gyro", color='blue')
# line2, = ax.plot([], [], label="Ay", color='blue')
line3, = ax.plot([], [], label="Accel", color='red')
# line4, = ax.plot([], [], label="Gx", color='red')
line5, = ax.plot([], [], label="Com", color='green')
# line6, = ax.plot([], [], label="Gz", color='green')

ax.legend()

while True:
    line = ser.readline().decode().strip()
    data = line.split(",")

    ax_vals.append(float(data[0]))  # gyro
    ay_vals.append(float(data[1]))  # gyro
    az_vals.append(float(data[2]))  # accel
    gx_vals.append(float(data[3]))  # accel
    gy_vals.append(float(data[4]))  # com
    gz_vals.append(float(data[5]))  # com

    line1.set_data(range(len(ax_vals)), ax_vals)
    # line2.set_data(range(len(ay_vals)), ay_vals)
    line3.set_data(range(len(az_vals)), az_vals)
    # line4.set_data(range(len(gx_vals)), gx_vals)
    line5.set_data(range(len(gy_vals)), gy_vals)
    # line6.set_data(range(len(gz_vals)), gz_vals)

    ax.relim()  
    ax.autoscale_view() 
    plt.pause(0.01) 