import serial
import collections
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import time
import threading

from matplotlib import ticker


# Function to parse the incoming data
def parse_data(data):
    try:
        parts = data.split()
        rpm = float(parts[3])
        comm = float(parts[5])
        return rpm, comm
    except:
        return None

# Function to handle sending commands to the device
def send_commands(ser):
    while not stop_thread:
        command = input("Enter command: ")
        ser.write(f"{command}\n".encode())
        time.sleep(0.1)  # Adjust delay as needed

# Set up the serial connection
ser = serial.Serial('COM6', 115200, timeout=1)

# Lists to store the data for plotting
rpm_data = collections.deque(maxlen=150)
time_data = collections.deque(maxlen=150)
comm_data= collections.deque(maxlen=150)
start_time = time.time()

# Flag to control thread
stop_thread = False

# Start the command sending thread
thread = threading.Thread(target=send_commands, args=(ser,))
thread.start()

# Set up the plot
fig, ax = plt.subplots()
line, = ax.plot([], [], 'r-')

plt2 = fig.add_subplot(111)
plt2.yaxis.tick_right()
comm_line, = plt2.plot([], [], 'g-')
plt2.set_ylim([-255, 255])
ax.set_xlabel('Time (s)')
ax.set_ylabel('RPM')
ax.set_title('RPM over Time')

# Initialize the plot line
def init():
    line.set_data([], [])
    comm_line.set_data([], [])
    return line,

# Update function for animation
def update(frame):
    while (ser.in_waiting > 0):
        data_line = ser.readline().decode().strip()
        if data_line:
            parsed_data = parse_data(data_line)
            if parsed_data:
                rpm, comm = parsed_data
                rpm_data.append(rpm)
                time_data.append(time.time() - start_time)
                comm_data.append(comm)

    line.set_data(time_data, rpm_data)
    ax.relim()
    ax.set_ylim([0, 80])
    comm_line.set_data(time_data, comm_data)
    ax.autoscale_view()
    plt2.relim()
    plt2.autoscale_view()
    ax.xaxis.set_major_locator(ticker.MultipleLocator(1))
    ax.grid(True)
    plt2.xaxis.set_major_locator(ticker.MultipleLocator(1))
    plt2.grid(True)
    fig.canvas.draw()
    return line,

# Create animation
ani = animation.FuncAnimation(fig, update, init_func=init, blit=True, interval=20)

try:
    plt.show()
except KeyboardInterrupt:
    print("Stopped by User")
    stop_thread = True  # Signal the thread to stop
    thread.join()

finally:
    ser.close()
