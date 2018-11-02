import time
import serial


def record_data():
    sample_period = 0.1
    prev_time = 0

    data_error = False  # zero data, duplicate data, or missing one of lat/long, ie. len(data) != 2

    port_name = '/dev/tty.usbmodem1411'
    ser = serial.Serial(port_name, 115200)  # serial port config
    f = open('serial_output.txt', 'w')

    ready = False

    if not ready:
        print("-----\nFlushing zeros...\n-----")

    while not ready:  # waits for GPS to output non-zero values
        try:
            # attempt reading, may run into error due to bad data
            reading = str(ser.readline()).split(",")

            if len(reading) == 3:
                reading[0] = reading[0].split("'")[-1]
                reading[2] = reading[2].split("\\")[0]

            if reading[0][0] != '0' and reading[1][0] != '0' and len(
                    reading) == 3:  # non-zero and two coordinate values
                print("-----\nBeginning recording...\n-----")
                start_time = time.time() - 0.1
                ready = True
            else:
                print(str(reading))
        except IndexError:
            pass
    try:
        while ready:
            current_time = round(time.time() - start_time, 1)

            reading = str(ser.readline()).split(",")
            if len(reading) == 3:
                reading[0] = reading[0].split("'")[-1]
                reading[2] = reading[2].split("\\")[0]
            else:
                data_error = True
            try:
                if reading[0][0] == '0' or reading[1][0] == '0':  # check for zeros
                    print("Zeros detected. Ignoring reading and awaiting non-zero data...")
                    data_error = True
            except IndexError:  # will call if there aren't two data values for lat/long or if the data is blank
                data_error = True

            print("Previous time: " + str(prev_time))
            print("Current time: " + str(current_time))
            print(reading)

            # check sequence of readings
            if round(current_time - prev_time, 1) > 0.1 and not data_error:
                print("Missing " + str(round((current_time - prev_time) * 10) - 1) + " readings of data...")
                print("Back-filling data with most recently obtained value...")
                for i in range(0, int(round((current_time - prev_time) * 10) - 1)):
                    f.write(str(round(prev_time + (i + 1) / 10.0, 1)) + "," + str(reading[0]) + "," + str(
                        reading[1]) + "," + str(reading[2]) + "\n")

            if current_time == prev_time:
                print("Duplicated time reading. Sampling stream again...")
                data_error = True

            # check if data is corrupt
            if not data_error:
                f.write(str(current_time) + "," + str(reading[0]) + "," + str(reading[1]) + "," + str(reading[2]) + "\n")
                prev_time = round(current_time, 1)
            else:  # reset flags
                data_error = False

            print("-----")
            time.sleep(sample_period - ((time.time() - start_time) % sample_period))
    except (KeyboardInterrupt, ValueError):
        f.close()
        print("-----\nProgram Terminated...\n-----")


def playback_data():
    print("-----\nBeginning Playback...\n-----")
    f = open('serial_output.txt', 'r')
    start_time = time.time()-0.1
    prev_time = 0

    current_location = [0, 0]
    current_direction = 0

    try:
        while True:  # main program loop ----------------------------------------------------(60Hz) OpenCV frame
            current_time = round(time.time() - start_time, 1)

            if current_time != prev_time:  # ------------------------------------------------(10Hz) Update serial data
                data = f.readline().split(",")
                current_location = [float(data[1]), float(data[2])]
                current_direction = float(data[3])
                print("system_time: " + str(current_time))
                # print("data_time: " + data[0])
                if current_time - float(data[0]) > 0:  # data_time is lagging, therefore advance read line f'n
                    while current_time - float(data[0]) > 0:
                        # print("Advancing line until current time is found...")
                        data = f.readline().split(",")
                        print("new_data_time: " + data[0])
                #
                print("current_location: " + str(current_location))
                print("current_direction: " + str(current_direction))
                print("-----")
                # time.sleep(0.5)  # testing lagging resiliency
            prev_time = current_time * 1
    except KeyboardInterrupt:
        f.close()
        print("-----\nProgram Terminated by User...\n-----")
    except IndexError:
        f.close()
        print("-----\nReached end of Recording...\n-----")


# record_data()
playback_data()
