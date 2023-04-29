import matplotlib.pyplot as plt

filename="time_ros2.txt"
sim_time = []
real_time = []
line_max = 1000000
sim_time_offset = 0
real_time_offset = 0


with open(filename, "r") as file:
    new_sim_time = 0
    new_real_time = 0
    actual_sim_time = 0
    actual_real_time = 0

    line_count = 0

    for line in file.readlines():
        if (line_count == line_max):
            break

        elif ("sim") in line:
            new_sim_time = 1

        elif ("real") in line:
            new_real_time = 1

        elif (new_sim_time == 1):
            new_sim_time = 2
            if (line_count < 2):
                sim_time_offset = int(line.strip().removeprefix("sec: "))
            actual_sim_time = int(line.strip().removeprefix("sec: ")) - sim_time_offset

        elif(new_sim_time == 2):
            try:
                actual_sim_time = actual_sim_time + int(line.strip().removeprefix("nsec: "))*1e-9
            except:
                pass
            sim_time.append(actual_sim_time)
            line_count = line_count + 1
            new_sim_time = 0

        elif (new_real_time == 1):
            new_real_time = 2
            if (line_count < 2):
                real_time_offset = int(line.strip().removeprefix("sec: "))
            actual_real_time = int(line.strip().removeprefix("sec: ")) - real_time_offset
            

        elif(new_real_time == 2):
            try:
                actual_real_time = actual_real_time + int(line.strip().removeprefix("nsec: "))*1e-9
            except:
                pass
            real_time.append(actual_real_time)
            line_count = line_count + 1
            new_real_time = 0


plt.plot(real_time, sim_time)
plt.show()
