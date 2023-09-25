import subprocess
import datetime
import os

date = datetime.datetime.now()
date_num = [date.month, date.day, date.hour, date.minute, date.second]
date_str = []
for i in date_num:
    if i < 10:
        date_str.append(str(0) + str(i))
    else:
        date_str.append(str(i))

save_dir = os.path.join("/home/amabe/rosbag/d405", date_str[0] + "-" + date_str[1])

if not os.path.isdir(save_dir): 
    os.mkdir(save_dir)

file_name = date_str[0] + "-" + date_str[1] + "-" + date_str[2] + "-" + date_str[3] + "-" + date_str[4] + ".bag"
rosbag = os.path.join(save_dir, file_name)

command = ["roslaunch", "multi_device_view", "rosbag_record_d405.launch","rosbag:=" + rosbag]
print(command)
result = subprocess.run(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
output = result.stdout
error = result.stderr

if output:
    print("SUCCESS!")
    print(output)

if error:
    print("ERROR!")
    print(error)
