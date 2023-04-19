import os
import re

max = 0
max_line = ""

csv_files = [f for f in os.listdir(
    "telemetry/Coax_PRAC") if f.endswith(".csv")]

for file in csv_files:
    with open("telemetry/Coax_PRAC" + "/" + file, "r") as f:
        for line in f:
            if re.search(r"\/Speed", line):
                match = re.search(r"[0-9]*.[0-9]*e?-?[0-9]*$", line)
                speed = float(line[match.start()+1:match.end()])
                # print(speed)
                if speed > max:
                    max = speed
                    max_line = line


print(max * 60)
print(max_line)
