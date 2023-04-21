import os
import re

max = 0
max_line = ""

csv_files = [f for f in os.listdir(
    "telemetry/GCMP_Qual") if f.endswith(".csv")]

for file in csv_files:
    with open("telemetry/GCMP_Qual" + "/" + file, "r") as f:
        for line in f:
            if re.search(r"\/Speed", line):
                match = re.search(r"[0-9]*.[0-9]*e?-?[0-9]*$", line)
                try:
                    speed = float(line[match.start():match.end()])
                    print(speed)
                    if speed > max:
                        max = speed
                        max_line = line
                except Exception:
                    print(match.start(), match.end())
                    print(line)


print(max)
print(max_line)
