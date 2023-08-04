import re
import sys
import os


def main():
    folder = sys.argv[1]
    files = [f for f in os.listdir(folder) if f.endswith(".csv")]

    currents = []

    for file in files:
        enabled = False
        count = 0
        total = 0
        with open(folder + "/" + file, "r") as f:
            for line in f:
                if not enabled and line.endswith('"DS:enabled",true\n'):
                    enabled = True
                elif enabled and line.endswith('"DS:enabled",false\n'):
                    enabled = False

                if not enabled:
                    continue

                if re.search(r"\/Total Current", line):
                    match = re.search(r"[0-9]*$", line)
                    try:
                        current = float(line[match.start() : match.end()])
                        total += current
                        count += 1
                    except Exception:
                        print(match.start(), match.end())
                        print(line)
            if count > 0:
                currents.append(total / count)
            else:
                print("No current data for " + file)

    print(sum(currents) / len(currents))
    print(currents)


if __name__ == "__main__":
    main()
