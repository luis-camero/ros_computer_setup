import os
import subprocess

lshw = []
lshw_class = "display"

with subprocess.Popen(["lshw", "-c", lshw_class],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        universal_newlines=True) as proc:
    raw = proc.stdout.read()

entries = raw.strip(" \t\n").split("*-{}".format(lshw_class))[1:]

for i, _ in enumerate(entries):
    entries[i] = [line.strip(" ") for line in entries[i].strip(" \n\t").split("\n")]
    entry_dict = {}
    for line in entries[i]:
        key = line.split(": ")[0]
        val = line.split(": ")[1]
        entry_dict[key] = val
    lshw.append(entry_dict)

for i, entry in enumerate(lshw):
    print("Entry {}: ".format(i))
    for key, val in entry.items():
        print("\t{} : {}".format(key, val))
