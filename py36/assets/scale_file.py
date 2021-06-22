import os

from numpy import float64

asset_dir = os.path.dirname(os.path.abspath(__file__))
file = os.path.join(asset_dir, "Vive_Tracker.obj")
new_file = os.path.join(asset_dir, "Vive_Tracker_meter.obj")
new_file = open(new_file, "w")

def isDigit(x):
    try:
        float(x)
        return True
    except ValueError:
        return False

with open(file,"r") as f:
    lines = f.readlines()
    for idx, line in enumerate(lines):
        # if idx>10:
        #     break
        if line[0]=="v":
            words = line[:-2].split(" ")
            print(words)
            new_line = []
            for word in words:
                if isDigit(word):
                    # print("here")
                    new_line += [str(float64(word)*0.00254)]
                else:
                    new_line += [word]
            new_file.write(" ".join(new_line)+"\n")
        else:
            new_file.write(line)

new_file.close()