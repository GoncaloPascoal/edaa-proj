
min = [1037, 279, 3728]
max = [1037, 279, 3728]

files = ["df.txt", "pa.txt", "rj.txt"]
for (index, file) in enumerate(files):
    with open(file, "r") as f:
        while (True):
            check = f.readline()
            if check is None:
                break
            check = f.readline()
            if check == '':
                break
            number = int(check)
            if (number < min[index]):
                min[index] = number
            if (number > max[index]):
                max[index] = number

for (index, file) in enumerate(files):
    print(file, min[index], max[index])
