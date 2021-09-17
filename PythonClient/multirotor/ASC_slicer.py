X = 800000
Y = 896007

with open("1631800648.1502333_LidarSensor1_pointcloud.asc") as myfile:
    excerpt=myfile.readlines()[X:Y]
filename = f"lines_{X}_thru_{Y}.asc"
f = open(filename,'w')
for line in excerpt:
    f.write(line)
f.close()
print("sliced!")