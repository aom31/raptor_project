import matplotlib.pyplot as plt

x = []
y = []

data = open('/home/aom/raptor_testcode/src/path_reference/ref_data/path_data1.csv' , 'r')

for line in data:
    lines = [i for i in line.split()]
    x.append(int(lines[0]))
    y.append(float(lines[1]))

print(x,y)
plt.plot(x,y)
plt.title('reference data test for tacking')
plt.xlabel('x-axis')
plt.ylabel('y-axis')
plt.show()