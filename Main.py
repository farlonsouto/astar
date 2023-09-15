from Map import *
from Algorithm import *

bothMaps = Map_Obj(1)
numMap = bothMaps.get_maps()[0]
stringMap = bothMaps.get_maps()[1]
nodeMap = []

# loads both the numeric and the string array data into the Node map (2D array)
for i in range(len(numMap)):
    nodeArray = []
    for j in range(len(numMap[i])):
        value = stringMap[i][j]
        num = numMap[i][j]
        nodeArray.append(Node((i, j), None, num, num > 0, value == 'S', value == 'G'))
    nodeMap.append(nodeArray)



