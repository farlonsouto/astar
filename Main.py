if __name__ == "__main__":
    print("Entry point. Will not runif imported as a module.")

from AStar import *

# print(AStar(TaskMap(1)).aStartInformedSearch())
# print(AStar(TaskMap(2)).aStartInformedSearch())
# print(AStar(TaskMap(3)).aStartInformedSearch())
print(AStar(TaskMap(4)).aStartInformedSearch())
# tic-tac: Clock ticking active
# print(AStar(TaskMap(5)).aStartInformedSearch(True))


