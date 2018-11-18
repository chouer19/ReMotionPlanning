import os
from path_msgs.msg import Waypoint

class MapReader:
    def __init__(self,dirName = None):
        self._dir = dirName
        self._path = []
        self._file = -1
        self._id = -1
        pass

    def next(self):
        path = []
        self._file += 1
        inFile = self._dir + str(self._file) + '.waypoints'
        if not os.path.isfile(inFile):
            return []
        f = open(inFile)
        line = f.readline()
        while line:
            contents = line.split(',')
            line = f.readline()
            if len(contents) < 3:
                continue
            path.append(Waypoint(float(contents[0]), float(contents[1]),float(contents[2]),0))
        f.close()
        return path
        pass
