import time
import collections

# https://stackoverflow.com/a/54539292/10401512
class FPS:
    def __init__(self,avarageof=50,printevery=None,name="FPS"):
        self.frametimestamps = collections.deque(maxlen=avarageof)
        self.printevery = printevery
        self.callcount = 0
    def __call__(self):
        self.frametimestamps.append(time.time())
        self.callcount += 1
        if self.printevery is not None and self.callcount >= self.printevery:
            self.callcount = 0
            print("{}: {}".format(name,self))
    def __str__(self):
        if(len(self.frametimestamps) > 1):
            return len(self.frametimestamps)/(self.frametimestamps[-1]-self.frametimestamps[0])
        else:
            return 0.0