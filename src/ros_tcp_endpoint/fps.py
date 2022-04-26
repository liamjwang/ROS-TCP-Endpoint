import time
import collections

# https://stackoverflow.com/a/54539292/10401512
class FPS:
    def __init__(self,avarageof=50,printevery=None,name="FPS"):
        self.frametimestamps = collections.deque(maxlen=avarageof)
        self.printevery = printevery
        self.last_print_time = time.time()
        self.name = name
        
    def __call__(self):
        self.frametimestamps.append(time.time())
        if self.printevery is not None and time.time() - self.last_print_time > self.printevery:
            print("{}: {}".format(self.name,self))
            self.last_print_time = time.time()
            
            
    def __str__(self):
        if(len(self.frametimestamps) > 1):
            return str(len(self.frametimestamps)/(self.frametimestamps[-1]-self.frametimestamps[0]))
        else:
            return "Unknown"