import time

class Rate(object):
    """
    Convenience class for sleeping in a loop at a specified rate
    """
    
    def __init__(self, hz, reset=False):
        """
        Constructor.
        @param hz: hz rate to determine sleeping
        @type  hz: float
        @param reset: if True, timer is reset when rostime moved backward. [default: False]
        @type  reset: bool
        """
        # #1403
        self.last_time = time.time()
        self.sleep_dur = 1.0/hz
        self._reset = reset

    def _remaining(self, curr_time):
        """
        Calculate the time remaining for rate to sleep.
        @param curr_time: current time
        @type  curr_time: L{Time}
        @return: time remaining
        @rtype: L{Time}
        """
        # detect time jumping backwards
        if self.last_time > curr_time:
            self.last_time = curr_time

        # calculate remaining time
        elapsed = curr_time - self.last_time
        return self.sleep_dur - elapsed

    def remaining(self):
        """
        Return the time remaining for rate to sleep.
        @return: time remaining
        @rtype: L{Time}
        """
        curr_time = time.time()
        return self._remaining(curr_time)

    def sleep(self):
        """
        Attempt sleep at the specified rate. sleep() takes into
        account the time elapsed since the last successful
        sleep().
        
        @raise ROSInterruptException: if ROS shutdown occurs before
        sleep completes
        @raise ROSTimeMovedBackwardsException: if ROS time is set
        backwards
        """
        curr_time = time.time()
        remaining_time = self._remaining(curr_time)
        if (remaining_time > 0):
            time.sleep(remaining_time)
        self.last_time = self.last_time + self.sleep_dur

        # detect time jumping forwards, as well as loops that are
        # inherently too slow
        if curr_time - self.last_time > self.sleep_dur * 2:
            self.last_time = curr_time