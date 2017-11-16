import b3
import random

__all__ = ['RandomNodeBH']

class RandomNode(b3.Action):
    def tick(self, tick):
        x = random.random()
        if(x > 0.5):
	  return b3.SUCCESS
	elseif (0.5 < x < 0.75)
	  return b3.RUNNING
	else
	  return b3.FAILURE
	
	
	