import b3

__all__ = ['Runner']

class Runner(b3.Action):
    def tick(self, tick):
        return b3.RUNNING;