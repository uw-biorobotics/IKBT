import b3

__all__ = ['Succeeder']

class Succeeder(b3.Action):
    def tick(self, tick):
        return b3.SUCCESS;