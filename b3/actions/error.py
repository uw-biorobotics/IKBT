import b3

__all__ = ['Error']

class Error(b3.Action):
    def tick(self, tick):
        return b3.ERROR;