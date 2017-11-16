import b3

__all__ = ['Failer']

class Failer(b3.Action):
    def tick(self, tick):
        return b3.FAILURE;