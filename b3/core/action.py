import b3

__all__ = ['Action']


class Action(b3.BaseNode):
    category = b3.ACTION

    def __init__(self):
        super(Action, self).__init__()
        
