import b3

__all__ = ['Condition']


class Condition(b3.BaseNode):
    category = b3.CONDITION

    def __init__(self):
        super(Condition, self).__init__()
        
