import b3

__all__ = ['Composite']


class Composite(b3.BaseNode):
    category = b3.COMPOSITE

    def __init__(self, children=None):
        super(Composite, self).__init__()

        self.children = children or []
