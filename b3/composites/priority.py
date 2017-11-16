import b3

__all__ = ['Priority']

class Priority(b3.Composite):
    def __init__(self, children=None):
        super(Priority, self).__init__(children)
        self.Name = '*Priority*'

    def tick(self, tick):
        self.Cost = 0
        for node in self.children:
          status = node._execute(tick)
          #Add in cost of selected leaf (requires zero cost for Seq node)
          self.Cost += node.Cost
          if status != b3.FAILURE:
                return status

        return b3.FAILURE
