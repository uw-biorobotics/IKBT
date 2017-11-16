import b3

__all__ = ['Sequence']

class Sequence(b3.Composite):

    def __init__(self, children=None):
        super(Sequence, self).__init__(children)
        self.Name = '*Sequence*'

    def tick(self, tick): 
        tmpCost = 0    
        for node in self.children:
            status = node._execute(tick)
            #Add in cost of selected leaf (requires zero cost for Seq node)
            #print node.Name + ':  Cost: '+str(node.Cost)
            tmpCost += node.Cost
            if status != b3.SUCCESS:
                self.Cost = tmpCost
                return status
        self.Cost = tmpCost
        return b3.SUCCESS
