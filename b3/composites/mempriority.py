import b3

__all__ = ['MemPriority']

class MemPriority(b3.Composite):
    def __init__(self, children=None):
        super(MemPriority, self).__init__(children)

    def open(self, tick):
        tick.blackboard.set('running_child', 0, tick.tree.id, self.id)

    def tick(self, tick):
        idx = tick.blackboard.get('running_child', tick.tree.id, self.id)

        for i in xrange(idx, len(self.children)):
            node = self.children[i]
            status = node._execute(tick)

            if status != b3.FAILURE:
                if status == b3.RUNNING:
                    tick.blackboard.set('running_child', i, tick.tree.id, self.id)
                return status

        return b3.FAILURE
