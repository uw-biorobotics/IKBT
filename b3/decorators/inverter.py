import b3

__all__ = ['Inverter']

class Inverter(b3.Decorator):
    def __init__(self,children=None):
        super(Inverter, self).__init__(children)
        self.Name = '*Inverter*'
        
    def tick(self, tick):
        if not self.child:
            return b3.ERROR

        status = self.child._execute(tick)

        if status == b3.SUCCESS:
            status = b3.FAILURE
        elif status == b3.FAILURE:
            status = b3.SUCCESS

        return status
        
