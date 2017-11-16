__all__ = ['Tick']

class Tick(object):
    '''Tick Class.

    A new Tick object is instantiated every tick by BehaviorTree. It is passed
    as parameter to the nodes through the tree during the traversal.

    The role of the Tick class is to store the instances of tree, debug, target
    and blackboard. So, all nodes can access these informations.

    For internal uses, the Tick also is useful to store the open node after the
    tick signal, in order to let `BehaviorTree` to keep track and close them 
    when necessary.

    This class also makes a bridge between nodes and the debug, passing the 
    node state to the debug if the last is provided.
    '''

    def __init__(self, tree=None, target=None, blackboard=None, debug=None):
        '''Constructor.
    
        :param tree: a BehaviorTree instance.
        :param target: a target object.
        :param blackboard: a Blackboard instance.
        :param debug: a debug instance.
        '''
        self.tree = tree
        self.target = target
        self.blackboard = blackboard
        self.debug = debug

        self._open_nodes = []
        self._node_count = 0

    def _enter_node(self, node):
        '''Called when entering a node (called by BaseNode).

        :param node: a node instance.
        '''
        self._node_count += 1
        self._open_nodes.append(node)

    def _open_node(self, node):
        '''Called when opening a node (called by BaseNode).

        :param node: a node instance.
        '''
        pass

    def _tick_node(self, node):
        '''Called when ticking a node (called by BaseNode).
    
        :param node: a node instance.
        '''
        pass

    def _close_node(self, node):
        '''Called when closing a node (called by BaseNode).

        :param node: a node instance.
        '''
        self._open_nodes.pop()

    def _exit_node(self, node):
        '''Called when exiting a node (called by BaseNode).

        :param node: a node instance.
        '''
        pass