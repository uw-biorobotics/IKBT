__all__ = ['Blackboard']

class Blackboard(object):
    def __init__(self):
        self._base_memory = {}
        self._tree_memory = {}
        self.set('TotalCost', 0)

    def _get_tree_memory(self, tree_scope):
        if (tree_scope not in self._tree_memory):
            self._tree_memory[tree_scope] = {
                'node_memory': {},
                'open_nodes': []
            }

        return self._tree_memory[tree_scope]

    def _get_node_memory(self, tree_memory, node_scope):
        memory = tree_memory['node_memory']

        if (node_scope not in memory):
            memory[node_scope] = {}

        return memory[node_scope]

    def _get_memory(self, tree_scope, node_scope):
        memory = self._base_memory

        if (tree_scope is not None):
            memory = self._get_tree_memory(tree_scope)

            if (node_scope is not None):
                memory = self._get_node_memory(memory, node_scope)

        return memory

    def set(self, key, value, tree_scope=None, node_scope=None):
        memory = self._get_memory(tree_scope, node_scope)
        memory[key] = value

    def get(self, key, tree_scope=None, node_scope=None):
        memory = self._get_memory(tree_scope, node_scope)
        return memory.get(key)
 
    #BH make it easier to increment a BB value
    def inc(self, key, value, tree_scope=None, node_scope=None):
        memory = self._get_memory(tree_scope, node_scope)
        a = 5
        if (type(a) == type(memory[key])):
           memory[key] += value
        else:
	  print type(memory[key])
	  print type(a)
	  print "Blackboard increment error - must be an int"
	  quit()
    
      