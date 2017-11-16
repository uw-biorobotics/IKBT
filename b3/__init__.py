VERSION   = '0.1.0'

SUCCESS   = 1
FAILURE   = 2
RUNNING   = 3
ERROR     = 4

COMPOSITE = 'composite'
DECORATOR = 'decorator'
ACTION    = 'action'
CONDITION = 'condition'

# CORE
from b3.core.tick import Tick
from b3.core.basenode import BaseNode
from b3.core.blackboard import Blackboard
from b3.core.behaviortree import BehaviorTree
from b3.core.composite import Composite
from b3.core.decorator import Decorator
from b3.core.action import Action
from b3.core.condition import Condition

# COMPOSITES
from b3.composites.sequence import Sequence
from b3.composites.priority import Priority
from b3.composites.mempriority import MemPriority
from b3.composites.memsequence import MemSequence
from b3.composites.ornode import OrNode

# ACTIONS
from b3.actions.succeeder import Succeeder
from b3.actions.failer import Failer
from b3.actions.runner import Runner
from b3.actions.error import Error
from b3.actions.wait import Wait

# DECORATORS
from b3.decorators.inverter import Inverter
from b3.decorators.limiter import Limiter
from b3.decorators.maxtime import MaxTime
from b3.decorators.repeater import Repeater
from b3.decorators.repeatuntilfailure import RepeatUntilFailure
from b3.decorators.repeatuntilsuccess import RepeatUntilSuccess

