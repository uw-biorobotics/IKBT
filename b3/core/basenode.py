import b3
import uuid


__all__ = ['BaseNode']

class BaseNode(object):
    category = None
    title = None
    description = None

    def __init__(self):
        self.id = str(uuid.uuid1())
        self.title = self.title or self.__class__.__name__
        self.description = self.description or ''
        self.parameters = {}
        self.properties = {}
        # custom members for learning
        self.Name = "--unnamed--"
        self.TEST = "TESTING"
        self.BHdebug = 0
        self.N_leaf_ticks = 0       # only increment if a leaf
        self.N_ticks_all = 0        # number of ticks
        self.N_ticks = 0            # 
        self.N_success = 0
        self.state = 0
        self.N_tik2 = [0, 0, 0, 0]  # number of ticks on each state
        self.N_suc2 = [0, 0, 0, 0]  # prob success conditioned on state
        self.Ps = 0.0               # basic P(success)
        self.P_selector = 0.0       # probability selected by selector S02
        self.Cost = 0               # Cost of ticking this leaf (INT!)
        self.Utility = 0.0	        # U = P/C
        self.Utility_Mode = "RATIO"
   
        self.Tree = {}
        
        
        
    # BH  estimate local success probability    
    def prob(self):
      if self.N_ticks > 0:
        self.Ps = float(self.N_success) / float(self.N_ticks)
        #print "P(s) = ", self.Ps
        return self.Ps
      else:
        return 0.1
    
    # BH get Utility for this node   
    def get_Utility(self):
      if(self.Utility_Mode == "RATIO"):
          if(self.Cost > 0):
            self.Utility =  self.prob() / self.Cost
          else:
            self.Utility =  self.prob() 
            #if(self.BHdebug == 1):
                #print self.Name + '.get_Utility(): Cost: ' + str(self.Cost) +'  P(S): ' + str(self.Ps) +  '  Utility: ' + str(self.Utility)
      if(self.Utility_Mode == "NEG_COST"):
           self.Utility = self.prob() * (-1) * self.Cost
      return self.Utility

    # BH get Utility for this node  CONDITIONED on STATE  
    def get_Utility2(self):     #  for now use on Leafs only      
      if(self.Utility_Mode == "RATIO"):
          if(self.Cost > 0):
              self.Utility =  self.prob_state()[self.state] / self.Cost
          else:
              self.Utility =  self.prob_state()[self.state] 
              #if(self.BHdebug == 1):
                   #print self.Name + '.get_Utility(): Cost: ' + str(self.Cost) +'  P(S): ' + str(self.Ps) +  '  Utility: ' + str(self.Utility)
      if(self.Utility_Mode == "NEG_COST"):
          self.Utility = self.prob_state()[self.state] * (-1) * self.Cost
      return self.Utility

    # BH 
    def get_state(self, bb):    # update your own state -- need to overlay this
        print(self.Name , ": trying to update external sensing state -- not yet defined")
        quit()
        
    # BH  estimate local success probability    
    def prob_state(self):
        p = [0.0,0.0,0.0,0.0]
        for i in range(0,len(p)):
           if self.N_tik2[i] > 0:
             p[i] = float(self.N_suc2[i]) / float(self.N_tik2[i]) 
           else:
             p[i] = 0.5
        if(0):
            s = "{:4.2} {:4.2} {:4.2} {:4.2} ".format(p[0],p[1],p[2],p[3])
            print(self.Name,"prob_state(): ",s)
            s = "{:4} {:4} {:4} {:4} ".format(self.N_suc2[0],self.N_suc2[1],self.N_suc2[2],self.N_suc2[3])
            print(self.Name,"N Success:    ",s)
            s = "{:4} {:4} {:4} {:4} ".format(self.N_tik2[0],self.N_tik2[1],self.N_tik2[2],self.N_tik2[3])
            print(self.Name,"N tik2:       ",s)
        #print self.Name, p
        return p
      
    # BH reset the probability counters
    def p_reset(self):
      self.N_ticks = 0
      self.N_ticks_all = 0
      self.N_success = 0
      self.Ps = 0
      self.N_tik2 = [0.0,0.0,0.0,0.0]
      self.N_suc2 = [0.0,0.0,0.0,0.0]
      
      
      #  report your stats
      
    def report_stats(self):
      print('\n\n',self.Name,'    Statistics')
      print('N_ticks:            ',self.N_ticks)
      print('N_ticks_all:        ',self.N_ticks_all)
      print('N_success:          ',self.N_success)
      print('prob                ',self.prob())
      print('Cost:               ',self.Cost)
      print('Utility:            ',self.get_Utility())
          
    @property
    def name(self):
        return self.__class__.__name__

    def _execute(self, tick):
        self._enter(tick)
        if (not tick.blackboard.get('is_open', tick.tree.id, self.id)):
            self._open(tick)

        status = self._tick(tick)

        if (status != b3.RUNNING):
            self._close(tick)

        self._exit(tick)

        return status

    def _enter(self, tick):
        tick._enter_node(self)
        self.enter(tick)

    def _open(self, tick):
        tick._open_node(self)
        tick.blackboard.set('is_open', True, tick.tree.id, self.id)
        self.open(tick)

    def _tick(self, tick):
        tick._tick_node(self)
        #self.N_ticks += 1   # used to have this but caused double ticks
        if(self.BHdebug == 1):
            print('basenode: ', self.Name, " ticked ")
        #BH count the ticks
        self.N_ticks_all += 1
        status = self.tick(tick)
        #BH count the total cost 
        tick.blackboard.inc('TotalCost',self.Cost)
        
        if(self.BHdebug == 1):
            if(status == b3.SUCCESS):
                print("basenode: ", self.Name, " SUCCESS ")
            elif(status == b3.FAILURE):
                print("basenode: ", self.Name, " FAIL")
        
        # BH keep track of successful ticks
        if(status == b3.SUCCESS):
            self.N_success += 1
            self.N_suc2[self.state]   += 1
            if(tick.tree.log_flag > 0):
                tick.tree.log_file.write('S '+self.Name+'\n')
        if(status == b3.FAILURE and tick.tree.log_flag > 1):
                tick.tree.log_file.write('F '+self.Name+'\n')

        return status

    def _close(self, tick):
        tick._close_node(self)
        tick.blackboard.set('is_open', False, tick.tree.id, self.id)
        self.close(tick)

    def _exit(self, tick):
        tick._exit_node(self)
        self.exit(tick)

    def enter(self, tick): pass
    def open(self, tick): pass
    def tick(self, tick): pass
    def close(self, tick): pass
    def exit(self, tick): pass
