import b3
# run all the children (regardless of status)
# return success if any child succeeds
# equvalent to logical OR
# added by Dianmu Zhang 06/2017

# Copyright 2017 University of Washington

# Developed by Dianmu Zhang <dianmuz at uw.edu> and 
# Blake Hannaford <blake at uw.edu>
# BioRobotics Lab, University of Washington

# Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

# 1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

# 2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

# 3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.



__all__ = ['OrNode']


class OrNode(b3.Composite):
    def __init__(self, children=None):
        super(OrNode, self).__init__(children)
        self.Name = '*OrNode*'

    def tick(self, tick):
        self.Cost = 0
        status = b3.FAILURE
        
        for node in self.children:
          status_curr = node._execute(tick)
          #Add in cost of selected leaf (requires zero cost for Seq node)
          
          self.Cost += node.Cost
          if status_curr != b3.FAILURE:
                status = b3.SUCCESS

        return status