from os.path import dirname, basename, isfile
import glob

from sys import  path  
path.append('../')   # allow this code to go 'up' to project main dir. 

import ikbtfunctions
import ikbtbasics

modules = glob.glob(dirname(__file__)+"/*.py")
__all__ = [ basename(f)[:-3] for f in modules if isfile(f) and not f.endswith('__init__.py')]


