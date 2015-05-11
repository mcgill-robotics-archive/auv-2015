'''
Imports all python files in this directory. The only files here should be 
filters for use with quick test. Each file should define one method named filter, 
taking as input a cv image and returning a cv image. 
'''

import os, glob

modules = glob.glob(os.path.join(os.path.dirname(__file__), "*.py"))
__all__ = [os.path.basename(f)[:-3] for f in modules if not f.endswith("__init__.py")]
from . import *
