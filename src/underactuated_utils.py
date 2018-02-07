import os

def findResource(filename):
   return os.path.join(os.path.dirname(__file__), filename)