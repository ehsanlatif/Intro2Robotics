import pandas as pd
from IPython.display import display
import os
import glob
import sys
  
  
# use glob to get all the csv files 
# in the folder
path = os.getcwd()
pos_est_dir = pd.read_csv(glob.glob(os.path.join(path, sys.argv[1])))
odom_dir = pd.read_csv(glob.glob(os.path.join(path, sys.argv[2])))

