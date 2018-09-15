def log_info(message, args = []):
  print()
  print(message if len(args) == 0 else message.format(args))

log_info('importing libs')

import OMPython
from OMPython import OMCSessionZMQ
import pandas as pd
# from matplotlib import pyplot as plt

log_info('starting omc session')
omc = OMCSessionZMQ()


log_info('going to tmp dir')
omc.sendExpression('cd()')
omc.sendExpression('cd("tmp")')
omc.sendExpression('cd()')

log_info('loading MSL')
omc.sendExpression('loadModel(Modelica)')

log_info('initializing omni lib paths')
root = '/home/vf/om/models/omni/thin/MBS'
path_MBS = root + '/package.mo'
path_MBS_Basics = root + '/Basics.mo'
paths = [
    path_MBS,
    path_MBS_Basics,
]
print(paths)

log_info('loading MBS')
omc.sendExpression('loadFile("' + path_MBS + '")')

log_info('finished')
