import os
import time
from optimise_synapses_full import OptimiseSynapsesFull
from ipyparallel import Client

os.environ["IPYTHONDIR"] = os.path.join(os.path.abspath(os.getcwd()), ".ipython")
os.environ["IPYTHON_PROFILE"] = "default"
os.system("ipcluster start --n=20 --profile=$IPYTHON_PROFILE --ip=127.0.0.1 --log-level ERROR 2> parallel-log.txt &")
time.sleep(20) # Wait for ipcluster to start

compile_mod = True

data_file = "../example_data/10_MSN12_GBZ_CC_H20.json"
synapse_parameters = "../example_data/M1LH-contra_dSPN.json"


rc = Client(profile="default")   # connects to running ipcluster
d_view = rc[:]                   # all engines
opt_method = "sobol"

log_file_name = os.path.join("logs", f"{os.path.basename(data_file)}-log.txt")

if not os.path.exists("logs/"):
    os.makedirs("logs/")

ly = OptimiseSynapsesFull(snudda_data="../../BasalGangliaData/data",
                          data_file=data_file,
                          synapse_parameter_file=synapse_parameters,
                          synapse_type="glut", d_view=d_view,
                          role="master",
                          log_file_name=log_file_name, 
                          opt_method=opt_method)

ly.parallel_optimise_single_cell(n_trials=5000, post_opt=False)