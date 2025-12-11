import os
from snudda import SnuddaInit
import time
from snudda import Snudda


snudda_data = "/Users/peirui/BasalGangliaData/data"
os.environ["SNUDDA_DATA"] = snudda_data

# define and write network
network_path = os.path.join("networks","simple_example_parallel")
si = SnuddaInit(struct_def={}, network_path=network_path, random_seed=123,snudda_data=snudda_data)
si.define_striatum(num_dSPN=20, num_iSPN=20, num_FS=4, num_LTS=0, num_ChIN=0,
                    volume_type="cube", neurons_dir="$DATA/neurons")
si.write_json()

# create network and set up input
snd = Snudda(network_path=network_path)
snd.create_network()
snd.setup_input(input_config=os.path.join("input_config","simple-input-2.json"))

# #run simulation in parallel
# duration = 0.5
# n_cores = 8
# network_path = os.path.join("networks","simple_example_parallel")
# snudda_data = "/Users/peirui/BasalGangliaData/data"
# cmd_str = f"mpiexec -n {n_cores} snudda simulate {network_path} --time {duration}"
# print(cmd_str)
# os.system(cmd_str)
