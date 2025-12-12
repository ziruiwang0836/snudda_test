#!/bin/bash -l
#SBATCH --partition=main
#SBATCH -o log/Simulate-%j-output.txt
#SBATCH -e log/Simulate-%j-error.txt
#SBATCH -t 2:00:00
#SBATCH --time-min=2:00:00
#SBATCH -J Simulate
#SBATCH -A naiss2025-5-309
#SBATCH --nodes=1
#SBATCH --tasks-per-node=10
#SBATCH --mail-type=ALL

srun -n 1 python test_network.py
echo "network creation complete"
# You need to point this as the directory where you created the network in
NETWORK_DIR=/cfs/klemming/home/${USER:0:1}/$USER/snudda_test/test/networks/simple_example_parallel

# This is important, otherwise Snudda import will fail
unset DISPLAY

module load snic-env
module load cray-python
module swap PrgEnv-cray PrgEnv-gnu
module load cray-mpich-abi
module unload cray-libsci

### activate virtual env
conda activate snudda_env
SNUDDA_DIR=/cfs/klemming/home/"${USER:0:1}"/$USER/Snudda

### use basal ganglia data for mod files compiling
# If the BasalGangliaData directory exists, then use that for our data
if [[ -d "/cfs/klemming/home/${USER:0:1}/$USER/BasalGangliaData/data" ]]; then
    export SNUDDA_DATA="/cfs/klemming/home/${USER:0:1}/$USER/BasalGangliaData/data"
    echo "Setting SNUDDA_DATA to $SNUDDA_DATA"
    rm mechanisms
    ln -s $SNUDDA_DATA/neurons/mechanisms/ mechanisms
else
    echo "SNUDDA_DATA environment variable not changed (may be empty): $SNUDDA_DATA"
    rm mechanisms
    ln -s ../../../snudda/data/neurons/mechanisms/
fi

NETWORK_INFO_FILE=$NETWORK_DIR/network-synapses.hdf5
NETWORK_INPUT_FILE=$NETWORK_DIR/input-spikes.hdf5
# NETWORK_VOLTAGE_FILE=$NETWORK_DIR/simulation/voltage-trace-${SLURM_JOBID}.txt

echo "Network dir: "$NETWORK_DIR

srun -n 1 nrnivmodl mechanisms/
echo "mod files compiled"

srun -n $N_WORKERS x86_64/special -mpi -python $SNUDDA_DIR/snudda/simulate/simulate.py $NETWORK_INFO_FILE $NETWORK_INPUT_FILE --time 3.5