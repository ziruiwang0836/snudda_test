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

# Total number of MPI ranks/tasks
export N_WORKERS=$SLURM_NTASKS

# This is important, otherwise Snudda import may fail (no GUI on compute nodes)
unset DISPLAY

# Load Dardel modules
module load snic-env
module load cray-python
module swap PrgEnv-cray PrgEnv-gnu
module load cray-mpich-abi
module unload cray-libsci

# Activate your conda env
source ~/.bashrc
conda activate snudda_env

# Snudda base dir
SNUDDA_DIR=/cfs/klemming/home/"${USER:0:1}"/$USER/Snudda

# Go to your working directory (adjust if needed)
cd /cfs/klemming/home/${USER:0:1}/$USER/snudda_test/test

### use basal ganglia data for mod files compiling
if [[ -d "/cfs/klemming/home/${USER:0:1}/$USER/BasalGangliaData/data" ]]; then
    export SNUDDA_DATA="/cfs/klemming/home/${USER:0:1}/$USER/BasalGangliaData/data"
    echo "Setting SNUDDA_DATA to $SNUDDA_DATA"
    rm -rf mech
    ln -s "$SNUDDA_DATA/neurons/mechanisms/" mechanisms
else
    echo "SNUDDA_DATA environment variable not changed (may be empty): $SNUDDA_DATA"
    rm -rf mech
    ln -s ../../../snudda/data/neurons/mechanisms/
fi

# Compile MOD files
srun -n 1 nrnivmodl mechanisms/
echo "mod files compiled"

# Create the network (your Python script)
srun -n 1 python test_network.py
echo "network creation complete"

# Point to the network you just created
NETWORK_DIR=/cfs/klemming/home/${USER:0:1}/$USER/snudda_test/test/networks/simple_example_parallel
NETWORK_INFO_FILE=$NETWORK_DIR/network-synapses.hdf5
NETWORK_INPUT_FILE=$NETWORK_DIR/input-spikes.hdf5

echo "Network dir: $NETWORK_DIR"

# Run the parallel simulation with NEURON special
srun -n $N_WORKERS ./x86_64/special -mpi -python \
    $SNUDDA_DIR/snudda/simulate/simulate.py \
    "$NETWORK_INFO_FILE" "$NETWORK_INPUT_FILE" --time 3.5

echo "simulation complete"
