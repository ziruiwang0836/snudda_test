#!/bin/bash

rm -r .ipython

export IPYTHONDIR="`pwd`/.ipython"
export IPYTHON_PROFILE=default

# If the BasalGangliaData directory exists, then use that for our data                      
if [[ -d "/Users/peirui/BasalGangliaData/data" ]]; then
    export SNUDDA_DATA="/Users/peirui/BasalGangliaData/data"
    echo "Setting SNUDDA_DATA to $SNUDDA_DATA"
else
    echo "SNUDDA_DATA environment variable not changed (may be empty): $SNUDDA_DATA"
fi

ipcluster start --profile=$IPYTHON_PROFILE --ip=127.0.0.1 &

sleep 20

echo "Increase nTrials from 2 to a higher number for a proper optimisation"
python optimise_synapses_full.py ../data/synapses/example_data/10_MSN12_GBZ_CC_H20.json --synapseParameters ../data/synapses/example_data/M1LH-contra_dSPN.json --compile --nTrials 20

ipcluster stop


python optimise_synapses_full.py ../data/synapses/example_data/10_MSN12_GBZ_CC_H20.json --synapseParameters ../data/synapses/example_data/M1LH-contra_dSPN.json --plot
