#!/bin/bash

# define active cores in the original cmp configuration
./gen_mapped_cmp.py ./test/cmp16x16.cmp

# run CMPexplore to dump the power file
./cmpexplore -test ./test/cmp16x16_map.cmp -dump_ptsim_power

cp ptsim_power.txt ../PTsim/
cd ../PTsim

# run simulator to dump the temperature file
./PTsimtest

# convert temperature file into a map
./mygrid_thermal_map.pl cmp16x16.flp  cmp16x16.grid.steady > cmp16x16.svg

# display it
display cmp16x16.svg

cd -

