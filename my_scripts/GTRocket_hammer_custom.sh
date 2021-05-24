BENCHMARK=$1
GEM5_DIR=/home/grads/t/tapojyoti.mandal/gem5_garnet2 
OUT_DIR=/home/grads/t/tapojyoti.mandal/gem5_garnet2/my_outdir/$BENCHMARK  
SPEC_DIR=/home/grads/t/tapojyoti.mandal/spec2006

if [[ "$BENCHMARK" == "perlbench" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/400.perlbench/run/run_base_test_amd64-m64-gcc43-nn.0000
fi
if [[ "$BENCHMARK" == "bzip2" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/401.bzip2/run/run_base_test_amd64-m64-gcc43-nn.0000
fi
if [[ "$BENCHMARK" == "gcc" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/403.gcc/run/run_base_test_amd64-m64-gcc43-nn.0000
fi
if [[ "$BENCHMARK" == "bwaves" ]]; then #Looks like not compiling
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/410.bzip2/run/run_base_test_amd64-m64-gcc43-nn.0000
fi
if [[ "$BENCHMARK" == "gamess" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/416.games/run/run_base_test_amd64-m64-gcc43-nn.0000
fi
if [[ "$BENCHMARK" == "mcf" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/429.mcf/run/run_base_test_amd64-m64-gcc43-nn.0000
fi
if [[ "$BENCHMARK" == "milc" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/433.milc/run/run_base_test_amd64-m64-gcc43-nn.0000
fi
if [[ "$BENCHMARK" == "zeusmp" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/434.zeusmp/run/run_base_test_amd64-m64-gcc43-nn.0000
fi
if [[ "$BENCHMARK" == "gromacs" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/435.gromacs/run/run_base_test_amd64-m64-gcc43-nn.0000
fi
if [[ "$BENCHMARK" == "cactusADM" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/436.cactusADM/run/run_base_test_amd64-m64-gcc43-nn.0000
fi
if [[ "$BENCHMARK" == "leslie3d" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/437.leslie3d/run/run_base_test_amd64-m64-gcc43-nn.0000
fi
if [[ "$BENCHMARK" == "namd" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/444.namd/run/run_base_test_amd64-m64-gcc43-nn.0000
fi
if [[ "$BENCHMARK" == "gobmk" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/445.gobmk/run/run_base_test_amd64-m64-gcc43-nn.0000
fi
if [[ "$BENCHMARK" == "dealII" ]]; then # DOES NOT WORK
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/447.dealII/run/run_base_test_amd64-m64-gcc43-nn.0000
fi
if [[ "$BENCHMARK" == "soplex" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/450.soplex/run/run_base_test_amd64-m64-gcc43-nn.0000
fi
if [[ "$BENCHMARK" == "povray" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/453.povray/run/run_base_test_amd64-m64-gcc43-nn.0000
fi
if [[ "$BENCHMARK" == "calculix" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/454.calculix/run/run_base_test_amd64-m64-gcc43-nn.0000
fi
if [[ "$BENCHMARK" == "hmmer" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/456.hmmer/run/run_base_test_amd64-m64-gcc43-nn.0000
fi
if [[ "$BENCHMARK" == "sjeng" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/458.sjeng/run/run_base_test_amd64-m64-gcc43-nn.0000
fi
if [[ "$BENCHMARK" == "GemsFDTD" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/459.GemsFDTD/run/run_base_test_amd64-m64-gcc43-nn.0000
fi
if [[ "$BENCHMARK" == "libquantum" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/462.libquantum/run/run_base_test_amd64-m64-gcc43-nn.0000
fi
if [[ "$BENCHMARK" == "h264ref" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/464.h264ref/run/run_base_test_amd64-m64-gcc43-nn.0000
fi
if [[ "$BENCHMARK" == "tonto" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/465.tonto/run/run_base_test_amd64-m64-gcc43-nn.0000
fi
if [[ "$BENCHMARK" == "lbm" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/470.lbm/run/run_base_test_amd64-m64-gcc43-nn.0000
fi
if [[ "$BENCHMARK" == "omnetpp" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/471.omnetpp/run/run_base_test_amd64-m64-gcc43-nn.0000
fi
if [[ "$BENCHMARK" == "astar" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/473.astar/run/run_base_test_amd64-m64-gcc43-nn.0000
fi
if [[ "$BENCHMARK" == "wrf" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/481.wrf/run/run_base_test_amd64-m64-gcc43-nn.0000
fi
if [[ "$BENCHMARK" == "sphinx3" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/482.sphinx3/run/run_base_test_amd64-m64-gcc43-nn.0000
fi
if [[ "$BENCHMARK" == "xalancbmk" ]]; then # DOES NOT WORK
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/483.xalancbmk/run/run_base_test_amd64-m64-gcc43-nn.0000
fi
if [[ "$BENCHMARK" == "specrand_i" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/998.specrand/run/run_base_test_amd64-m64-gcc43-nn.0000
fi
if [[ "$BENCHMARK" == "specrand_f" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/999.specrand/run/run_base_test_amd64-m64-gcc43-nn.0000
fi


NUM_CPUS=64
NUM_CHIPLETS=8


#--debug-flags=RubySNI 

#cd $RUN_DIR

$GEM5_DIR/build/X86_MOESI_hammer/gem5.opt \
-d $OUT_DIR \
$GEM5_DIR/configs/example/se.py \
--cpu-type TimingSimpleCPU \
--num-cpus=${NUM_CPUS} \
--l1d_size=64kB --l1i_size=32kB --l1d_assoc=4 \
--num-l2caches=${NUM_CPUS} \
--l2_size=2MB --l2_assoc=8 \
--num-dirs=4 \
--ruby \
--mem-type=DDR3_1600_8x8 \
--mem-size=1024MB \
--num-chiplets=${NUM_CHIPLETS} \
--sni=1 \
--network=garnet2.0 \
--link-width-bits=1024 \
--vcs-per-vnet=4 \
--topology=CHIPS_GTRocket_Mesh_hammer \
-c "m5threads/tests/array_sum" \
-o "3"

# Print Stats
echo
echo "CHIPS Demo: Stats:"
grep "sim_ticks" $OUT_DIR/stats.txt
grep "average_packet_latency" $OUT_DIR/stats.txt

#-c "m5threads/tests/array_sum" \
#-o "3"
#-c "m5threads/tests/test_atomic" \
#-o "3"
#-c "bzip2_base.amd64-m64-gcc43-nn" \
#-o "input.program 5"
#-c "perlbench_base.amd64-m64-gcc43-nn" \
#-o "-I./lib attrs.pl"
