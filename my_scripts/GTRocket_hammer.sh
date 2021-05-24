BENCHMARK=$1
GEM5_DIR=/home/grads/t/tapojyoti.mandal/SNI_Regressions/gem5_garnet2_perlbench
OUT_DIR=/home/grads/t/tapojyoti.mandal/SNI_Regressions/gem5_garnet2_perlbench/my_outdir/$BENCHMARK  
SPEC_DIR=/home/grads/t/tapojyoti.mandal/SNI_Regressions/spec2006

if [[ "$BENCHMARK" == "perlbench" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/400.perlbench/run/run_base_ref_amd64-m64-gcc43-nn.0000
    EXECUTABLE="perlbench_base.amd64-m64-gcc43-nn"
    INPUT="-I./lib checkspam.pl 2500 5 25 11 150 1 1 1 1"
fi
if [[ "$BENCHMARK" == "bzip2" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/401.bzip2/run/run_base_ref_amd64-m64-gcc43-nn.0000
    EXECUTABLE="bzip2_base.amd64-m64-gcc43-nn" 
    INPUT="input.source 280"
fi
if [[ "$BENCHMARK" == "gcc" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/403.gcc/run/run_base_ref_amd64-m64-gcc43-nn.0000
    EXECUTABLE="gcc_base.amd64-m64-gcc43-nn"
    INPUT="166.i -o 166.s" 
fi
if [[ "$BENCHMARK" == "bwaves" ]]; then 
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/410.bzip2/run/run_base_ref_amd64-m64-gcc43-nn.0000
    EXECUTABLE="bwaves_base.amd64-m64-gcc43-nn" 
    INPUT="" 
fi
if [[ "$BENCHMARK" == "gamess" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/416.games/run/run_base_ref_amd64-m64-gcc43-nn.0000
    EXECUTABLE="gamess_base.amd64-m64-gcc43-nn"  
    INPUT="cytosine.2.config"
fi
if [[ "$BENCHMARK" == "mcf" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/429.mcf/run/run_base_ref_amd64-m64-gcc43-nn.0000
    EXECUTABLE="mcf_base.amd64-m64-gcc43-nn"   
    INPUT="inp.in"  
fi
if [[ "$BENCHMARK" == "milc" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/433.milc/run/run_base_ref_amd64-m64-gcc43-nn.0000
    EXECUTABLE="milc_base.amd64-m64-gcc43-nn"    
    INPUT="su3imp.in" 
fi
if [[ "$BENCHMARK" == "zeusmp" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/434.zeusmp/run/run_base_ref_amd64-m64-gcc43-nn.0000
    EXECUTABLE="zeusmp_base.amd64-m64-gcc43-nn"     
    INPUT="" 
fi
if [[ "$BENCHMARK" == "gromacs" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/435.gromacs/run/run_base_ref_amd64-m64-gcc43-nn.0000
    EXECUTABLE="gromacs_base.amd64-m64-gcc43-nn"      
    INPUT="-silent -deffnm gromacs -nice 0" 
fi
if [[ "$BENCHMARK" == "cactusADM" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/436.cactusADM/run/run_base_ref_amd64-m64-gcc43-nn.0000
    EXECUTABLE="cactusADM_base.amd64-m64-gcc43-nn"        
    INPUT="benchADM.par"
fi
if [[ "$BENCHMARK" == "leslie3d" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/437.leslie3d/run/run_base_ref_amd64-m64-gcc43-nn.0000
    EXECUTABLE="leslie3d_base.amd64-m64-gcc43-nn"        
    INPUT="" 
fi
if [[ "$BENCHMARK" == "namd" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/444.namd/run/run_base_ref_amd64-m64-gcc43-nn.0000
    EXECUTABLE="namd_base.amd64-m64-gcc43-nn"          
    INPUT="--input namd.input --output namd.out --iterations 38"
fi
if [[ "$BENCHMARK" == "gobmk" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/445.gobmk/run/run_base_ref_amd64-m64-gcc43-nn.0000
    EXECUTABLE="gobmk_base.amd64-m64-gcc43-nn"           
    INPUT="--quiet --mode gtp"
fi
#if [[ "$BENCHMARK" == "dealII" ]]; then # DOES NOT WORK
#    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/447.dealII/run/run_base_ref_amd64-m64-gcc43-nn.0000
#    EXECUTABLE="dealII_base.amd64-m64-gcc43-nn"          
#    INPUT="" 
#fi
if [[ "$BENCHMARK" == "soplex" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/450.soplex/run/run_base_ref_amd64-m64-gcc43-nn.0000
    EXECUTABLE="soplex_base.amd64-m64-gcc43-nn"            
    INPUT="-m45000 pds-50.mps"
fi
if [[ "$BENCHMARK" == "povray" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/453.povray/run/run_base_ref_amd64-m64-gcc43-nn.0000
    EXECUTABLE="povray_base.amd64-m64-gcc43-nn"            
    INPUT="SPEC-benchmark-ref.ini" 
fi
if [[ "$BENCHMARK" == "calculix" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/454.calculix/run/run_base_ref_amd64-m64-gcc43-nn.0000
    EXECUTABLE="calculix_base.amd64-m64-gcc43-nn"             
    INPUT="-i hyperviscoplastic"   
fi
if [[ "$BENCHMARK" == "hmmer" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/456.hmmer/run/run_base_ref_amd64-m64-gcc43-nn.0000
    EXECUTABLE="hmmer_base.amd64-m64-gcc43-nn"              
    INPUT="nph3.hmm swiss41" 
fi
if [[ "$BENCHMARK" == "sjeng" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/458.sjeng/run/run_base_ref_amd64-m64-gcc43-nn.0000
    EXECUTABLE="sjeng_base.amd64-m64-gcc43-nn"               
    INPUT="ref.txt" 
fi
if [[ "$BENCHMARK" == "GemsFDTD" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/459.GemsFDTD/run/run_base_ref_amd64-m64-gcc43-nn.0000
    EXECUTABLE="GemsFDTD_base.amd64-m64-gcc43-nn"               
    INPUT="" 
fi
if [[ "$BENCHMARK" == "libquantum" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/462.libquantum/run/run_base_ref_amd64-m64-gcc43-nn.0000
    EXECUTABLE="libquantum_base.amd64-m64-gcc43-nn"                
    INPUT="1397 8" 
fi
if [[ "$BENCHMARK" == "h264ref" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/464.h264ref/run/run_base_ref_amd64-m64-gcc43-nn.0000
    EXECUTABLE="h264ref_base.amd64-m64-gcc43-nn"                 
    INPUT="-d foreman_ref_encoder_baseline.cfg" 
fi
if [[ "$BENCHMARK" == "tonto" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/465.tonto/run/run_base_ref_amd64-m64-gcc43-nn.0000
    EXECUTABLE="tonto_base.amd64-m64-gcc43-nn"
    INPUT=""    
fi
if [[ "$BENCHMARK" == "lbm" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/470.lbm/run/run_base_ref_amd64-m64-gcc43-nn.0000
    EXECUTABLE="lbm_base.amd64-m64-gcc43-nn"                  
    INPUT="300 reference.dat 0 0 100_100_130_ldc.of"
fi
if [[ "$BENCHMARK" == "omnetpp" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/471.omnetpp/run/run_base_ref_amd64-m64-gcc43-nn.0000
    EXECUTABLE="omnetpp_base.amd64-m64-gcc43-nn"  
    INPUT="omnetpp.ini"    
fi
if [[ "$BENCHMARK" == "astar" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/473.astar/run/run_base_ref_amd64-m64-gcc43-nn.0000
    EXECUTABLE="astar_base.amd64-m64-gcc43-nn"                 
    INPUT="rivers.cfg" 
fi
if [[ "$BENCHMARK" == "wrf" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/481.wrf/run/run_base_ref_amd64-m64-gcc43-nn.0000
    EXECUTABLE="wrf_base.amd64-m64-gcc43-nn"                  
    INPUT="" 
fi
if [[ "$BENCHMARK" == "sphinx3" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/482.sphinx3/run/run_base_ref_amd64-m64-gcc43-nn.0000
    EXECUTABLE="sphinx3_base.amd64-m64-gcc43-nn"                  
    INPUT="ctlfile . args.an4"  
fi
if [[ "$BENCHMARK" == "xalancbmk" ]]; then # ISSUE
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/483.xalancbmk/run/run_base_ref_amd64-m64-gcc43-nn.0000
    EXECUTABLE="xalancbmk_base.amd64-m64-gcc43-nn"                   
    INPUT="-v t5.xml xalanc.xsl"
fi
if [[ "$BENCHMARK" == "specrand_i" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/998.specrand/run/run_base_ref_amd64-m64-gcc43-nn.0000
    EXECUTABLE="specrand_base.amd64-m64-gcc43-nn"                  
    INPUT="1255432124 234923" 
fi
if [[ "$BENCHMARK" == "specrand_f" ]]; then
    RUN_DIR=$SPEC_DIR/benchspec/CPU2006/999.specrand/run/run_base_ref_amd64-m64-gcc43-nn.0000
    EXECUTABLE="specrand_base.amd64-m64-gcc43-nn"                  
    INPUT="1255432124 234923"   
fi

if [[ ! -e $OUT_DIR ]]; then
    mkdir $OUT_DIR
else
    rm -rf $OUT_DIR/*    
fi

NUM_CPUS=64
NUM_CHIPLETS=8

echo "===================================================================="
echo "Benchmark: $BENCHMARK"
#--debug-flags=RubySNI 

cd $RUN_DIR

$GEM5_DIR/build/X86_MOESI_hammer/gem5.opt \
-d $OUT_DIR \
$GEM5_DIR/configs/example/se.py \
--maxinsts=5000000000 \
--cpu-type TimingSimpleCPU \
--num-cpus=${NUM_CPUS} \
--l1d_size=64kB --l1i_size=32kB --l1d_assoc=4 \
--num-l2caches=${NUM_CPUS} \
--l2_size=2MB --l2_assoc=8 \
--num-dirs=4 \
--ruby \
--mem-type=DDR4_2400_8x8 \
--mem-size=4096MB \
--num-chiplets=${NUM_CHIPLETS} \
--sni=0 \
--network=garnet2.0 \
--link-width-bits=1024 \
--vcs-per-vnet=4 \
--topology=CHIPS_GTRocket_Mesh_hammer \
-c $EXECUTABLE \
-o "$INPUT" 

# Print Stats
echo
echo "CHIPS Demo: Stats:"
grep "sim_ticks" $OUT_DIR/stats.txt
grep "average_packet_latency" $OUT_DIR/stats.txt
echo "TEST: $BENCHMARK Finished"
echo "====================================================================="

#-c "m5threads/tests/array_sum" \
#-o "3"
#-c "m5threads/tests/test_atomic" \
#-o "3"
#-c "bzip2_base.amd64-m64-gcc43-nn" \
#-o "input.program 5"
#-c "perlbench_base.amd64-m64-gcc43-nn" \
#-o "-I./lib attrs.pl"
