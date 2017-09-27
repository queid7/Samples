#!/bin/bash
#$ -S /bin/bash
#
# execute the job for the current working directory
#$ -cwd
#
# Set the Parallel Environment and number of procs.
#$ -pe mpich 60
##$ -pe mpich 80
##$ -pe mpich 120 



#
# Put your Job commands here.
#
#------------------------------------------------

echo " "
echo " "
echo "Job started on `hostname` at `date`"

#echo $NSLOTS
#cat $TMP/machines

LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/lib:/home/yslee/lib; export LD_LIBRARY_PATH

cd ../../work
/usr/mpi/gcc/openmpi-1.4.3/bin/mpiexec -machinefile $TMP/machines -np $NSLOTS ./console_mpi MovingWindowOptimizerMPIMuscle.lua ../Samples/ysscripts/samples/

echo " "
echo "Job Ended at `date`"
echo " "

#------------------------------------------------

