#
# Put your Job commands here.
#
#------------------------------------------------

echo " "
echo " "
echo "Job started on `hostname` at `date`"

#echo $NSLOTS
#cat $TMP/machines

cd ../../work
#/usr/mpi/gcc/openmpi-1.4.3/bin/mpiexec -machinefile ../Samples/console/machines_test -np 8 ./console_mpi MovingWindowOptimizerMPIMuscle.lua ../Samples/ysscripts/samples/
/usr/bin/mpiexec -machinefile ../Samples/console/mrlmachines -np 6 ./console_mpi MovingWindowOptimizerMPIMuscle.lua ../Samples/ysscripts/samples/

echo " "
echo "Job Ended at `date`"
echo " "

#------------------------------------------------

