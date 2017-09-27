cd ../../../
sshpass -e sftp dcjo@rock.snu.ac.kr << EOF
cd Research/yslee/Samples/scripts/
put yslee/Samples/scripts/common.lua

cd RigidBodyWin/subRoutines
put yslee/Samples/scripts/RigidBodyWin/subRoutines/Optimizer.lua
put yslee/Samples/scripts/RigidBodyWin/subRoutines/OptimizerMPI.lua

cd ../../../QP_controller/lua/IPC_based/
put yslee/Samples/QP_controller/lua/IPC_based/LocoSynthesis.lua
put yslee/Samples/QP_controller/lua/IPC_based/LocoGraph.lua

cd ../subRoutines/
put yslee/Samples/QP_controller/lua/subRoutines/QPservo.lua
put yslee/Samples/QP_controller/lua/subRoutines/CompareChain.lua

cd ../../../ysscripts/lib/
put yslee/Samples/ysscripts/lib/OsModel.lua
put yslee/Samples/ysscripts/lib/QPservo2.lua
put yslee/Samples/ysscripts/lib/LocoSimMuscle.lua
put yslee/Samples/ysscripts/lib/WalkingDevice.lua
put yslee/Samples/ysscripts/lib/utilfunc.lua
put yslee/Samples/ysscripts/lib/Lee2013MuscleDyn.lua

cd WalkingDevices
put yslee/Samples/ysscripts/lib/WalkingDevices/WalkingDeviceBase.lua
put yslee/Samples/ysscripts/lib/WalkingDevices/WalkingDeviceRAA.lua
put yslee/Samples/ysscripts/lib/WalkingDevices/WalkingDeviceCMABase.lua
cd ../

cd ../samples
put yslee/Samples/ysscripts/samples/driverDynamicMuscle.lua
put yslee/Samples/ysscripts/samples/driverDynamicConsoleMuscle.lua
put yslee/Samples/ysscripts/samples/MovingWindowOptimizerMPIMuscle.lua
put yslee/Samples/ysscripts/samples/MovingWindowOptimizerSubroutines4Muscle.lua
put yslee/Samples/ysscripts/samples/useCaseMuscle.lua
put yslee/Samples/ysscripts/samples/useCaseMuscle_gait1956_6steps.lua
put yslee/Samples/ysscripts/samples/useMuscles.lua
put yslee/Samples/ysscripts/samples/useMuscle_g2592_gait.lua
put yslee/Samples/ysscripts/samples/useMuscle_g2592_gait_device.lua

cd ../../../Resource/motion/opensim
put yslee/Resource/motion/opensim/gait2592_modified_tal_path.luamscl

EOF

cd yslee/Samples/yssample1

