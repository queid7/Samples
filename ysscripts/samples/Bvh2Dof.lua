package.path=package.path..";../Samples/scripts/RigidBodyWin/motionFormatConversion/?.lua" --;"..package.path
require("RetargetDOFtoVRML")

local wrlpath, bvhpath, dofpath
do
	---- g25*
	wrlpath = '../Resource/motion/opensim/gait2592_modified.wrl'

--	bvhpath = '../Resource/motion/opensim/g25_gait_repeat.bvh'
--	dofpath = '../Resource/motion/opensim/gait25_gait_repeat.dof'

--	bvhpath = '../Resource/motion/opensim/dcjo_test_bvh_data/Trial001_dcjo.bvh'
	bvhpath = '../Resource/motion/opensim/g25_gait_repeat.bvh'
	dofpath = '../Resource/motion/opensim/gait25_gait_repeat_tutorial.dof'

	--bvhpath = '../Resource/motion/opensim/orig_bvh/soldier_repeat.bvh'
	--dofpath = '../Resource/motion/opensim/dcjo_bvh/soldier_repeat.dof'

--	bvhpath = '../Resource/motion/opensim/dcjo_bvh/g2592_dcjo_16_16_walk.bvh'
--	bvhpath = '../Resource/motion/opensim/dcjo_bvh/g2592_dcjo_16_16_walk_modified.bvh'
--	dofpath = '../Resource/motion/opensim/dcjo_bvh/g2592_dcjo_16_16_walk.dof'

	--bvhpath = '../Resource/motion/opensim/g25_gait110_repeat.bvh'
	--dofpath = '../Resource/motion/opensim/gait25_gait110_repeat.dof'
	--bvhpath = '../Resource/motion/opensim/g25_gait120_repeat.bvh'
	--dofpath = '../Resource/motion/opensim/gait25_gait120_repeat.dof'
	--bvhpath = '../Resource/motion/opensim/g25_gait130_repeat.bvh'
	--dofpath = '../Resource/motion/opensim/gait25_gait130_repeat.dof'
	--bvhpath = '../Resource/motion/opensim/g25_gait140_repeat.bvh'
	--dofpath = '../Resource/motion/opensim/gait25_gait140_repeat.dof'
	--bvhpath = '../Resource/motion/opensim/g25_gait160_repeat.bvh'
	--dofpath = '../Resource/motion/opensim/gait25_gait160_repeat.dof'
	--bvhpath = '../Resource/motion/opensim/g25_gait180_repeat.bvh'
	--dofpath = '../Resource/motion/opensim/gait25_gait180_repeat.dof'

	----bvhpath = '../Resource/motion/opensim/gait19_soldier_repeat.bvh'
	----dofpath = '../Resource/motion/opensim/gait25_soldier_repeat.dof'

	----bvhpath = '../Resource/motion/opensim/gait19_tong_repeat.bvh'
	----dofpath = '../Resource/motion/opensim/gait25_tong_repeat.dof'
	
	----bvhpath = '../Resource/motion/opensim/g25_lean_repeat.bvh'
	----dofpath = '../Resource/motion/opensim/g25_lean_repeat.dof'

	----bvhpath = '../Resource/motion/opensim/g25_srun_repeat.bvh'
	----dofpath = '../Resource/motion/opensim/g25_srun_repeat.dof'

	--bvhpath = '../Resource/motion/opensim/g25_frun_repeat.bvh'
	--dofpath = '../Resource/motion/opensim/g25_frun_repeat.dof'

	--bvhpath = '../Resource/motion/opensim/g25_ipfrun_repeat.bvh'
	--dofpath = '../Resource/motion/opensim/g25_ipfrun_repeat.dof'

	--bvhpath = '../Resource/motion/opensim/g25_ipnrun_repeat.bvh'
	--dofpath = '../Resource/motion/opensim/g25_ipnrun_repeat.dof'

end

do
	---- full2*
	--wrlpath = '../Resource/motion/opensim/FullBody2_lee.wrl'

	----bvhpath = '../Resource/motion/opensim/full_soldier_repeat.bvh'
	----dofpath = '../Resource/motion/opensim/full2_soldier_repeat.dof'
	
	----bvhpath = '../Resource/motion/opensim/full_tong_repeat.bvh'
	----dofpath = '../Resource/motion/opensim/full2_tong_repeat.dof'
	
	----bvhpath = '../Resource/motion/opensim/full_lean_repeat.bvh'
	----dofpath = '../Resource/motion/opensim/full2_lean_repeat.dof'
	
	----bvhpath = '../Resource/motion/opensim/full_same_repeat.bvh'
	----dofpath = '../Resource/motion/opensim/full2_same_repeat.dof'

	----bvhpath = '../Resource/motion/opensim/full_ipnrun_repeat.bvh'
	----dofpath = '../Resource/motion/opensim/full2_ipnrun_repeat.dof'
	
	----bvhpath = '../Resource/motion/opensim/full_ipfrun_repeat.bvh'
	----dofpath = '../Resource/motion/opensim/full2_ipfrun_repeat.dof'

	----bvhpath = '../Resource/motion/opensim/full_sit_repeat.bvh'
	----dofpath = '../Resource/motion/opensim/full2_sit_repeat.dof'
	
	----bvhpath = '../Resource/motion/opensim/full2_srun_repeat.bvh'
	----dofpath = '../Resource/motion/opensim/full2_srun_repeat.dof'

	--bvhpath = '../Resource/motion/opensim/full2_frun_repeat.bvh'
	--dofpath = '../Resource/motion/opensim/full2_frun_repeat.dof'

end

-------------------------------------------------
--NEW
do
	------ gait25*
	----wrlpath = '../Resource/motion/opensim/gait2592_modified.wrl'

	----bvhpath = '../Resource/motion/opensim/gait19_06140c.bvh'
	----dofpath = '../Resource/motion/opensim/gait25_06140c.dof'
	
	
	--wrlpath = '../Resource/motion/opensim/gait2592_lank_fixed.wrl'

	--bvhpath = '../Resource/motion/opensim/g25_gait_repeat.bvh'
	--dofpath = '../Resource/motion/opensim/g25lankf_gait.dof'
	
end

targets={"general_bvh"}

Motions.general_bvh=deepCopyTable(Motions.default)
Motions.general_bvh.conversionMethod=conversionMethod_T.useAllLocalAxis
Motions.general_bvh.src_skel_file=bvhpath
Motions.general_bvh.scale=0.01	-- change inch to METER unit system
Motions.general_bvh.out_file=dofpath
Motions.general_bvh.wrl_file=wrlpath
-- motion T-pose direction difference to wrl T-pose direction
--Motions.general_bvh.default_pose_rotation = quater(-3.14/2,vector3(0,1,0)) 
