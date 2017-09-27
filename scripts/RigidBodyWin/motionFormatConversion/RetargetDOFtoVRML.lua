
require("config")
require("common") -- model definitions

-- how to create a VRML model for simulation?
-- 1. run MotToVRML.lua  (lua short.lua mottovrml)
-- See MotToVRML.lua for details

-- simply reuse existing data before the start frame. Conveninent to append a new motion data
startFrame=0
endFrame=10000000
--endFrame=200

--targets={"justin_straight_run", "justin_straight_run_cart"}
--targets={"justin_runf3", "justin_runf3_cart"}
--targets={"gf_scene1"}
--targets={"hyunwoo_full"}
--targets={"gait23"}
--targets={"g25_cmurun"}
targets={"full_cmurun"}
--dbg.startTrace()


conversionMethod_T={
	useAllLocalAxis=1, -- IK. use 3 markers for each joint.
	useLocalOrientation=2, -- Forward kinematics. (all local axes of model and motion should match.)
	useConvertMotionToMotDOFfunc=3, -- Forward kinematics. (all local axes of model and motion should match.)
	
}

-- to maintain a history of conversions
Motions={}
Motions.default={}
Motions.default.src_mot_files={}-- usually src_skel_file contains motions too.
Motions.default.conversionMethod=conversionMethod_T.useAllLocalAxis

--Motions.gait23=deepCopyTable(Motions.default)
----Motions.gait23.src_skel_file="../Resource/motion/opensim/gait23_2geometry.wrl"
--Motions.gait23.src_skel_file="../Resource/motion/opensim/gait23_full_ground_rst.bvh"
----Motions.gait23.src_mot_files={"../Resource/motion/opensim/gait23_full_ground_rst.bvh"}
----Motions.gait23.src_mot_files={"../Resource/motion/opensim/gait23_full_ground_rst.dof"}
--Motions.gait23.scale=1	-- change inch to METER unit system
--Motions.gait23.out_file="../Resource/motion/opensim/gait23_full_ground_rst2.dof"
--Motions.gait23.wrl_file="../Resource/motion/opensim/gait23_2geometry.wrl"

Motions.g25_cmurun=deepCopyTable(Motions.default)
Motions.g25_cmurun.src_skel_file="../Resource/motion/opensim/g25_cmurun_repeat.bvh"
Motions.g25_cmurun.scale=1
Motions.g25_cmurun.out_file="../Resource/motion/opensim/g25_cmurun_repeat.dof"
Motions.g25_cmurun.wrl_file="../Resource/motion/opensim/gait2592_modified.wrl"
Motions.g25_cmurun.upper_markerDistance=2
Motions.g25_cmurun.markerDistance=.2

Motions.full_cmurun=deepCopyTable(Motions.default)
Motions.full_cmurun.src_skel_file="../Resource/motion/opensim/full2_cmurun_repeat.bvh"
Motions.full_cmurun.scale=1
Motions.full_cmurun.out_file="../Resource/motion/opensim/full2_cmurun_repeat.dof"
Motions.full_cmurun.wrl_file="../Resource/motion/opensim/FullBody2_lee.wrl"
Motions.full_cmurun.upper_markerDistance=2
Motions.full_cmurun.markerDistance=.2

Motions.muaythai1=deepCopyTable(Motions.default)
Motions.muaythai1.src_skel_file="../Resource/motion/kickboxer/muaythai/muaythai1.mot"
Motions.muaythai1.scale=0.01	-- change inch to METER unit system
Motions.muaythai1.out_file="../Resource/motion/muaythai1.dof"
Motions.muaythai1.wrl_file="../Resource/motion/muaythai1.wrl"

Motions.muaythai2=deepCopyTable(Motions.default)
Motions.muaythai2.src_skel_file="../Resource/motion/kickboxer/muaythai/muaythai2.mot"
Motions.muaythai2.scale=0.01	-- change inch to METER unit system
Motions.muaythai2.out_file="../Resource/motion/muaythai2.dof"
Motions.muaythai2.wrl_file="../Resource/motion/muaythai2.wrl"

Motions.taekwondo1=deepCopyTable(Motions.default)
Motions.taekwondo1.src_skel_file="../Resource/motion/kickboxer/man1all_nd.mot"
Motions.taekwondo1.scale=0.0254	-- change inch to METER unit system
Motions.taekwondo1.out_file="../Resource/motion/taekwondo1.dof"
Motions.taekwondo1.wrl_file="../Resource/motion/taekwondo1.wrl"

Motions.taekwondo2=deepCopyTable(Motions.default)
Motions.taekwondo2.src_skel_file="../Resource/motion/kickboxer/man2all_nd.mot"
Motions.taekwondo2.scale=0.0254	-- change inch to METER unit system
Motions.taekwondo2.out_file="../Resource/motion/taekwondo2.dof"
Motions.taekwondo2.wrl_file="../Resource/motion/taekwondo2.wrl"

Motions.taekwondo3=deepCopyTable(Motions.default)
Motions.taekwondo3.src_skel_file="../Resource/mocap_manipulation/bvh_files/2/2_skel.bvh"
Motions.taekwondo3.scale=1-- change inch to METER unit system
Motions.taekwondo3.skinScale=1
--Motions.taekwondo3.conversionMethod=conversionMethod_T.useLocalOrientation
--Motions.taekwondo3.conversionMethod=conversionMethod_T. useConvertMotionToMotDOFfunc
Motions.taekwondo3.markerDistance=2
Motions.taekwondo3.out_file="../Resource/mocap_manipulation/bvh_files/2/2_skel.dof"
Motions.taekwondo3.wrl_file="../Resource/mocap_manipulation/bvh_files/2/2_skel.wrl"

Motions.cocktail1=deepCopyTable(Motions.default)
Motions.cocktail1.src_skel_file="../Resource/mocap_manipulation/bvh_files/gf_0515/2/gf_skel2-1.bvh"
Motions.cocktail1.scale=1-- change inch to METER unit system
Motions.cocktail1.skinScale=1
Motions.cocktail1.markerDistance=2
Motions.cocktail1.out_file="../Resource/mocap_manipulation/bvh_files/gf_0515/2/gf_skel2-1.dof"
Motions.cocktail1.wrl_file="../Resource/mocap_manipulation/bvh_files/gf_0515/2/gf_skel2-1.wrl"

Motions.gf_scene1=deepCopyTable(Motions.default)
Motions.gf_scene1.src_skel_file="../Samples/classification/lua/gf/motion/human/skel_nj.bvh"
Motions.gf_scene1.scale=1-- change inch to METER unit system
Motions.gf_scene1.skinScale=1
Motions.gf_scene1.markerDistance=2
Motions.gf_scene1.out_file= "../Samples/classification/lua/gf/motion/human/skel_nj.dof"
Motions.gf_scene1.wrl_file= "../Samples/classification/lua/gf/motion/human/skel_nj.wrl"

Motions.brush=deepCopyTable(Motions.default)
Motions.brush.src_skel_file="../Resource/mocap_manipulation/bvh_files/gf_0516/b_skel.bvh"
Motions.brush.scale=1-- change inch to METER unit system
Motions.brush.skinScale=1
Motions.brush.markerDistance=2
Motions.brush.out_file="../Resource/mocap_manipulation/bvh_files/gf_0516/b_skel.dof"
Motions.brush.wrl_file="../Resource/mocap_manipulation/bvh_files/gf_0516/b_skel.wrl"

Motions.hyunwoo_real=deepCopyTable(Motions.default)
Motions.hyunwoo_real.src_skel_file="../Resource/motion/locomotion_hyunwoo2/locomotion.mot"
Motions.hyunwoo_real.scale=0.01	-- change to METER unit system
Motions.hyunwoo_real.out_file="../Resource/scripts/ui/RigidBodyWin/hyunwoo_real.dof"

Motions.hyunwoo_real_cart=deepCopyTable(Motions.default)
Motions.hyunwoo_real_cart.src_skel_file="../Resource/scripts/ui/RigidBodyWin/hyunwoo_real.wrl"
Motions.hyunwoo_real_cart.src_dof_files={"../Resource/scripts/ui/RigidBodyWin/hyunwoo_real.dof"}
Motions.hyunwoo_real_cart.out_file="../Resource/scripts/ui/RigidBodyWin/hyunwoo_real_cart.dof"
Motions.hyunwoo_real_cart.conversionMethod=conversionMethod_T.useLocalOrientation
Motions.hyunwoo_real_cart.upsample=4

Motions.hyunwoo_full=deepCopyTable(Motions.default)
Motions.hyunwoo_full.wrl_file="../Resource/motion/locomotion_hyunwoo/hyunwoo_full.wrl"
Motions.hyunwoo_full.src_skel_file="../Resource/motion/locomotion_hyunwoo/locomotion_hl.mot"
Motions.hyunwoo_full.scale=0.0254	-- change from inch to METER unit system
Motions.hyunwoo_full.out_file="../Resource/motion/locomotion_hyunwoo/hyunwoo_full.dof"
Motions.hyunwoo_full.markerDistance = 0.05
--Motions.hyunwoo_full.conversionMethod=conversionMethod_T.useConvertMotionToMotDOFfunc

Motions.hyunwoo_full_cart=deepCopyTable(Motions.default)
Motions.hyunwoo_full_cart.src_skel_file="../Resource/scripts/ui/RigidBodyWin/hyunwoo_full.wrl"
Motions.hyunwoo_full_cart.src_dof_files={"../Resource/scripts/ui/RigidBodyWin/hyunwoo_full.dof"}
Motions.hyunwoo_full_cart.out_file="../Resource/scripts/ui/RigidBodyWin/hyunwoo_full_cart.dof"
Motions.hyunwoo_full_cart.conversionMethod=conversionMethod_T.useLocalOrientation
Motions.hyunwoo_full_cart.upsample=4

Motions.hyunwoo_ball_cart=deepCopyTable(Motions.default)
Motions.hyunwoo_ball_cart.src_skel_file="../Resource/scripts/ui/RigidBodyWin/hyunwoo_full_cart.wrl"
Motions.hyunwoo_ball_cart.src_dof_files={"../Resource/scripts/ui/RigidBodyWin/hyunwoo_full_cart.dof"}
Motions.hyunwoo_ball_cart.out_file="../Resource/scripts/ui/RigidBodyWin/hyunwoo_ball_cart.dof"
Motions.hyunwoo_ball_cart.conversionMethod=conversionMethod_T.useLocalOrientation

Motions.justin_jump=deepCopyTable(Motions.default)
Motions.justin_jump.src_skel_file="../Resource/scripts/ui/RigidBodyWin/justin_raw.mot"
Motions.justin_jump.out_file="../Resource/scripts/ui/RigidBodyWin/justin_jump.dof"

Motions.gymnist=deepCopyTable(Motions.default)
Motions.gymnist.src_skel_file="../Resource/scripts/ui/RigidBodyWin/gymnist_sd/gymnist_raw.mot"
Motions.gymnist.out_file="../Resource/scripts/ui/RigidBodyWin/gymnist.dof"
Motions.gymnist.debug_vrml_file="../Resource/scripts/ui/RigidBodyWin/gymnist_mot.wrl"
--Motions.gymnist.src_skel_file="../Resource/scripts/ui/RigidBodyWin/gymnist_sd/gymnist_raw_ROM.mot"
--Motions.gymnist.out_file="../Resource/scripts/ui/RigidBodyWin/gymnist_ROM.dof"

Motions.justin_runf3=deepCopyTable(Motions.default)
Motions.justin_runf3.src_skel_file="../Resource/scripts/ui/RigidBodyWin/justin_runf3_raw.mot"
Motions.justin_runf3.out_file="../Resource/scripts/ui/RigidBodyWin/justin_runf3.dof"


Motions.justin_runf3_cart=deepCopyTable(Motions.default)
Motions.justin_runf3_cart.src_skel_file="../Resource/scripts/ui/RigidBodyWin/justin_runf3.wrl"
Motions.justin_runf3_cart.src_dof_files={"../Resource/scripts/ui/RigidBodyWin/justin_runf3.dof"}
Motions.justin_runf3_cart.out_file="../Resource/scripts/ui/RigidBodyWin/justin_runf3_cart.dof"
Motions.justin_runf3_cart.conversionMethod=conversionMethod_T.useLocalOrientation




Motions.justin_jump_cart=deepCopyTable(Motions.default)
Motions.justin_jump_cart.src_skel_file="../Resource/scripts/ui/RigidBodyWin/justin_jump.wrl"
Motions.justin_jump_cart.src_dof_files={"../Resource/scripts/ui/RigidBodyWin/justin_jump.dof"}
Motions.justin_jump_cart.out_file="../Resource/scripts/ui/RigidBodyWin/justin_jump_cart.dof"
Motions.justin_jump_cart.conversionMethod=conversionMethod_T.useLocalOrientation

Motions.justin_run=deepCopyTable(Motions.default)
Motions.justin_run.src_skel_file="../Resource/scripts/ui/RigidBodyWin/justin_run_raw.mot"
Motions.justin_run.out_file="../Resource/scripts/ui/RigidBodyWin/justin_run.dof"


Motions.justin_run_cart=deepCopyTable(Motions.default)
Motions.justin_run_cart.src_skel_file="../Resource/scripts/ui/RigidBodyWin/justin_run.wrl"
Motions.justin_run_cart.src_dof_files={"../Resource/scripts/ui/RigidBodyWin/justin_run.dof"}
Motions.justin_run_cart.out_file="../Resource/scripts/ui/RigidBodyWin/justin_run_cart.dof"
Motions.justin_run_cart.conversionMethod=conversionMethod_T.useLocalOrientation

Motions.justin_straight_run=deepCopyTable(Motions.default)
Motions.justin_straight_run.src_skel_file="../Resource/scripts/ui/RigidBodyWin/justin_straight_run_raw.mot"
Motions.justin_straight_run.out_file="../Resource/scripts/ui/RigidBodyWin/justin_straight_run.dof"


Motions.justin_straight_run_cart=deepCopyTable(Motions.default)
Motions.justin_straight_run_cart.src_skel_file="../Resource/scripts/ui/RigidBodyWin/justin_straight_run.wrl"
Motions.justin_straight_run_cart.src_dof_files={"../Resource/scripts/ui/RigidBodyWin/justin_straight_run.dof"}
Motions.justin_straight_run_cart.out_file="../Resource/scripts/ui/RigidBodyWin/justin_straight_run_cart.dof"
Motions.justin_straight_run_cart.conversionMethod=conversionMethod_T.useLocalOrientation


-- Motions.justin_straight_run=deepCopyTable(Motions.default)
-- Motions.justin_straight_run.src_skel_file="../Resource/scripts/ui/RigidBodyWin/justin_straight_run_cart.wrl"
-- Motions.justin_straight_run.src_dof_files={"../Resource/scripts/ui/RigidBodyWin/justin_straight_run_cart.dof"}
-- Motions.justin_straight_run.out_file="../Resource/scripts/ui/RigidBodyWin/justin_straight_run.dof"
-- Motions.justin_straight_run.conversionMethod=conversionMethod_T.useLocalOrientation

-- output motion file name will be the same as the input model file!!!

function convert(target, askOverwrite)
	model=model_files[target]-- defined in common.lua
	if type(target)=='string' then
		motion=Motions[target]
	else
		motion=target
	end
	if model==nil then
		model={file_name=motion.wrl_file}
	end
	-- model.file_name = motion.wrl_file
	-- motion = Motions.general_bvh

	local skel1
	local mot1

	if motion.src_dof_files then
		skel1=VRMLloader(motion.src_skel_file)
		mot1=MotionDOFcontainer(skel1.dofInfo, motion.src_dof_files[1])		--dof

		if motion.upsample~=nil then
			mot1:upsample(motion.upsample)
		end

	else
		skel1=MotionLoader.new(motion.src_skel_file, motion.src_mot_files)	--bvh
		mot1=skel1.mMotion
		if motion.debug_vrml_file then
			MotionUtil.exportVRML(mot1, motion.debug_vrml_file, 0, 1000000000)
		end
	end

	conversionMethod=	motion.conversionMethod
	if motion.scale~=nil then
		skel1:scale(motion.scale)
	end

	skel2=MainLib.VRMLloader(model.file_name)		--wrl
	skel2:updateInitialBone()

	-- mot1 is either an instance of Motion or MotionDOFcontainer
	-- both has members : numFrames

	if startFrame~=0 then
		mot2=MotionDOFcontainer(skel2.dofInfo,motion.out_file)
	else	
		mot2=MotionDOFcontainer(skel2.dofInfo)
	end

	reversed=false
	if startFrame>endFrame then
		reversed=true
	end
	
	local numFrame2
	if (not reversed) then
		endFrame=math.min(endFrame, mot1:numFrames())
		numFrame2=endFrame-startFrame
		print(numFrame2)
		mot2:resize(numFrame2+startFrame)
	else
		numFrame2=startFrame-endFrame
	end

local effectors=MotionUtil.Effectors()
local effectorPos=vector3N()

local jointName=vector()
for i=0,skel1:numRotJoint()-1 do
	local nameId=skel1:getBoneByRotJointIndex(i).NameId;
	print("nameID",nameId)
	
	if skel2:getTreeIndexByName(nameId)~=-1 then
		print("if passed")
		jointName:pushBack(nameId)
	end
end

print(jointName)

local countExistingJoint=jointName:size()

print(countExistingJoint)

if conversionMethod==conversionMethod_T.useAllLocalAxis then		--using in Bvh2Dof.lua
	local c=0
	for i=1, countExistingJoint-1 do	-- excluding the root
		local bone=MainLib.VRMLloader.upcast(skel2:getBoneByName(jointName(i)))
		assert(bone~=nil)
		if bone:childHead()==nil then
			-- add SITE markers
			c=c+1
		end
	end

	effectors:resize(countExistingJoint*3-3+3*c)	--excluding the root
	effectorPos:resize(countExistingJoint*3-3+3*c)
	local src_bones={}
	local c=0
	local SITE_NUM={}
	for i=1,countExistingJoint-1 do
		local bone=MainLib.VRMLloader.upcast(skel2:getBoneByName(jointName(i)))
		assert(bone~=nil)
		local srcbone=skel1:getBoneByName(jointName(i))

		print("JOINT", bone, srcbone)

		-- add joint marker
		local markerDistance=motion.markerDistance or 0.02
		
		--ys
		if motion.upper_markerDistance~=nil then
			if jointName(i)=='back' then
				markerDistance=motion.upper_markerDistance
			end
		end

		effectors:at(c):init(bone, vector3(markerDistance,0,0))
		src_bones[c]=srcbone
		c=c+1
		effectors:at(c):init(bone, vector3(0,markerDistance,0))
		src_bones[c]=srcbone
		c=c+1
		effectors:at(c):init(bone, vector3(0,0,markerDistance))
		src_bones[c]=srcbone
		c=c+1
		if bone:childHead()==nil then
			-- add SITE markers
			local offset=bone:localCOM()
--			print("offset", offset)
--			offset = Bone.getOffset(srcbone)
--			print("srcbone offset", offset)

			print("SITE", bone, srcbone, offset)
			local mlen=0.02
			if string.find(bone:name(), "foot") then
				mlen=0.07 -- ankle orientation is very important
			end

			effectors:at(c):init(bone, vector3(mlen,0,0)+offset*2)
			src_bones[c]=srcbone
			SITE_NUM[c] = 1
			print(c.."		"..SITE_NUM[c])
			c=c+1
			effectors:at(c):init(bone, vector3(0,mlen,0)+offset*2)
			src_bones[c]=srcbone
			SITE_NUM[c] = 1
			c=c+1
			effectors:at(c):init(bone, vector3(0,0,mlen)+offset*2)
			src_bones[c]=srcbone
			SITE_NUM[c] = 1
			c=c+1

		end

	end

	local iksolver=createIKsolver(skel2,effectors)

	--dcjo
--	local rotvec3 = vector3(0,1,0)
--	local q = quater(3.14/2, rotvec3)
--	Motion.rotate(mot1, q)
	if motion.default_pose_rotation ~= nil then
		Motion.rotate(mot1, motion.default_pose_rotation:inverse())
	end

	for ii=0,numFrame2-1 do

		local i=ii+startFrame
		if reversed then
			i=startFrame-ii
		end
		mot1:setSkeleton(i)

		-- dcjo
		for j=0, c-1 do

			if motion.default_pose_rotation == nil then
				effectorPos:at(j):assign(
				src_bones[j]:getFrame():toGlobalPos(
				effectors(j).localpos))
			else
				local localpos = vector3(effectors(j).localpos.x,effectors(j).localpos.y, effectors(j).localpos.z);
				localpos:rotate(motion.default_pose_rotation)
				effectorPos:at(j):assign(
				src_bones[j]:getFrame():toGlobalPos(
				localpos))
			end
		end

		print("ik"..i)

		-- mot2 - MotionDOFcontainer    mot2.mot - motionDOF			mot1 - Motion
		if reversed then
			-- use next frame as initial solution.
			mot2:row(i):assign(mot2:row(i+1))												-- matrixn:row(index)
			MotionDOF.setRootTransformation(mot2:row(i), mot1:rootTransformation(i))
			iksolver:IKsolve(mot2:row(i), effectorPos)
			if i==0 or mot1:isConstraint(i, Motion.IS_DISCONTINUOUS) then
				mot2:setConstraint(i, Motion.IS_DISCONTINUOUS, true)
			end
		else
			if i==0 or mot1:isConstraint(i, Motion.IS_DISCONTINUOUS) then
				mot2:row(ii+startFrame):setAllValue(0)
--				local transff = mot1:rootTransformation(i)
--				transff= transff * trans
--				MotionDOF.setRootTransformation(mot2:row(ii+startFrame), transff)--mot1:rootTransformation(i))
				MotionDOF.setRootTransformation(mot2:row(ii+startFrame), mot1:rootTransformation(i))
--				MotionDOF.setRootTransformation(mot2:row(ii+startFrame), mot1rootTrans[i])
				iksolver:IKsolve(mot2:row(ii+startFrame), effectorPos)
				mot2:setConstraint(ii+startFrame, Motion.IS_DISCONTINUOUS, true)
			else	-- use previous frame as initial solution.
				mot2:row(ii+startFrame):assign(mot2:row(ii+startFrame-1))
--				local transff = mot1:rootTransformation(i)
--				transff = transff *trans
--				MotionDOF.setRootTransformation(mot2:row(ii+startFrame), transff)--mot1:rootTransformation(i))
				MotionDOF.setRootTransformation(mot2:row(ii+startFrame), mot1:rootTransformation(i))
--				MotionDOF.setRootTransformation(mot2:row(ii+startFrame), mot1rootTrans[i])
				iksolver:IKsolve(mot2:row(ii+startFrame), effectorPos)
			end
		end

	end
elseif conversionMethod==conversionMethod_T.useLocalOrientation then

	local existing_bones1={}
	local existing_bones2={}

	for i=0,countExistingJoint-1 do
		existing_bones1[i]=skel1:getBoneByName(jointName(i))
		existing_bones2[i]=skel2:getBoneByName(jointName(i))
	end

	for ii=0,numFrame2-1 do
		local i=ii+startFrame

		mot2:row(i):setAllValue(0)

		local dim={'x','y','z'}	 
		for j=0, countExistingJoint-1 do
			local ti1=existing_bones1[j]:treeIndex()
			local ti2=existing_bones2[j]:treeIndex()
			--            assert(skel1.dofInfo:numDOF(ti1)==
			--	skel2.dofInfo:numDOF(ti2))
			local startT1=skel1.dofInfo:startT(ti1)
			local startR1=skel1.dofInfo:startR(ti1)
			local endR1=skel1.dofInfo:endR(ti1)

			local startT2=skel2.dofInfo:startT(ti2)
			local startR2=skel2.dofInfo:startR(ti2)
			local endR2=skel2.dofInfo:endR(ti2)

			local channel1=existing_bones1[j]:getRotationalChannels()
			local channel2=existing_bones2[j]:getRotationalChannels()
			if endR2-startR2==4 then
				-- ball joint
				assert(string.len(channel1)==3)
				for d=startT2, startR2-1 do
					mot2.mot:matView():set(i, d, mot1.mot:matView()(i, d-startT2+startT1))
				end

				local q1=quater()

				if endR1-startR1==4 then
					q1=mot1.mot(i):toQuater(startR1)
				else
					local v1=mot1.mot(i):toVector3(startR1)
					q1:setRotation(channel1, v1)
				end

				local q2_ref=mot2.mot(math.max(0, i-1)):toQuater(startR2)
				q1:align(q2_ref)

				mot2.mot:matView():row(i):setQuater(startR2, q1)

			elseif channel1 ~= channel2 then
				assert(string.len(channel1)==3)
				--	       assert(string.len(channel2)==3)
				for d=startT2, startR2-1 do
					mot2.mot:matView():set(i, d, mot1.mot:matView()(i, d-startT2+startT1))
				end

				-- if startR1>= mot1.mot(i):size()-3 then
				-- 	  debug.debug()
				-- end
				local v1=mot1.mot(i):toVector3(startR1)
				local q1=quater()
				q1:setRotation(channel1, v1)

				local v2_ref=vector3()

				for d=startR2, endR2-1 do
					v2_ref[dim[d-startR2+1]]=mot2.mot:matView()(math.max(0,i-1), d)
				end

				local q2_ref=quater()
				q2_ref:setRotation(channel2, v2_ref)
				q1:align(q2_ref)

				local q2=vector3()
				q1:getRotation(channel2, q2)


				for d=startR2, endR2-1 do
					mot2.mot:matView():set(i, d, q2[dim[d-startR2+1]])
				end
			else
				for d=startT2, endR2-1 do
					mot2.mot:matView():set(i, d, mot1.mot:matView()(i, d-startT2+startT1))
				end
			end


		end

		if i==0 or mot1:isConstraint(i, Motion.IS_DISCONTINUOUS) then
			mot2:setConstraint(i, Motion.IS_DISCONTINUOUS, true)
		end
	end
elseif conversionMethod==conversionMethod_T. useConvertMotionToMotDOFfunc then
	-- output: mot2
	-- input: skel1 : src_skel_file
	--        skel2 : wrl
	--        mot1  : skel1.mMotion
	--

	local output=convertMotionToMotDOF(skel1, mot1, skel2)
	mot2.mot:matView():assign(output:matView():sub(0, endFrame))

else
	assert(false)
end
local outFileName=motion.out_file

if util.isFileExist(outFileName) then
	if askOverwrite and not Fltk.ask(outFileName.." exists. Do you want to overwrite it?") then
		util.msgBox("exporting canceled")
	else
		print('exporting '..outFileName)
		mot2:exportMot(outFileName)
	end
else
	print('exporting '..outFileName)
	mot2:exportMot(outFileName)
end

if motion.src_dof_files~=nil then
	mot1conv=Motion(skel1)
	mot1.mot:get(mot1conv)
	mot1back=mot1
	mot1=mot1conv
end

end

function ctor()

	for i,target in ipairs(targets) do
		if target=="justin_run" then
		endFrame=607
		-- elseif target=="justin_runf3_cart" then
		--    endFrame=717
	end

	convert(target)
end

assert(skel2)
mSkin2=RE.createVRMLskin(skel2, true)
--mSkin2:scale(100,100,100)

mSkin2:scale(motion.skinScale or 100,motion.skinScale or 100,motion.skinScale or 100)
mSkin2:setThickness(0.02)
mSkin2:applyMotionDOF(mot2.mot)
mSkin2:setMaterial("lightgrey_transparent")
mSkin2:startAnim()

RE.motionPanel():motionWin():addSkin(mSkin2)



mot2conv=Motion(skel2)
mot2.mot:get(mot2conv)
createDebugSkin(mot2conv, vector3(100,0,0))

end

function createDebugSkin(mot, translation)
	local skin=RE.createSkin(mot, PLDPrimSkin.POINT)
	skin:scale(1,1,1)
	skin:setThickness(1)
	local skin2=RE.createSkin(mot, PLDPrimSkin.BOX)
	skin2:scale(1,1,1)
	skin2:setThickness(1)

	if translation~=nil then
		skin:setTranslation(translation.x, translation.y, translation.z)
		skin2:setTranslation(translation.x, translation.y, translation.z)
	end
	RE.connectSkin(skin)
	RE.connectSkin(skin2)
end

function dtor()
	dbg.finalize()
	RE.removeAllConnectedSkins()
	if RE.motionPanelValid() then RE.motionPanel():motionWin():detachAllSkin() end
	collectgarbage()
end

function onCallback(w, userData)
end

function frameMove(fElapsedTime)
end

function createIKsolver(skel, effectors)

	-- return MotionUtil.createFullbodyIkDOF_MultiTarget(skel.dofInfo, effectors)
	-- much faster with almost identical results
	return MotionUtil.createFullbodyIkDOF_UTPoser(skel.dofInfo, effectors)
end

 FullbodyIK_UTPoser=LUAclass()
function FullbodyIK_UTPoser:__init(skel, effectors)
	self.skel=skel
	self.iksolver=MotionUtil.createFullbodyIkDOF_UTPoser(skel.dofInfo, effectors)
	self.tempPose=vectorn()
end

function FullbodyIK_UTPoser:IKsolve(pose_inout, effectorPos)
	self.skel.dofInfo:getDOF(pose_inout, self.tempPose)
	self.iksolver:IKsolve(self.tempPose, effectorPos)
	self.skel.dofInfo:setDOF(self.tempPose, pose_inout)
end
