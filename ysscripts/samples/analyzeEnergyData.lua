require("config")

package.projectPath='../Samples/QP_controller/'
package.path=package.path..";../Samples/QP_controller/lua/?.lua" --;"..package.path
package.path=package.path..";../Samples/classification/lua/?.lua" --;"..package.path

require("IPC_based/common")

package.path=package.path..";../Samples/ysscripts/samples/?.lua" --;"..package.path
require("useMuscles")

require("subRoutines/motiongraph")

require("IPC_based/LocoGraph")

-- 최적화 루틴 사용법
-- trackingOptimizer.lua최적화 후 cartpoleanalysis2.lua로 타겟포즈 익스포트한다.
-- 그 후 useCase.lua적당히 고친 후 zmpcontrollerfullbody2.lua에서 불러온다.
--require("IPC_based/LocoSimulation")
-- following values are automatically connected to UI.

package.path=package.path..";../Samples/ysscripts/lib/?.lua" --;"..package.path
require('QPservo2')
require("utilfunc")
require("OsModel")
require("LocoSimMuscle")

gTimerMuscle=util.PerfTimer2()

finalRender=false--true
cleanVisualize = true

g_plotdata = false

g_intgplot = false

function ctor()
	
	this:updateLayout()

	RE.renderer():fixedTimeStep(true)

	model = scenarios.toModel(useCase.scenario)
	mOsim = OsModel(model.wrlpath, model.luamsclpath, model)
	
	mLoader = mOsim.mLoader

	do
		--temp
		local container=MotionDOFcontainer(mLoader.dofInfo, model.mot_file)
		mMotionDOF=container.mot
		mMotionDOF:resize(1200)

	-- dcjo to fix motion with ankle_r degree 20
	-- modify motion manually at LocoGraphMotion:__init() in IPC_based/LocoGraph.lua
		-- fill mot dof
		mOsim.mSkin:applyMotionDOF(mMotionDOF)
		RE.motionPanel():motionWin():detachAllSkin()
		RE.motionPanel():motionWin():addSkin(mOsim.mSkin)
	end
	-- to draw objects in Ogre
	mObjectList=Ogre.ObjectList()

	mSynthesis	= OnlineLocoSynthesis:new()
	boolean_options={}
	boolean_options.attachCamera=useCase.attachCamera or false
	boolean_options.drawDisplacementMap=false
	boolean_options.useOrientationCorrection=false
	boolean_options.solveIK=true
	boolean_options.drawPredictedCOM=false
	boolean_options.drawPredictedZMP =true
	boolean_options.drawPelvis=false
	boolean_options.drawControlForce=true
	boolean_options.drawMatching=true


	-- load data table
	local msclEnergyData_angle = util.loadTable("msclEnergyData_angle.tbl")
	local msclEnergyData_mscl = util.loadTable("msclEnergyData_mscl.tbl")

	local distanceData_angle = util.loadTable("distanceData_angle.tbl")

	local msclRatioData_angle = nil


	if msclEnergyData_angle~= nil then
		print("loading angle success")
	end
	if msclEnergyData_mscl~= nil then
		print("loading mscl success")
	end

	--[[
	for i=1, 90 do
		if msclEnergyData_angle[i] ~= nil then
			print(i)
			print(msclEnergyData_angle[i][1])
			print(msclEnergyData_angle[i][1]:get(4))
		end
	end

]]
	local joints = {'ankle_r'}
	local relatedMuscles = mOsim:getMusclesRelatedJoint(joints)
	for i=1, #relatedMuscles do 
--		printtblh(relatedMuscles[i])
	end
	
	local relatedMsclIdxs = mOsim:getMuscleIdxsRelatedJoint(joints)
	for i=1, #relatedMsclIdxs do 
		print(relatedMsclIdxs[i])
	end



	function analyzeJoint(joints)
		local relatedMuscleIdxs = mOsim:getMuscleIdxsRelatedJoint(joints)
		
		local mergedEnergyData = {}
		
		for angle = 1, 90 do 
			if msclEnergyData_angle[angle] ~= nil then
				for k,v in pairs(relatedMuscleIdxs) do 
					if mergedEnergyData[angle] == nil then
						mergedEnergyData[angle] = msclEnergyData_angle[angle][v]
					else
						mergedEnergyData[angle] = mergedEnergyData[angle] + msclEnergyData_angle[angle][v]
					end
				end
				-- distance

			end
		end

		return mergedEnergyData
	end

	function analyzeJointRatio(joints)
		if msclRatioData_angle == nil then
			msclRatioData_angle = makeMsclRatioData()
		end
		local relatedMuscleIdxs = mOsim:getMuscleIdxsRelatedJoint(joints)
		
		local mergedRatioData = {}
		
		for angle = 1, 90 do 
			if msclRatioData_angle[angle] ~= nil then
				for k,v in pairs(relatedMuscleIdxs) do 
					if mergedRatioData[angle] == nil then
						mergedRatioData[angle] = msclRatioData_angle[angle][v]
					else
						mergedRatioData[angle] = mergedRatioData[angle] + msclRatioData_angle[angle][v]
					end
				end
				-- distance

			end
		end

		return mergedRatioData
	end

	function mergeAllJoint()
		local mergedEnergyData = {}
		for angle = 1, 90 do 
			if msclEnergyData_angle[angle] ~= nil then
				for mscl_i = 1, #mOsim.msclname do
					if mergedEnergyData[angle] == nil then
						mergedEnergyData[angle] = msclEnergyData_angle[angle][mscl_i]
					else
						mergedEnergyData[angle] = mergedEnergyData[angle] + msclEnergyData_angle[angle][mscl_i]
					end
				end
			end
		end

		return mergedEnergyData
	end

	function divideByDistance(mergedEnergyData, distanceData)
		local mergedEnergyDataPerDistance = {}
		for k, v in pairs(mergedEnergyData) do 
			mergedEnergyDataPerDistance[k] = mergedEnergyData[k] / distanceData[k]
		end
		return mergedEnergyDataPerDistance
	end


	function makeMsclRatioData()
		local msclRatioData_angle = {}
		local totalData_angle = mergeAllJoint()

		for k_a, v_a in pairs(msclEnergyData_angle) do 
			for k_m, v_m in pairs(v_a) do
				if msclRatioData_angle[k_a] == nil then
					msclRatioData_angle[k_a] = {}
				end
				local msclRatioData_vn = vectorn(5)
				for i=0, 4 do 
					msclRatioData_vn:set(i, v_m:get(i) / totalData_angle[k_a]:get(i))
				end
				msclRatioData_angle[k_a][k_m] = msclRatioData_vn
			end
		end

		return msclRatioData_angle
	end


	function writeMergedEnergyData(mergedEnergyData, fileName)
		
		for angle = 1, 90 do 
			if mergedEnergyData[angle] ~= nil then
				local output = ""..angle
				for i = 1, 5 do
					print(mergedEnergyData[angle])
					print(i)
					local MEDtbl = vecn2tbl(mergedEnergyData[angle])
--					output = output.."\t"..(mergedEnergyData[angle]:get(i))
					output = output.."\t"..(MEDtbl[i])
				end
				util.outputToFile(fileName, output)
			end
		end
	end
	

--[[
	print("mtp_r_start")
	writeMergedEnergyData(analyzeJoint({"mtp_r"}), "analyzedMED_mtp_r.dat")
	print("ankle_r_start")
	writeMergedEnergyData(analyzeJoint({"ankle_r"}), "analyzedMED_ankle_r.dat")
	print("knee_r_start")
	writeMergedEnergyData(analyzeJoint({"knee_r"}), "analyzedMED_knee_r.dat")
	print("done")
	writeMergedEnergyData(analyzeJoint({"hip_r"}), "analyzedMED_hip_r.dat")

	print("start")
	writeMergedEnergyData(analyzeJoint({"mtp_r", "ankle_r", "knee_r"}), "analyzedMED_total_r.dat")
	print("done")

	print("start ALL")
	writeMergedEnergyData(mergeAllJoint(), "analyzedMED_total.dat")
	print("all done")
]]


	print("mtp_r_start")
	writeMergedEnergyData(divideByDistance(analyzeJoint({"mtp_r"}), distanceData_angle), "analyzedMED_mtp_r.dat")
	print("ankle_r_start")
	writeMergedEnergyData(divideByDistance(analyzeJoint({"ankle_r"}), distanceData_angle), "analyzedMED_ankle_r.dat")
	print("knee_r_start")
	writeMergedEnergyData(divideByDistance(analyzeJoint({"knee_r"}), distanceData_angle), "analyzedMED_knee_r.dat")
	print("done")
	writeMergedEnergyData(divideByDistance(analyzeJoint({"hip_r"}), distanceData_angle), "analyzedMED_hip_r.dat")

	print("start")
	writeMergedEnergyData(divideByDistance(analyzeJoint({"mtp_r", "ankle_r", "knee_r", "hip_r"}), distanceData_angle), "analyzedMED_total_r.dat")
	print("done")

	print("start ALL")
	writeMergedEnergyData(divideByDistance(mergeAllJoint(), distanceData_angle), "analyzedMED_total.dat")
	print("all done")

-- ratioData
	print("mtp_r_start")
	writeMergedEnergyData(analyzeJointRatio({"mtp_r"}), "analyzedMED_mtp_r_ratio.dat")
	print("ankle_r_start")
	writeMergedEnergyData(analyzeJointRatio({"ankle_r"}), "analyzedMED_ankle_r_ratio.dat")
	print("knee_r_start")
	writeMergedEnergyData(analyzeJointRatio({"knee_r"}), "analyzedMED_knee_r_ratio.dat")
	print("done")
	writeMergedEnergyData(analyzeJointRatio({"hip_r"}), "analyzedMED_hip_r_ratio.dat")

	print("start")
	writeMergedEnergyData(analyzeJointRatio({"mtp_r", "ankle_r", "knee_r"}), "analyzedMED_total_r_ratio.dat")
	print("done")












--[[
	local msclidx_lists = {}
	for i = 1, #mOsim.msclname do
		local msclidx_list = {}
		msclidx_list[i] = i
		msclidx_lists[#msclidx_lists+1] = msclidx_list
	end
]]

--[[
	local msclEnergyData_angle = {}
	local msclEnergyData_mscl = {}
	for mscl_i = 1, #mOsim.msclname do 
		msclEnergyData_mscl[mscl_i] = {}
	end
]]


--[[
	titlebar:create()
	gTimerMuscle:reset()
	if g_plotdata then
		local dataname = useCase.modelName..'__'..g_mode
		mFile_l_mt = io.open(dataname..'__lmt.txt', 'w')
		mFile_act = io.open(dataname..'__act.txt', 'w')
		mFile_angle = io.open(dataname..'__ang.txt', 'w')
		mFile_grf = io.open(dataname..'__grf.txt', 'w')

		--lmt, act
		mFile_l_mt:write('frame\t')
		mFile_act:write('frame\t')
		for m,msclName in ipairs(mOsim:getMuscleNames()) do
			mFile_l_mt:write(msclName..'.lmt'..'\t')
			mFile_act:write(msclName..'.act'..'\t')
		end
		mFile_l_mt:write('\n')
		mFile_act:write('\n')

		--angle
		local angleNames={'hip_r.ang_flx','knee_r.ang_flx','ankle_r.ang_dor', 'pelvis.ang_roll'}
		mAngleIndices = {mOsim.rhipflexDOF, mOsim.rkneeDOF, mOsim.rankleDOF, -1}
		mFile_angle:write('frame\t')
		for m,angName in ipairs(angleNames) do
			mFile_angle:write(angName..'\t')
		end
		mFile_angle:write('\n')

		--grf
		local grfNames={'rfoot.grf.x','rfoot.grf.y','rfoot.grf.z','lfoot.grf.x','lfoot.grf.y','lfoot.grf.z'}
		mFile_grf:write('frame\t')
		for m,grfName in ipairs(grfNames) do
			mFile_grf:write(grfName..'\t')
		end
		mFile_grf:write('\n')

	end
]]--


end


function dtor()
	if g_plotdata then
		io.close(mFile_l_mt)
		io.close(mFile_act)
		io.close(mFile_angle)
		io.close(mFile_grf)
	end

   dbg.finalize()
   if mSynthesis then
	   mSynthesis:__finalize()
	   mSynthesis=nil
   end
   titlebar:destroy()
end

