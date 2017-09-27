require("config")

package.projectPath='../Samples/QP_controller/'
package.path=package.path..";../Samples/QP_controller/lua/?.lua" --;"..package.path
package.path=package.path..";../Samples/classification/lua/?.lua" --;"..package.path

require("IPC_based/common")

package.path=package.path..";../Samples/ysscripts/samples/?.lua" --;"..package.path
--require("IPC_based/useCases")
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
--dcjo
energy_test = false
--cleanVisualize=true
cleanVisualize = true
drawRefPose = true

g_plotdata = true
--g_plotdata = false

--g_intgplot = true
g_intgplot = false

function ctor()
--	modelChooser:createMenu("load a predefined model")
	this:create('Button', 'attach camera', 'attach camera')
	this:create("Button"	, "impulse", "impulse",0)


	--dcjo
	this:create("Button", "SAT", "shorten achiless tendon")
	this:create("Button", "RAT", "recover achiless tendon")
	this:create("Button", "RFT", "rectus femoris transfer")
	this:create("Button", "RFR", "rectus femoris retransfer")

	this:updateLayout()

	RE.renderer():fixedTimeStep(true)
	mEventReceiver=EVR()

	model = scenarios.toModel(useCase.scenario)
	mOsim = OsModel(model.wrlpath, model.luamsclpath, model)
	
	mLoader = mOsim.mLoader

	print('original f_m_o')
	printtblh(mOsim.f_m_o)
	if useCase.f_m_o ~= nil then
		mOsim.f_m_o = useCase.f_m_o
		print('adapted f_m_o')
		printtblh(mOsim.f_m_o)
	end

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

	-- dcjo. modify transform of bone


	--[[	local achiless_muscle_l = {'soleus_l', 'lat_gas_l', 'med_gas_l'}
	local achiless_muscle_r = {'soleus_r', 'lat_gas_r', 'med_gas_r'}
	if useCase.muscleModification then
		for i=1,#achiless_muscle_r do
			local i_mscl = mOsim:getMuscleIndexWithName(achiless_muscle_r[i])
			mOsim.l_t_sl[i_mscl] = mOsim.l_t_sl[i_mscl] - 0.030
		end
		for i=1,#achiless_muscle_l do
			local i_mscl = mOsim:getMuscleIndexWithName(achiless_muscle_l[i])
			mOsim.l_t_sl[i_mscl] = mOsim.l_t_sl[i_mscl] - 0.030
		end
	end
	]]
	-- dcjo. modify rectus femoris
	-- remove last pathpoints
--[[	local rectus_muscle = {}--{'rect_fem_r', 'rect_fem_l'}
	for i=1,#rectus_muscle do
		local i_mscl = mOsim:getMuscleIndexWithName(rectus_muscle[i])
		local removed_path = {}
		for j=1,#mOsim.pathpoints do
			if mOsim.pathpoints[j].muscleindex == i_mscl then
				if mOsim.pathpoints[j].joint == 'knee_r' or mOsim.pathpoints[j].joint == 'knee_l' then
					printtblh(mOsim.pathpoints[j])
					table.insert(removed_path, j)
				elseif mOsim.pathpoints[j].type == 'ConditionalPathPoint' then
					mOsim.pathpoints[j].coordinate = nil
					mOsim.pathpoints[j].range = nil
					mOsim.pathpoints[j].type = 'MovingPathPoint'
					printtblh(mOsim.pathpoints[j])
				end
			end
		end
		for j=1,#removed_path do
			table.remove(mOsim.pathpoints, removed_path[j])
			table.remove(mOsim.pathpointposz,removed_path[j])
		end
	end
--	print(#mOsim.pathpoints)
	mOsim.validPPIndices = false
]]
	-- make simulator with mOsim

	mSynthesis	= OnlineLocoSynthesis:new()
	useCases.unmapControlParam(useCase)

	mSynthesis:initializeControlParameters()
	mSynthesis:resetInitialState()
	mSynthesis:__updateConstraints()

--	refSynthesis = OnlineSynthesisMuscle:new()

--	useCases.accumulate(useCases.g2592_surgery, {['keyframe,1,ankleLmod,g2592_surgery,R1,x']=0.0, ['keyframe,1,ankleRmod,g2592_surgery,L,x']=0.0})
--	mSynthesis:initializeControlParameters()
--[[
	local joints = {'ankle_r'}
	local relatedMuscles = mOsim:getMusclesRelatedJoint(joints)
	for i=1, #relatedMuscles do
		printtblh(relatedMuscles[i])
	end
]]


	do
		local fn, path=os.processFileName(string.sub(model.file_name ,1, -5))
		markerFile=path.."/"..fn.."_sd/optComparision_marker.lua"
	end

	outputMotion=MotionDOFcontainer(mSynthesis.skel_withoutCOMjoint.dofInfo)
	compareChain=CompareChain2D(mSynthesis.skel_withoutCOMjoint,
	mSynthesis.simulator:getWorldState(0),
	mSynthesis.skel_withoutCOMjoint:fkSolver(), markerFile)

	origDMot=MotionDOF(mSynthesis.skel_withoutCOMjoint.dofInfo) -- original motion that is temporally aligned to the synthesized motion
	origMot=MotionDOF(mSynthesis.skel_withoutCOMjoint.dofInfo) -- original motion that is temporally aligned to the synthesized motion
	
	poseDiff = 0
	poseAvgDiff = 0
	segMarkerPoints = {}
	segGPPoints = {}
	poseDiff2 = 0
	poseAvgDiff2 = 0
	objFn = 0
	poseDiffPenalty = 0

	MSE=0
	nMSE=0


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

	---- camera
	--RE.viewpoint().vpos:set(30, 100, 300)	--side view
	----RE.viewpoint().vpos:set(300, 100, 30)	-- front view
	
	--RE.viewpoint().vat:set(0,80,0)
	--RE.viewpoint():update()
	
	--mCameraInfo={}
	--local curPos=vector3(0,0,0)
	--mCameraInfo.vpos=RE.viewpoint().vpos-curPos
	--mCameraInfo.vat=RE.viewpoint().vat-curPos
	--recCurPos = {}
	
	mCameraInfo={}
	recCurPos = {}
	
	setCamVPos(30,100,300)	--side view
	--setCamVPos(300,100,30)	--front view
	--setCamVPos(300,200,200)	--push view

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

end


function setCamVPos(x,y,z)
	RE.viewpoint().vpos:set(x,y,z)
	RE.viewpoint().vat:set(0,80,0)
	RE.viewpoint():update()

	local curPos=vector3(0,0,0)
	mCameraInfo.vpos=RE.viewpoint().vpos-curPos
	mCameraInfo.vat=RE.viewpoint().vat-curPos
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

function onCallback(w, userData)

	print("mouse!!!1")

	if w:id()=='attach camera' then
		mEventReceiver:attachCamera()
	elseif w:id()=="impulse" then
		float_options={}
		float_options.impulseMagnitude={ val=50, min=10, max=1000}
		float_options.impulseDuration={ val=0.2, min=1, max=100}
		float_options.desiredAngleX={ val=0, min=-1, max=1}
		float_options.desiredAngleZ={ val=0, min=-1, max=1}
		float_options.timescaleL={ val=0, min=-0.5, max=0.5}

		mSynthesis.impulse=float_options.impulseDuration.val*model.simulationFrameRate
		--mSynthesis.impulseDir=vector3(1,0,0)*float_options.impulseMagnitude.val
		mSynthesis.impulseDir=vector3(0,0,1)*float_options.impulseMagnitude.val
		mSynthesis.impulseGizmo=mSynthesis.objectList:registerEntity("arrow2", "arrow2.mesh")
		--mSynthesis.impulseGizmo:setScale(2,2,2)
	
	
--dcjo
	elseif w:id()=="SAT" then

		printtblh(mOsim.pathpoints)

		for i, pp in ipairs(mOsim.pathpoints) do
			printtblh(pp)
			printtblh(pp.location)
		end

--		mSynthesis = OnlineLocoSynthesis:new()	


	elseif w:id()=="RAT" then

			local muscleList = {'rect_fem_l', 'rect_fem_r', 'soleus_l', 'soleus_r', 'lat_gas_l', 'lat_gas_r', 'med_gas_l', 'med_gas_r'}
			mOsim:drawSomeMuscles(0, mObjectList, 1, 0, muscleList) 
	elseif w:id()=="RFT" then

		print(#mOsim.pathpoints)
		for i=1,#mOsim.pathpoints do
			printtblh(mOsim.pathpoints[i])
		end
	elseif w:id()=="RFR" then

		print(#mOsim.pathpoints)
		
	end
end


function frameMove(fElapsedTime)
	--mSynthesis:oneStep()
	--mSynthesis:oneStepSimul()
	--if error_feedback_method~=EFM.NONE then	 
		--mSynthesis:prepareNextStep()      
	--end
	
--[[	
	if error_feedback_method==EFM.NONE or mSynthesis.errorOccurred then
		return
	end
	
	mSynthesis:oneStep()
	mSynthesis:oneStepSimul()
	if error_feedback_method~=EFM.NONE then	 
		mSynthesis:prepareNextStep()      
	end
	g_simulated_ifr = mSynthesis.numFrames
	print(g_simulated_ifr)
	]]
end

if EventReceiver then
	EVR=LUAclass(EventReceiver)
	function EVR:__init(graph)
		self.currFrame=0
		self.cameraInfo={}
	end
end

g_kcal = 0
g_simulated_ifr = 0
g_int_met = 0
-- for camera setting
g_prev_ifr = 0
function EVR:attachCamera()

	if true then
		return
	end

	if g_simulated_ifr < 130 then
		return
	end

	local s=0
	local e=g_simulated_ifr


	self.trajectory=matrixn(e,3)
	self.trajectoryOri=matrixn(e,4)

	for f=s,e-1 do
		--self.trajectory:row(f):setVec3(0, MotionDOF.rootTransformation(mMotionDOF:row(f)).translation)
		--self.trajectory:row(f):set(1,0)
		--self.trajectoryOri:row(f):setQuater(0, MotionDOF.rootTransformation(mMotionDOF:row(f)).rotation:rotationY())
		self.trajectory:row(f):setVec3(0, MotionDOF.rootTransformation(mOsim.records[f+1]["bfkPoseDOF"]).translation)
		self.trajectory:row(f):set(1,0)
		self.trajectoryOri:row(f):setQuater(0, MotionDOF.rootTransformation(mOsim.records[f+1]["bfkPoseDOF"]).rotation:rotationY())
	end
	print("filtering",s,e)
	--math.filter(self.trajectory:range(s,e,0, 3), 63)
	--math.filter(self.trajectoryOri:range(s,e,0, 4), 63)
	math.filter(self.trajectory:range(s,e,0, 3), 63)
	math.filter(self.trajectoryOri:range(s,e,0, 4), 121)
	-- further smoothing
	-- desired contact state
	local state=mrd_info.outputMotion.signals.state
	assert(state:rows()==e)
	--if true then	-- taesoo camera smoothing
	if false then
		local conL=boolN(state:rows())
		conL:setAllValue(false)
		local conR=boolN(state:rows())
		conR:setAllValue(false)

		for i=0, e-1 do
			local s=state(i,0)
			if s==0 then
				conL:set(i, true)
			elseif s==1 then
				conR:set(i, true)
			else
				conL:set(i, true)
				conR:set(i, true)
			end
		end
		-- curve keyframes
		local function extract(con, curve, isQ)
			local coni=intIntervals()
			coni:runLengthEncode(con)
			local key=vectorn(coni:size()+2)
			local val=matrixn(coni:size()+2,curve:cols())
			key:set(0, 0)
			val:row(0):assign(curve:row(0))
			key:set(coni:size()+1, e-1)
			val:row(coni:size()+1):assign(curve:row(e-1))

			for i=0, coni:size()-1 do
				key:set(i+1, math.floor((coni:startI(i)+coni:endI(i))*0.5+0.5))
				val:row(i+1):assign(curve:row(key(i+1)))
			end

			if isQ then
				for i=0,val:size()-1 do
					val:row(i):assign(val:row(i):toQuater():rotationY())
				end
			end
			
			local sp=math.NonuniformSpline(key, val)
			local points=matrixn()
			local time=vectorn()
			time: linspace(0, e-1, e)
			sp:getCurve(time, points)
			return points
		end

		local lori=extract(conL, self.trajectoryOri, true)
		local rori=extract(conR, self.trajectoryOri, true)
		local lpos=extract(conL, self.trajectory)
		local rpos=extract(conR, self.trajectory)
		local ori=(lori+rori)*0.5
		local pos=(lpos+rpos)*0.5

		for i=0, e-1 do
			local q=ori:row(i):toQuater(0):rotationY()
			q:normalize()
			self.trajectoryOri:row(i):assign(q)
			self.trajectory:row(i):assign(pos:row(i):toVector3(0))
		end

	end
	local curPos=self.trajectory:row(self.currFrame):toVector3(0)*100
	--self.cameraInfo.vpos=RE.viewpoint().vpos-curPos
	--self.cameraInfo.vat=RE.viewpoint().vat-curPos
	--self.cameraInfo.dist=RE.viewpoint().vpos:distance(curPos)
	self.cameraInfo.vpos=mCameraInfo.vpos-curPos
	self.cameraInfo.vat=mCameraInfo.vat-curPos
	self.cameraInfo.dist=RE.viewpoint().vpos:distance(curPos)
	self.cameraInfo.refRot=self.trajectoryOri:row(self.currFrame):toQuater(0):rotationY()
end

g_rec_energy = {}
g_rec_impulse_t = {}
g_lfootcf_temp = 0
g_rfootcf_temp = 0
g_lankpff_temp = 0
g_rankpff_temp = 0
g_lrknee_temp = 0
g_cap_met = true
g_cap_eng = true
g_cap_eff = true
g_cap_spd = true

render_interval = 4
int_as = {}
render_as = {}

-- dcjo
currSegIndex = 0
prev_comPos = vector3()
	
function EVR:onFrameChanged_(win, iframe)
	if iframe > g_simulated_ifr then
		local prev_simulated_ifr = g_simulated_ifr

		for f=1,iframe-prev_simulated_ifr do
			if error_feedback_method==EFM.NONE or mSynthesis.errorOccurred then
				return
			end
	
			mSynthesis:oneStep()
			mSynthesis:oneStepSimul()
			if error_feedback_method~=EFM.NONE then	 
				mSynthesis:prepareNextStep()      
			end
			ifr = prev_simulated_ifr + f

			mOsim:drawMuscles(ifr, mObjectList)

			g_simulated_ifr = ifr
		end
	else
		mOsim:drawRecordedScene(iframe)
	end
	print(g_simulated_ifr)
end	

function EVR:onFrameChanged(win, iframe)

	if iframe > g_simulated_ifr then
		local prev_simulated_ifr = g_simulated_ifr

		for f=1,iframe-prev_simulated_ifr do
			local ifr = prev_simulated_ifr + f

			local perfFrame = 100
			--print()
			--print('simulate frame ', ifr)

			-- performance timer
			if ifr <= perfFrame then
				gTimerMuscle:start()
			end


			-- simulation
			if dbg_testmw then -- search for dbg_testmw in locoSimMuscle.lua 
				-- debug testmw
				local g_nf=mSynthesis.numFrames
				if preciseComparison and g_nf>=preciseComparison[1] and g_nf<preciseComparison[2] then
					g_debugOneStep=array:new()
					g_debugOneStepSaveFlag=true
				else
					g_debugOneStep=nil
				end
				mSynthesis:oneStep()
				if g_debugOneStep then
					util.saveTable(g_debugOneStep, "debugstates_comp/debugStates_oneStep1-"..g_nf..".tbl" )
					g_debugOneStepSaveFlag=false
				end
				mSynthesis:saveStates("debugstates_comp/debugStates1__"..g_nf..".tbl")
				if preciseComparisonSimul and g_nf>=preciseComparisonSimul[1] and g_nf<preciseComparisonSimul[2] then
					g_debugOneStep=array:new()
					g_debugOneStepFlag =true -- so that only the result of the first simulation frame is stored.
				else
					g_debugOneStep=nil
				end
				if g_nf==0 then
					saveDebugInfo(mSynthesis.simulator, "debugstates_comp/debugInfo_oneStep1.tbl")
				end
				mSynthesis:oneStepSimul()
				if g_nf==0 then
					saveDebugInfo(mSynthesis.simulator, "debugstates_comp/debugInfo_oneStepSimul1.tbl")
				end
				if g_debugOneStep then
					util.saveTable(g_debugOneStep, "debugstates_comp/debugStates_oneStepSimul1_"..g_nf..".tbl" )
				end
				mSynthesis:saveStates("debugstates_comp/debugStates1_"..g_nf..".tbl")
			else
				mSynthesis:oneStep()
				mSynthesis:oneStepSimul()
			end
			if error_feedback_method~=EFM.NONE then	 
				mSynthesis:prepareNextStep()      
			end
		
			if error_feedback_method==EFM.NONE or mSynthesis.errorOccurred then
				print("falldown!!!!")
				return
			end

			print(mSynthesis.numFrames)

			local curSeg = math.floor(mSynthesis.outputGlobal:getRefTime())
			
			if segGPPoints[curSeg]==nil then
				segGPPoints[curSeg] = vector3()
				segGPPoints[curSeg]:assign(mOsim:getGlobalPos('ground_pelvis', vector3(0,0,0)))
			end

			if prevFrac == nil then
				prevFrac=-1
			end

			function compareCurSegment(last)
				print(mSynthesis.numFrames)

				local scoreCoef = 10e8

				local segIndex = curSeg

				if not last then
					segIndex = segIndex -1
				end

				-- save marker points of segIndex segment
				local smp = vectorn()
				smp:assign(compareChain.points1)

				for i=0, smp:size()/3 - 1 do
					smp:setVec3(i*3, smp:toVector3(i*3) - segGPPoints[segIndex])
				end
				segMarkerPoints[segIndex] = smp

				local compareScore,nframes=compareChain:compareQueue() -- per segment
				--[[
				local compareScore = 5
				local nframes = 5
]]
				assert(compareScore~=nil)
				MSE=MSE+compareScore
--				print("cur seg 	:	"..seq.curSeg)
--				print("compareScore	:	"..compareScore)
--				print("nMSE	:	"..nMSE)
--				print("nframes	:	"..nframes)
				nMSE=nMSE+nframes
				
				-- dcjo
				poseDiff = poseDiff + compareScore
				poseAvgDiff = poseAvgDiff + compareScore/nframes
				
				local scorePerFrame = compareScore/nframes * scoreCoef
				if scorePerFrame > 10000 then
					mSynthesis.objectiveFunctionEnd = true
					errorOccurred = true
					poseDiffPenalty = poseDiffPenalty + 10000
				else
					poseDiffPenalty = poseDiffPenalty + scorePerFrame
				end

				print("poseDiff")
				print(poseDiff)
				print(poseAvgDiff)
				print(scorePerFrame)
				
				-- compare marker points of segIndex - 2 segment and segIndex segment
				if segMarkerPoints[segIndex-2] ~= nil then
					print("nf")
					print(segMarkerPoints[segIndex]:size())
					print(segMarkerPoints[segIndex-2]:size())
					local npoints = math.min(segMarkerPoints[segIndex]:size(),segMarkerPoints[segIndex-2]:size())
					print(npoints)
					local compareScore = compareChain.metric:calcDistance(segMarkerPoints[segIndex]:range(0,npoints-1), segMarkerPoints[segIndex-2]:range(0,npoints-1))
					local nframes = npoints/3

					poseDiff2 = poseDiff2 + compareScore
					poseAvgDiff2 = poseAvgDiff2 + compareScore/nframes

					local scorePerFrame = compareScore/nframes * scoreCoef
					if scorePerFrame > 10000 then
						mSynthesis.objectiveFunctionEnd = true
						errorOccurred = true
						poseDiffPenalty = poseDiffPenalty + 10000
					else
						poseDiffPenalty = poseDiffPenalty + scorePerFrame
					end
					print("====================")
					print("poseDiff2")
					print(poseDiff2)
					print(poseAvgDiff2)
					print(scorePerFrame)
				end
			end

			if mSynthesis.trackingInfo.frac < prevFrac then
				compareCurSegment()
			end
			compareChain:addQueue(outputMotion, mSynthesis)

			prevFrac = mSynthesis.trackingInfo.frac

			--dcjo skin test
			-- PLDPrimVRML inherited from PLDPrimSkin
--[[			skel_pos = vectorn()
			mSynthesis.skel_withoutCOMjoint:getPoseDOF(skel_pos)
			print("skel_pos")
			print(skel_pos(0))
	]]
--[[			local temp_vel = vectorn()
			mSynthesis.outputLocal:sampleVel(mSynthesis.outputGlobal:getRefTime(), temp_vel)
			print(temp_vel)

			local base_vel = vectorn(3)
			base_vel:assign(vector3(1,0,0))
			print(temp_vel*base_vel)
]]
--			print("COMlocal")
--			print(mSynthesis.outputLocal:sampleKey(mSynthesis.outputGlobal:getRefTime(), 'COMlocal'))
--			print("ZMPlocal")
--			print(mSynthesis.outputLocal:sampleKey(mSynthesis.outputGlobal:getRefTime(), 'ZMPlocal'))

			local currSeg = math.floor(mSynthesis.outputGlobal:getRefTime())
			local currRefPos = vectorn()
--[[		if currSeg > currSegIndex then
				local tempSkel = mSynthesis.skel
				currSegIndex = currSeg
				for k = 1,1 do
					for i=1, 1 do--0.05, 1, 0.05 do
						print(k+currSegIndex-1)
						mSynthesis.outputLocal:samplePose(k+currSegIndex-1, currRefPos)
						tempSkel:setPoseDOF(currRefPos)
						local temp = {}
						for j=0, 6 do
							temp[j] = currRefPos(j)
						end
						printtblh(temp)
						local temp_pelvis_pos = tempSkel:getBoneByName('ground_pelvis'):getFrame():toGlobalPos(vector3(0,0,0))
						print(temp_pelvis_pos)
					end
				end
			end
	]]	
			

			mSynthesis.outputLocal:samplePose(mSynthesis.outputGlobal:getRefTime(), currRefPos)
--[[		print(currRefPos(0))
			print(currSeg)
			print(mSynthesis.outputLocal.segInfo(currSeg))
			
			print(mSynthesis.outputLocal.segInfo(currSeg).len)
]]


			--[[			refSynthesis:oneStep()
			ref_skel_pos = vectorn()
			refSynthesis.skel_withoutCOMjoint:getPoseODF(ref_skel_pos)
			print("ref skel_pos")
			print(ref_skel_pos)
]]

			-- dcjo energy test
			if energy_test == true then
				local ml = {'med_gas_r', 'soleus_r', 'lat_gas_r'}
				local mlidxs = {}
				for i=1,#ml do
					local msclidxs = {mOsim:getMuscleIndexWithName(ml[i])}
					mlidxs[i] = msclidxs[1]
					local pM = mOsim:getPartialMetabolicEnergyRateOfMuscles(msclidxs)
					local output = ""..ifr
					for j=1, 5 do
						output = output.."\t"..pM[j]
					end
					util.outputToFile("printPartialMetabolicEnergy_"..ml[i]..".dat", output)
				end
			
				local pM = mOsim:getPartialMetabolicEnergyRateOfMuscles(mlidxs)
				local output = ""..ifr
				for j=1, 5 do
					output = output.."\t"..pM[j]
				end
				util.outputToFile("printPartialMetabolicEnergy_total.dat", output)

			end

			if false then --useCase.artificialJoints ~= nil then
				
				if refTimeTable == nil then
					refTimeTable = {}
				end
				-- ifr == mSynthesis.numFrames
				refTimeTable[ifr] = mSynthesis.outputGlobal.refTime(ifr-1)

				if jointTorqueResults == nil then
					jointTorqueResults = {}
				end
				jointTorqueResults[ifr] = mOsim.jointTorques

				if ifr%100 == 0 then
					util.saveTable(refTimeTable, "refTimeTable.tbl")
					util.saveTable(jointTorqueResults, "jointTorqueResults.tbl")
				end
			end

			if false then
				
				function writeJointTorques(jointList, torqueTypes)

					for _, jv in ipairs(jointList) do 

						local output = ""..mSynthesis.outputGlobal.refTime(ifr-1)
						local filename = "jointTorques_"..jv

						local joint = mOsim.jointTorques[mOsim.mLoader:getTreeIndexByName(jv)]

						for _, tv in ipairs(torqueTypes) do
							
							filename = filename..'_'..string.sub(tv,1,1)
							local torque = joint[tv]
		
							if torque ~= nil then 
								for _, v in ipairs(torque) do
									for _, vv in ipairs(vecn2tbl(v)) do
										output = output..'\t'..vv
									end
								end
							end

						end

						util.outputToFile(filename..".dat", output)
					end

				end



				local jointList = {'hip_r', 'hip_l', 'mtp_r'}

				if true then
					-- reftime, active, passive, contact, artificialjoint
					writeJointTorques(jointList, {"active", "passive", "contact", "artificial"})

				end

				if true then
					-- reftime, active
				end

				if true then
					-- reftime, passive

				end
				
				if true then
					-- reftime, contact
					
				end

				if true then
					-- reftime, artificialjoint

				end

			end

	
			if false then
				
				
				
				if artificialJointResults == nil then
					artificialJointResults = {}
				end
				artificialJointResults[mSynthesis.numFrames] = mOsim.artificialJoints
				for i, v in ipairs(mOsim.jointTorques) do 
					local output = ""..mSynthesis.outputGlobal.refTime(mSynthesis.numFrames-1)
					for t_i, t_v in ipairs(v["torque"]) do 
						output = output.."\t"..t_v
					end
					util.outputToFile("jointTorque_"..v["name"]..".dat", output)

				end

				util.saveTable(artificialJointResults, "artificialJointResult.tbl")
			end

			-- performance timer
			if ifr <= perfFrame then
				gTimerMuscle:pause()
				if ifr == perfFrame then
					local elapsed = gTimerMuscle:stop()
					print('avg. elapsed time for one frame (1/120 sec in realtime) in ms')
					print(elapsed/perfFrame)
					print('avg. scale to realtime')
					print((elapsed/perfFrame)/(1000000/120))
				end
			end
			------------------
			--energy print
			--1 energyRate = 1 J/s = (1/4.184) cal/s
			local energyRate = mOsim:getMetabolicEnergyRate()


			local kcalRate = energyRate*(1/4.184)*.001
			local met = (kcalRate*3600)/75
			g_kcal = g_kcal + kcalRate*(1/120.)
			g_int_met = g_int_met + met

			----print('energy rate', energyRate)
			----print('kcal rate', kcalRate)
			--print('kcal', g_kcal)
			----print('met', met)
			--print('avg met', g_int_met/ifr)

			local j = mOsim.mLoader:getBoneByName('ground_pelvis')
			local gp = mOsim.bfk:globalFrame(j):toGlobalPos(vector3(0,0,0))
			gp:setY(0)
			--dcjo
			--[[
			print("gp")
			print(gp)
			print(mOsim:getGlobalPos('ground_pelvis', vector3(0,0,0)))
			print(ifr)
			]]
			if ifr > 1 then
				local gp_diff = gp - prev_gp
				--[[
				print("gp_diff")
				print(gp_diff)
				print(math.acos(vector3.dotProduct(gp_diff, vector3(1,0,0))/gp_diff:length()))
				]]
			end
			prev_gp = gp
			--print('pelvis pos', gp)
			--print('move dist', gp:length())
			--print('int_met/movedist', g_int_met/gp:length())
			--print('kcal/movedist', g_kcal/gp:length())
			--print('move speed (m/s)', gp:length()/(ifr/120.))
			--print('move speed (km/h)', (gp:length()/(ifr/120.))*(1/1000.)*3600)

			g_rec_energy[ifr] = {}
			g_rec_energy[ifr].avgmet = g_int_met/ifr
			g_rec_energy[ifr].kcal = g_kcal
			g_rec_energy[ifr].kcaldist = g_kcal/gp:length()
			g_rec_energy[ifr].mps = gp:length()/(ifr/120.)
			g_rec_energy[ifr].kmph = (gp:length()/(ifr/120.))*(1/1000.)*3600
			------------------

			--render muscles
			if #int_as>0 then
				t_add_t_update(int_as, mOsim:getActivations())
			else
				int_as = mOsim:getActivations()
			end

			if g_plotdata and g_intgplot==false then
				mFile_l_mt:write(ifr..'\t')
				for m,l_mt in ipairs(mOsim:getTendonMuscleLengths()) do
					mFile_l_mt:write(l_mt..'\t')
				end
				mFile_l_mt:write('\n')

				mFile_act:write(ifr..'\t')
				for m,act in ipairs(mOsim:getActivations()) do
					mFile_act:write(act..'\t')
				end
				mFile_act:write('\n')

				local posedof = vectorn()
				mOsim.bfk:getPoseDOFfromGlobal(posedof)
				mFile_angle:write(ifr..'\t')
				for j,angIndex in ipairs(mAngleIndices) do
					local angValue
					if angIndex==mOsim.rkneeDOF then
						angValue = -posedof(angIndex)
					elseif angIndex==-1 then	--pelvis_roll
						local rootTF=MotionDOF.rootTransformation(posedof)
						local rotY=rootTF.rotation:rotationY()
						local upward=vector3(0,1,0)
						upward:rotate(rootTF.rotation)
						local localRot=rotY:inverse()*rootTF.rotation
						local roll=localRot:rotationAngleAboutAxis(vector3(1,0,0))	    
						angValue = roll
					else
						angValue = posedof(angIndex)
					end
					mFile_angle:write(math.deg(angValue)..'\t')
				end
				mFile_angle:write('\n')

				mFile_grf:write(ifr..'\t')
				local rcf = mOsim:getRFootCF()
				local lcf = mOsim:getLFootCF()
				mFile_grf:write(rcf.x..'\t')
				mFile_grf:write(rcf.y..'\t')
				mFile_grf:write(rcf.z..'\t')
				mFile_grf:write(lcf.x..'\t')
				mFile_grf:write(lcf.y..'\t')
				mFile_grf:write(lcf.z..'\t')
				mFile_grf:write('\n')
			end

			--rendering
			if ifr % render_interval == 0 then
				render_as = t_div_s(int_as, render_interval)
				int_as = create_array(mOsim:getNumMuscles(), 0.)

				if g_plotdata and g_intgplot then
					mFile_l_mt:write(ifr..'\t')
					for m,l_mt in ipairs(mOsim:getTendonMuscleLengths()) do
						mFile_l_mt:write(l_mt..'\t')
					end
					mFile_l_mt:write('\n')

					mFile_act:write(ifr..'\t')
					for m,act in ipairs(render_as) do
						mFile_act:write(act..'\t')
					end
					mFile_act:write('\n')

					local posedof = vectorn()
					mOsim.bfk:getPoseDOFfromGlobal(posedof)
					mFile_angle:write(iframe..'\t')
					for j,angIndex in ipairs(mAngleIndices) do
						local angValue
						if angIndex==mOsim.rkneeDOF then
							angValue = -posedof(angIndex)
						elseif angIndex==-1 then	--pelvis_roll
							local rootTF=MotionDOF.rootTransformation(posedof)
							local rotY=rootTF.rotation:rotationY()
							local upward=vector3(0,1,0)
							upward:rotate(rootTF.rotation)
							local localRot=rotY:inverse()*rootTF.rotation
							local roll=localRot:rotationAngleAboutAxis(vector3(1,0,0))	    
							angValue = roll
						else
							angValue = posedof(angIndex)
						end
						mFile_angle:write(math.deg(angValue)..'\t')
					end
					mFile_angle:write('\n')

				end
			else
				if #render_as==0 then
					render_as = mOsim:getActivations()
				end
			end

--			mOsim:drawMuscles(ifr, mObjectList, 1, render_as)
			local muscleList = {'rect_fem_l', 'rect_fem_r', 'soleus_l', 'soleus_r', 'lat_gas_l', 'lat_gas_r', 'med_gas_l', 'med_gas_r'}
		--	muscleList = {'tib_post_r', 'flex_dig_r', 'flex_hal_r', 'tib_ant_r', 'per_brev_r', 'per_long_r', 'per_tert_r', 'ext_dig_r', 'ext_hal_r', 'med_gas_r', 'lat_gas_r', 'soleus_r'}
--			muscleList = {'grac_r'}
			mOsim:drawSomeMuscles(ifr, mObjectList, 1, render_as, muscleList) 

			--mOsim:drawMuscles(ifr, mObjectList)
			--mOsim:drawMuscles2(ifr, mObjectList)
			--mOsim:drawJoints(iframe, mObjectList)|||     muscledyn
			-- 567                     table.insert(self.eppis[i_muscle], i_pp)                                                    
			--mOsim:drawMuscleWithForces(ifr, mObjectList)
			
			local ml = {}--{'soleus_r', 'lat_gas_r', 'med_gas_r'}
		
--			util.outputToFile("tendonForces.dat", ifr)
			local tendonForcesData = ""
			tendonForcesData = tendonForcesData..ifr
			for i=1,#ml do
				print(ml[i])
				print(mOsim:getTendonForce(mOsim:getMuscleIndexWithName(ml[i])))
				if mOsim:getTendonForce(mOsim:getMuscleIndexWithName(ml[i]))~= nil then
					tendonForcesData = tendonForcesData.."\t"..mOsim:getTendonForce(mOsim:getMuscleIndexWithName(ml[i]))
					--util.outputToFile("tendonForces.dat", "	"..mOsim:getTendonForce(mOsim:getMuscleIndexWithName(ml[i])))
				end
			end
			util.outputToFile("tendonForces.dat", tendonForcesData)
			
			
			if boolean_options.attachCamera then
				local curPos= mSynthesis.pendulum:calcCOMpos()*100
			   curPos.y=0
			   recCurPos[ifr] = curPos
		--	   RE.viewpoint().vpos:assign(mCameraInfo.vpos+curPos)
		--	   RE.viewpoint().vat:assign(mCameraInfo.vat+curPos)
				if recCurPos[ifr-1] ~= nil then
					RE.viewpoint().vpos:assign(RE.viewpoint().vpos + curPos - recCurPos[ifr-1])
					RE.viewpoint().vat:assign(RE.viewpoint().vat + curPos - recCurPos[ifr-1])
				else
					RE.viewpoint().vpos:assign(mCameraInfo.vpos+curPos)
					RE.viewpoint().vat:assign(mCameraInfo.vat+curPos)
				end
				--print(RE.viewpoint().vpos)
				--print(RE.viewpoint().vat)
				RE.viewpoint():update()     
			end

			----test lfootcf
			--g_lfootcf_temp = g_lfootcf_temp + mOsim:getLFootCF():length()
			--g_rfootcf_temp = g_rfootcf_temp + mOsim:getRFootCF():length()
			--print('l', g_lfootcf_temp, 'r', g_rfootcf_temp)
			----print('l-r', g_lfootcf_temp-g_rfootcf_temp)

			----test ankpf force
			--g_lankpff_temp = g_lankpff_temp + mOsim:getLAnkPFForce()
			--g_rankpff_temp = g_rankpff_temp + mOsim:getRAnkPFForce()
			--print('lankpf', g_lankpff_temp, 'rankpf', g_rankpff_temp)

			----test lkneeang, rkneeang
			--local lexcess = 0
			--local rexcess = 0
			--if mOsim:getLKneeExtAng() > -20 then
				--lexcess = mOsim:getLKneeExtAng() - (-20)
			--end
			--if mOsim:getRKneeExtAng() > -20 then
				--rexcess = mOsim:getRKneeExtAng() - (-20)
			--end
			--g_lrknee_temp = g_lrknee_temp + lexcess + rexcess
			--print('lkneeang', mOsim:getLKneeExtAng())
			--print('rkneeang', mOsim:getRKneeExtAng())
			--print('lrkneelim', g_lrknee_temp)

			--test lhipang, rhipang
			--print('lhipext', mOsim:getLHipExtAng())
			--print('rhipext', mOsim:getRHipExtAng())
			--if mOsim:getLHipExtAng() > -10 then
				--lexcess = mOsim:getLHipExtAng() - (-10)
				--print(lexcess)
			--end

			g_simulated_ifr = ifr

			--when falldown
			if mSynthesis.errorOccurred==true or error_feedback_method==EFM.NONE then	 
				print("fall down")
				return
			end
		end	
	else
		if boolean_options.attachCamera then
			local curPos = recCurPos[iframe]
			if curPos~= nil then
				if recCurPos[g_prev_ifr] ~= nil then
					RE.viewpoint().vpos:assign(RE.viewpoint().vpos + curPos - recCurPos[g_prev_ifr])
					RE.viewpoint().vat:assign(RE.viewpoint().vat + curPos - recCurPos[g_prev_ifr])
				else
					RE.viewpoint().vpos:assign(mCameraInfo.vpos+curPos)
					RE.viewpoint().vat:assign(mCameraInfo.vat+curPos)
				end
				RE.viewpoint():update()
			end
				
--[[	if curPos~=nil then
				RE.viewpoint().vpos:assign(mCameraInfo.vpos+curPos)
				RE.viewpoint().vat:assign(mCameraInfo.vat+curPos)
				RE.viewpoint():update()     
			end			]]
		end
		if self.trajectory then
			local ifr = iframe
			if ifr<self.trajectory:rows() then
				local curPos=self.trajectory:row(ifr):toVector3(0)*100
				local currRot=self.trajectoryOri:row(ifr):toQuater(0):rotationY()

				local tf=transf()
				tf:identity()
				tf:leftMultTranslation(curPos*-1)
				local qd=quater()
				qd:difference(self.cameraInfo.refRot, currRot)
				tf:leftMultRotation(qd)
				tf:leftMultTranslation(curPos)

				RE.viewpoint().vpos:assign(tf*(self.cameraInfo.vpos+curPos))
				RE.viewpoint().vat:assign(tf*(self.cameraInfo.vat+curPos))
				RE.viewpoint():update()     
			end
		end
	
		-- recoreded muscle rendering
		mOsim:drawRecordedScene(iframe, mObjectList)

		if g_rec_impulse_t[iframe]~=nil then
			local t = g_rec_impulse_t[iframe]
			mSynthesis.impulseGizmo=mSynthesis.objectList:registerEntity("arrow2", "arrow2.mesh")
			mSynthesis.impulseGizmo:transform(t)
		else
			mSynthesis.objectList:erase("arrow2")
			mSynthesis.impulseGizmo=nil
		end
	end

	if g_rec_energy[iframe]~=nil then
		local avgmet = g_rec_energy[iframe].avgmet
		local cal = g_rec_energy[iframe].kcal*1000.
		local calpm = g_rec_energy[iframe].kcaldist*1000.
		local mps = g_rec_energy[iframe].mps
		local kmph = g_rec_energy[iframe].kmph

		local caption=''
		if g_cap_met then
			caption=caption..string.format("avg.met.    %.1f\n", avgmet)
		end
		if g_cap_eng then
			caption=caption..string.format("energy     %.1f cal\n", cal)
		end
		if g_cap_eff then
			caption=caption..string.format("energy/dist %.1f cal/m\n", calpm)
		end
		if g_cap_spd then
			caption=caption..string.format("speed      %.1f km/h", kmph)
		end

		--titlebar:setCaption(string.format("avg.met    %.1f\nenergy     %.1f cal\nenergy/dist %.1f cal/m\nspeed      %.1f km/h",
							--avgmet, cal, calpm, kmph))
		titlebar:setCaption(caption)
	end

	g_prev_ifr = iframe

end


