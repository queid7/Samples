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
--	modelChooser:createMenu("load a predefined model")
--	this:create('Button', 'attach camera', 'attach camera')
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

	mSynthesis:saveStates("testPushRecovery.dat")

	print("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!1")

	local function OneStep()
		
		mSynthesis:oneStep()

		if mSynthesis.errorOccurred then
			return false
		end
		mSynthesis:oneStepSimul()

		if mSynthesis.errorOccurred then
			return false
		end

		if error_feedback_method == EFM.NONE then
			return false
		else
			mSynthesis:prepareNextStep()
		end

		if mSynthesis.errorOccurred then
			return false
		end
		return true
	end


	local numFrame = 800

	local numTest = 10
	local impulseMagList = {100, 100, 100, 100, 100, 100, 100, 100, 100, 100}
	
	for i=1,10 do
		impulseMagList[i] = 80
	end


	local impulseDir = vector3(1,0,0)
	local impulseDuration = 0.2

	local numError = 0

	for t=1, numTest do
		print(t.."-th test	")

		local pushFrame = math.random(200) + 200
--		pushFrame = 5
		
		for f=1, numFrame do

			if f == pushFrame then
				mSynthesis.impulse = impulseDuration*model.simulationFrameRate
				mSynthesis.impulseDir = impulseDir*impulseMagList[t]
				print("PUSH!!!!")
			end
	
			local pcall_ok, errMsg=pcall(OneStep)
			if pcall_ok == false then
		
				print("f	"..f)
				print("error at "..mSynthesis.numFrames.."-th frame")
				print("????????????????????????????????????????????????????????????????????????????????/")

				numError = numError + 1
				break
			else
				if errMsg~=true then
					print("f	"..f)
					print("error at "..mSynthesis.numFrames.."-th frame")
					print("i+_+_+_+_+++_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+_+__+_+_+_+_+_+_+_+")

					numError = numError + 1
					break
				end

			end

		end

		mSynthesis:restoreStates("testPushRecovery.dat")


	end

	print("numTest	: "..numTest)
	print("numError	: "..numError)
	print("success rate	: "..(1-numError/numTest))



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

--[[
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
	end
	]]
end


function frameMove(fElapsedTime)
	--mSynthesis:oneStep()
	--mSynthesis:oneStepSimul()
	--if error_feedback_method~=EFM.NONE then	 
		--mSynthesis:prepareNextStep()      
	--end
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
	if g_simulated_ifr < 130 then
		return
	end

	local s=0
	local e=g_simulated_ifr


	self.trajectory=matrixn(e,3)
	self.trajectoryOri=matrixn(e,4)

	for f=s,e-1 do
		self.trajectory:row(f):setVec3(0, MotionDOF.rootTransformation(mOsim.records[f+1]["bfkPoseDOF"]).translation)
		self.trajectory:row(f):set(1,0)
		self.trajectoryOri:row(f):setQuater(0, MotionDOF.rootTransformation(mOsim.records[f+1]["bfkPoseDOF"]).rotation:rotationY())
	end
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

function EVR:onFrameChanged(win, iframe)

	local numTest = 10;
	local testValue = {10, 20, 30, 40, 50, 60, 70, 80, 90, 100};

	for i=1, numTest do










	end



--	mSynthesis:oneStep()
--	mSynthesis:oneStepSimul()
	if error_feedback_method~=EFM.NONE then	 
--		mSynthesis:prepareNextStep()      
	end




--[[
	if iframe > g_simulated_ifr then
		local prev_simulated_ifr = g_simulated_ifr

		for f=1,iframe-prev_simulated_ifr do
			local ifr = prev_simulated_ifr + f

			local perfFrame = 100
			print()
			print('simulate frame ', ifr)

			-- performance timer
			if ifr <= perfFrame then
				gTimerMuscle:start()
			end

			--impulse in useCase
			if useCase.impulse~=nil then
				for i=1,#useCase.impulse do
					local chest=mSynthesis.skel_withoutCOMjoint:VRMLbone(10)
					local imp=useCase.impulse[i]

					--impulse simul
					if ifr==imp.startframe then
						mSynthesis.impulse=imp.dur*model.simulationFrameRate
						mSynthesis.impulseDir=imp.dir*imp.mag
						mSynthesis.impulseGizmo=mSynthesis.objectList:registerEntity("arrow2", "arrow2.mesh")
					end

					--impulse rendering save
					if ifr>=imp.startframe and ifr<imp.startframe+imp.dur*model.simulationFrameRate then
						local frame=mSynthesis.simulator:getWorldState(0):globalFrame(chest)
						local gf=mSynthesis.impulseDir
						local dir=gf:copy()
						dir:normalize()
						local t=transf()
						t:axisToAxis(vector3(0,0,0), vector3(0,1,0), frame.translation*100-10*dir, dir)
						g_rec_impulse_t[ifr] = t:copy()
					end
				end
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

			local currSeg = math.floor(mSynthesis.outputGlobal:getRefTime())
			local currRefPos = vectorn()
			if currSeg > currSegIndex then
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
			if ifr > 1 then
				local gp_diff = gp - prev_gp
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
			mOsim:drawSomeMuscles(ifr, mObjectList, 1, render_as, muscleList) 

			--mOsim:drawMuscles(ifr, mObjectList)
			--mOsim:drawMuscles2(ifr, mObjectList)
			--mOsim:drawJoints(iframe, mObjectList)
			--mOsim:drawMuscleWithForces(ifr, mObjectList)
			
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
]]
end
