require("subRoutines/motiongraph")

scenario=useCase.scenario

--dbg.startDebug()
iterativeIK={ drawDebugPose=false, nIter=5 }

if useCase.referenceScenario==nil then
   referenceScenario=scenario
else
   referenceScenario=useCase.referenceScenario
end


require("IPC_based/LocoGraph")

package.path=package.path..";../Samples/ysscripts/lib/?.lua" --;"..package.path
--require("IPC_based/LocoSimulation")
require("LocoSimMuscle")

require('IPC_based/trajGenerator')


function OnlineLocoSynthesis:endTrigger(prevSeg, currSeg)
   self.objectiveFunctionEnd=true
   print("endTriggered")
end

Tracker=LUAclass()




function Tracker.objCost_general(chain2d, seg, penalty, skipCriteria)
	local mrdMotion=chain2d.mrdMotion
	local frames=chain2d.frames
	local targets={'L','R'}
	local name=string.lower(seg.name)
	local allow=3
	-- swingfoot error
	local error=0
	for it, target in ipairs(targets) do
		if frames:back()-frames(0) >allow*2 then
			if seg['swing'..target]~=0 then
				-- swing foot error
				error=error+mrdMotion['con'..target]:range(frames(0)+allow, frames:back()+1-allow):count()*1e-5
			else
				-- support foot error
				local st=frames(0)+allow
				local ed=frames:back()+1-allow
				error=error+math.abs(ed-st-mrdMotion['con'..target]:range(st,ed):count())*1e-5
			end
		end
	end

	return error, skipCriteria
end
function Tracker.objCost_backflip(chain2d, seg, penalty)
	return Tracker.objCost_general(chain2d, seg, penalty, nil)
end
function Tracker.objCost_roundoff2(chain2d, seg, penalty)
	local name=string.lower(seg.name)
	local skipCriteria= not (name=="l2" or name=="l" or name=='l3' or name=='l4') 
	return Tracker.objCost_general(chain2d, seg, penalty, skipCriteria)
end

function Tracker.objCost_walk(chain2d, seg,penalty)
	local mrdMotion=chain2d.mrdMotion
	local frames=chain2d.frames
	local name=seg.name
	local allow=3
	local skipCriteria= not (name=="L" or name=="l") 
	--print('skip',skipCriteria)
	-- local cdbg
	-- if g_cdbg then
	-- 	cdbg=g_cdbg
	-- 	array.pushBack(cdbg,{"objcost", seg.name, frames:copy()})
	-- end
	-- swingfoot error
	if frames:back()-frames(0) >allow*2 then
		local conL=mrdMotion.conL:range(frames(0)+allow, frames:back()+1-allow)
		local conR=mrdMotion.conR:range(frames(0)+allow, frames:back()+1-allow)
		-- if cdbg then
		-- 	array.pushBack(cdbg,{"objcost", conL:count(), conR:count()})
		-- end

		if name=="L" then
			--print("L finished")
			--print(penalty)
			--print(conR:count(), conR, conL)
			return conR:count()*1e-5+conL:count(false)*1e-5,skipCriteria
		elseif name=="R" then
			--print("R finished")
			----print(penalty)
			--print(conL:count(), conL, conR)
			return conL:count()*1e-5+conR:count(false)*1e-5 ,skipCriteria
		end
	end
	-- spprtfoot error
	if frames:back()-frames(0) >allow*2 then
		local st=frames(0)+allow
		local ed=frames:back()+1-allow
		local conL=mrdMotion.conL:range(st, ed)
		local conR=mrdMotion.conR:range(st, ed)
		-- if cdbg then
		-- 	array.pushBack(cdbg,{"objcost", st,ed,conL:count(), conR:count()})
		-- end
		local err=0
		if name=="L" or name=="LR" or name=="RL" then
			err=conL:count(false)*1e-5 
		end

		if name=="R" or name=="LR" or name=="RL" then
			err=err+conR:count(false)*1e-5
		end
		return err,skipCriteria
	end

	return 0,skipCriteria
end



function Tracker:__init()
	--setInitialPose

	self.synthesis=OnlineLocoSynthesis:new()

	--self.synthesis:saveStates("debugStates_arff.tbl")
	--dbg.console()
	--mrd_info.outputMotion=nil -- exporting motion to a file is prohibitive in optimization.
	mrd_info.export_freq=1000000000
	mrd_info.outputContactForce=nil -- exporting contact force to a file is prohibitive in optimization.

	local function setDefaultRefSeg()

		local graph=self.synthesis.graph
		for i,grpName in ipairs(graph.groups.tf) do
			for k,v in pairs(graph[grpName]) do
				v.refSeg=v
			end
		end
	end
	setDefaultRefSeg()

	--if scenario==scenarios.JUMP_UP and  referenceScenario==scenarios.JUMP_FORWARD2 then
		--self.synthesis.graph.hopping1.jump_down.transitions.jump_after.trigger=self.synthesis.endTrigger
		----	    self.synthesis.graph.stand.stand_short.transitions.jump_up.trigger=self.synthesis.endTrigger
--
		--local graph=self.synthesis.graph
		--local v
		--v=graph.stand.stand_short
		--v.refSeg=v
--
		--for k,v in pairs(graph.hopping1) do
			--v.refSeg=graph.hopping2[k]
			--assert(v~=nil)
		--end
	--elseif scenario==scenarios.RUN then
		--local motions=self.synthesis.graph.motions
		--self.sequence={motions.SR, motions.RD, motions.DLF, motions.FRF, motions.FLF,motions.FRF, motions.FLF, N=7, curSeg=1}	 
	--elseif useCases.isRun(useCase) then
		--local STP=self.synthesis.graph.STP
		--local StR=self.synthesis.graph.StR
		--local run=self.synthesis.graph.run
		--self.sequence={STP.ss, STP.st, StR.SR, StR.RL, StR.LR,
		--run.RL, run.LR, run.RL, run.LR,
		--run.RL, run.LR, run.RL, run.LR,
		--run.RL, run.LR, run.RL, run.LR,
		--run.RL, run.LR, run.RL, run.LR,
		--run.RL, run.LR, run.RL, run.LR,
		--run.RL, run.LR, run.RL, run.LR,
		--run.RL, run.LR, run.RL, run.LR,
		--run.RL, run.LR, run.RL, run.LR,
		--run.RL, run.LR, run.RL, run.LR,
		--run.RL, run.LR, run.RL, run.LR,
		--run.RL, run.LR, run.RL, run.LR,
		--run.RL, run.LR, run.RL, run.LR,
		--run.RL, run.LR, run.RL, run.LR,
		--run.RL, run.LR, run.RL, run.LR,
		--curSeg=1}
--
		--self.sequence.N=table.getn(self.sequence)
		--self.sequence[0]=self.synthesis.graph.STP.ss

	--else
do
		--self.synthesis.graph.stand.stand_short.transitions.stand_to_jump.trigger=self.synthesis.endTrigger
		self.sequence={}
		local currSeg=self.synthesis.graph.initialSegment
		array.pushBack(self.sequence, currSeg)
		for i=1, 80 do
			array.pushBack(self.sequence, currSeg.next)
			currSeg=currSeg.next
		end

		self.sequence.N=table.getn(self.sequence)
		self.sequence[0]=self.synthesis.graph.initialSegment
	end

	local stage=1 -- one indexing unlike previous classes

	self:saveStates({0})
	self:saveStates({stage})

	self.numSubStage=1 -- substage is unused. only for backward-compatibility

	if useCase.genStageParamInitial then
		self.stageParam=useCase:genStageParamInitial()
	else
		self.stageParam=useCase.stageParamInitial
	end
	--elseif useCase.controlParam==useCase.controlParamSet.fast  then 
	--self.stageParam=
	--{
	--{startSeg=1, endSeg=40, nvar=2,setStageFunc="setStage_xyz_run_short", baseStage=1,
	--desiredSpeedZ={0,0,0,0,0,0.4,0.8,1.2, 1.6, 2.0}
	----	  desiredSpeedZ={0,0,0,0,0,0,0,0.1,0.4,0.6,0.8,1.0, 1.2, 1.4, 1.6, 2.0}
	--},

	self.numStage=table.getn(self.stageParam)

	--   self:setStage(stage,0)

	self.skin=RE.createVRMLskin(self.synthesis.skel_withoutCOMjoint, false)
	self.skin:setTranslation(-150,0,0)
	self.skin:scale(100,100,100)
end


function Tracker:decr(tbl)
   local stage=tbl[1]
   assert(tbl[2]==0)
   stage=stage-1
   return {stage,0}
end

function Tracker:incr(tbl)
   local stage=tbl[1]
   assert(tbl[2]==0)
   return {stage+1,0}
end

-- accumulate until self.stage==stage and self.subStage==subStage
function Tracker:accumulateResult(tbl)
	local stage=tbl[1]
	for ii=1,stage do
		cp_mod=nil
		if rank==nil then
			dofile("optresult"..ii.."_0.lua")
		else
			dofile("optresult"..ii.."_rank"..rank..".lua")
			--dofile("optresult"..ii.."_rank0.lua")
		end
		assert(cp_mod~=nil)
		self.synthesis.graph:changeControlParametersInUseCase(cp_mod)
		useCases.unmapControlParam(useCase)
		self.synthesis:changeControlParameters(cp_mod)
	end
end

-- function Tracker:setStage(i, subi)

--    print(self.stageParam[i].setStageFunc)
--    local func=Tracker[self.stageParam[i].setStageFunc]
--    func(self, i, subi)
-- end

function Tracker:updateConstraints()

   self.synthesis:__updateConstraints()
--   self.synthesis.graph:preprocessPendulumMomentum()

end

function Tracker:__finalize()
   self.synthesis:__finalize()
   self.synthesis=nil
   self.compareChain=nil
   skin=nil

   collectgarbage("collect")
   collectgarbage("collect")
   collectgarbage("collect")
end

function Tracker:restoreStates(stage, subStage)
	--print('Tracker:restoreStates()', stage)
	if rank==nil then
		self.synthesis:restoreStates("states_"..stage..".dat")
	else
		self.synthesis:restoreStates("states_"..stage.."_rank"..rank..".dat")
	end
end

function Tracker:saveStates(stt)
	--print('Tracker:saveStates()', stt[1])
	if rank==nil then
		self.synthesis:saveStates("states_"..stt[1]..".dat")
	else
		self.synthesis:saveStates("states_"..stt[1].."_rank"..rank..".dat")
	end
end

----ys
--function Tracker:saveStatesAllRanks(stage)
	--print('Tracker:saveStatesAllRanks()', stage)
	--local numRank = MPI.size()
	--for i=0,numRank-1 do
		--self.synthesis:saveStates("states_"..stage.."_rank"..i..".dat")
	--end
--end

function Tracker:__prepareNextStage()

	-- proceed first two frames seperately. This is to avoid immediate trigger.

	local errorOccurred=false

	local function OneStep()
		self.synthesis:oneStep()
		if self.synthesis.errorOccurred==true then
			fineLog( "error0")
			return false
		end

		self.synthesis:oneStepSimul()

		if self.synthesis.errorOccurred==true then
			fineLog( "error1")
			return false
		end

		if error_feedback_method==EFM.NONE then
			fineLog( "error2")
			return false
		end

		if error_feedback_method~=EFM.NONE then	 
			self.synthesis:prepareNextStep()      
		end

		if self.synthesis.errorOccurred==true then
			fineLog( "error3")
			return false
		end
		return true
	end

	do
		self.synthesis.objectiveFunctionEnd=false

		local i=0
		local prevFrac=-1
		while self.synthesis.objectiveFunctionEnd==false do

			if not OneStep() then 
				errorOccurred=true
				break
			end


			i=i+1

			local event=self:updateCurrSeg(i, self.synthesis, self.nextStartSeg)
			if event then
				if event=="normal ending" then
					local stt=self:incr({self.stage, self.subStage})
					assert(stt[1]~=self.stage)
					self:saveStates(stt)
					--		  RE.output("prepareNextStage", "done"..seq.curSeg)
				end
				break
			end

			if math.mod(i,renderFreq)==0 then
				--	    if true then
				self.synthesis.simulator:drawDebugInformation()
				renderOneFrame()
			end

		end
	end
	assert(errorOccurred==false)
	collectgarbage("collect")
	collectgarbage("collect")
	collectgarbage("collect")

end

function Tracker:prepareNextStage()
	local i=self.stage
	local baseStage=self.stageParam[i+1].baseStage
	local baseStartSeg
	if baseStage==0 then
		baseStartSeg=1
	else
		baseStartSeg= self.stageParam[baseStage].startSeg
	end
	local nextStartSeg= self.stageParam[i+1].startSeg
	self:_prepareNextStage(baseStage, baseStartSeg, nextStartSeg)
end

function Tracker:_prepareNextStage(baseStage, baseStartSeg, nextStartSeg)
	self.nextStartSeg=nextStartSeg

   fineLog( "prepareNextStage",self.stage, self.subStage)

   renderFreq=4

   local graph=self.synthesis.graph


   local seq=self.sequence

 
   seq.curSeg=baseStartSeg
   
   if self.nextStartSeg>=seq.curSeg then
	   self:restoreStates(baseStage, 0)
   else
	   self:restoreStates(0,0)
	   seq.curSeg=1
	   self.endSeg=10000000
   end
   RE.output("prepareNextStage", seq.curSeg.." "..(self.nextStartSeg))


   self.synthesis:changeControlParameters(useCase.controlParam)
   -- simulation 진행후 다음 스테이지 시작상태 저장.
   self:accumulateResult({self.stage, self.subStage})

   self:updateConstraints()

   if seq.curSeg==nextStartSeg then
      self:saveStates({self.stage+1,0})
   else
      self:__prepareNextStage()
   end
end


function Tracker:playback(stage, subStage, bFinal, bAccumulate)
	-- set default parameters
	bFinal=bFinal or false


	fineLog( "playback",stage, subStage)

	local errorOccurred=false

	renderFreq=4


	local graph=self.synthesis.graph



	local seq=self.sequence

	self.stage=stage
	self.subStage=subStage

	self:setDomain()
	print("setStage", self.startSeg, self.endSeg, self.subStage)

	seq.curSeg=self.startSeg

	RE.output("prepareNextStage", seq.curSeg.." "..(seq.curSeg+self.nextStartSeg))
	self:restoreStates(stage, subStage)


	self.synthesis:changeControlParameters(useCase.controlParam)
	if bAccumulate then

		print("accumulating opt_results. if okay, type cont")
		dbg.console()
		self:accumulateResult({stage, subStage})
	end
	self:updateConstraints()


	-- proceed first two frames seperately. This is to avoid immediate trigger.

	local function OneStep()
		self.synthesis:oneStep()
		self.synthesis:oneStepSimul()

		if self.synthesis.errorOccurred==true then
			fineLog( "error1")
			return false
		end

		if error_feedback_method==EFM.NONE then
			fineLog( "error2")
			return false
		end

		if error_feedback_method~=EFM.NONE then	 
			self.synthesis:prepareNextStep()      
		end

		if self.synthesis.errorOccurred==true then
			fineLog( "error3")
			return false
		end
		return true
	end

	for i=1,2 do
		if not OneStep() then
			errorOccurred=true 
			break
		end
	end

	if not errorOccurred then
		self.synthesis.objectiveFunctionEnd=false

		local i=0
		local prevFrac=-1
		while self.synthesis.objectiveFunctionEnd==false do

			if not OneStep() then 
				errorOccurred=true
				break
			end

			if math.mod(i,renderFreq)==0 then

				if boolean_options.attachCamera then
					local curPos= self.synthesis.pendulum:calcCOMpos()*100
					curPos.y=0
					RE.viewpoint().vpos:assign(mCameraInfo.vpos+curPos)
					RE.viewpoint().vat:assign(mCameraInfo.vat+curPos)
					RE.viewpoint():update()     
				end

				self.synthesis.simulator:drawDebugInformation()
				renderOneFrame()
			end

			i=i+1

			local event
			if bFinal then
				event=self:updateCurrSeg(i, self.synthesis, self.endSeg)
			else
				event=self:updateCurrSeg(i, self.synthesis, self.nextStartSeg)
			end

			if event then break end
		end
	end
	assert(errorOccurred==false)
	collectgarbage("collect")
	collectgarbage("collect")
	collectgarbage("collect")
end


function Tracker:objectiveFunction(pos, renderFreq)
	local objectiveFunctionTypes={POSE=1, SEGMENT=2, POSE2=3}
	local objectiveFunctionType=objectiveFunctionTypes.SEGMENT
	local markerFile
	do
		local fn, path=os.processFileName(string.sub(model.file_name ,1, -5))
		if useCase.markerOption == nil then
			markerFile=path.."/"..fn.."_sd/optComparision_marker.lua"
		else
			markerFile=path.."/"..fn.."_sd/optComparision_marker_"..useCase.markerOption..".lua"
		end
	end

	mrd_info.outputMotion=MotionDOFcontainer(self.synthesis.skel_withoutCOMjoint.dofInfo)
		-- QP_controller/lua/subRoutines/CompareChain.lua
		self.compareChain=CompareChain2D(self.synthesis.skel_withoutCOMjoint,
		--self.compareChain=CompareChain2Dori(self.synthesis.skel_withoutCOMjoint,
	self.synthesis.simulator:getWorldState(0),
	self.synthesis.skel_withoutCOMjoint:fkSolver(), markerFile)
	self.compareChain.baseBoneName = "ground_pelvis"	-- calculate marker position from baseBone

	local origDMot=MotionDOF(self.synthesis.skel_withoutCOMjoint.dofInfo) -- original motion that is temporally aligned to the synthesized motion
	local origMot=MotionDOF(self.synthesis.skel_withoutCOMjoint.dofInfo) -- original motion that is temporally aligned to the synthesized motion

	--ys : integrated metabolic energy rate
	local int_energy = 0
	local int_lfootcf = 0
	local int_rfootcf = 0
	local int_lankforce = 0
	local int_rankforce = 0
	local int_lkneeExtExcess = 0
	local int_rkneeExtExcess = 0
	local int_lhipExtExcess = 0
	local int_rhipExtExcess = 0

	-- dcjo : number of excessively activated muscles
	local nEAM = 0
	-- dcjo : other terms
	local diffDir = 0 -- difference of direction btw origin motion and simulation. max 10000 per seg
	-- dcjo : for debug
	-- poseDiff : pose diffrence to ref pose
	local poseDiff = 0
	local poseAvgDiff = 0
	local totalEffort = 0
	local totalEffortPerDist = 0

	-- dcjo
	-- save marker points of a simulated motion
	-- segMarkerPoints[segment_number] = marker points of a segment
	local segMarkerPoints = {}
	-- segGPPoints[segment_number] = a ground_pelvis point of a segment
	local segGPPoints = {}
	-- poseDiff2 : pose diffrence to previous step pose( two segments ago)
	local poseDiff2 = 0
	local poseAvgDiff2 = 0
	local objFn = 0
	-- poseDiffPenalty : total penalty of pose diffrences (include poseDiff1, poseDiff2)
	local poseDiffPenalty = 0

	local MSE=0
	local MAX_E=0
	local nMSE=0
	local kenergy=0
	local errorOccurred=false
	local skin=self.skin
	local debugMode=false
	if debugMode then
		require('IPC_based/testmw_save')
		min_f=15
		max_f=20
	end
	do

		RE.output("pos", tostring(pos))
		local graph=self.synthesis.graph

		local seq=self.sequence
		seq.curSeg=self.startSeg

		--print("objectiveFunction")
		fineLog("objectiveFunction", self.stage, self.subStage, self.startSeg, self.endSeg)
		do
			local tbl={}
			for i=0, pos:size()-1 do
				local title=opt_dimension[i+1].title
				tbl[title]=pos(i)
			end
			fineLog(table.tostring(tbl))
		end

		local bRestoreStateSkipped=false

		self:restoreStates(self.stage, self.subStage)
		if debugMode then
			if g_i==nil then
				g_i=0
				os.execute('rm debugStates*.tbl')
				os.execute('rm debugInfo*.tbl')
			end
			self.synthesis:saveStates("debugStates_arbc1"..g_i..".tbl")
		end

		-- dcjo debug
		local testPosString = ""
		for i=0, pos:size()-1 do
			local title=opt_dimension[i+1].title
			testPosString = testPosString.."['"..title.."']".."= "..pos(i)..", "
		end			
		for i=0, pos:size()-1 do
			local title=opt_dimension[i+1].title
			--coarseLog('chcp', title, pos(i))
			-- Temporarily change useCase.controlParam.
			if false then
				-- for debugging
				-- local test = {['map,0,swingHandMod,handstand,x']=0.0198,...}
				self.synthesis.graph:changeControlParameterInUseCase(title, test[title])
			else
				self.synthesis.graph:changeControlParameterInUseCase(title, pos(i))
				--dcjo debug
				--local fileName = 'postest'..rank..'.txt'
				--util.outputToFile(fileName, title..'	'..pos(i))
				--self.synthesis.graph:changeControlParameterInUseCase(title, 0.03)
			end
		end
		
		if debugMode then
			self.synthesis:saveStates("debugStates_arbc2"..g_i..".tbl")
		end
		useCases.unmapControlParam(useCase)

		--dcjo  initialize pendulum state and pose of synthesis
		self.synthesis:initializeControlParameters()
		self.synthesis:resetInitialState()
		self:updateConstraints()

		if debugMode then
			saveDebugInfo(self.synthesis.simulator, "debugInfo_restoreState"..g_i..".tbl")
			self.synthesis:saveStates("debugStates_ar"..g_i..".tbl")
		end

		-- proceed first two frames seperately. This is to avoid immediate trigger.

		local function OneStep()

			if  debugMode then
				local nf=self.synthesis.numFrames
				
				if min_f<=nf and nf<=max_f then
					g_debugOneStep=array:new()
				else
					g_debugOneStep=nil
				end
			end

			self.synthesis:oneStep()

			if debugMode then
				local nf=self.synthesis.numFrames
				if g_debugOneStep then
					util.saveTable(g_debugOneStep, "debugStates_oneStep"..g_i.."-"..nf..".tbl" )
				end
				self.synthesis:saveStates("debugStates"..g_i.."__"..nf..".tbl")
				if nf==0 then
					saveDebugInfo(self.synthesis.simulator, "debugInfo_oneStep"..g_i..".tbl")
				end
				if min_f<=nf and nf<=max_f then
					g_debugOneStep=array:new()
					g_debugOneStepFlag=true
				else
					g_debugOneStep=nil

				end
			end

			if self.synthesis.errorOccurred==true then
				fineLog( "error0")
				return false
			end

			self.synthesis:oneStepSimul()

			if debugMode then
				local nf=self.synthesis.numFrames
				if nf==0 then
					saveDebugInfo(self.synthesis.simulator, "debugInfo_oneStepSimul"..g_i..".tbl")
				end

				if g_debugOneStep then
					util.saveTable(g_debugOneStep, "debugStates_oneStepSimul"..g_i.."_"..nf..".tbl" )
				end
				self.synthesis:saveStates("debugStates_afterSimul_"..g_i.."__"..nf..".tbl")
			end

			kenergy=kenergy+self.synthesis.simulator:calcKineticEnergy()
			if self.synthesis.errorOccurred==true then
				print("errorOccurred")
				fineLog("error1")
				return false
			end

			-- fall down case(?)
			if error_feedback_method==EFM.NONE then 
				self.synthesis.errorOccurred = true
				print("EFM.NONE") 
				fineLog("error2")
				return false 
			end

			if error_feedback_method~=EFM.NONE then	 
				self.synthesis:prepareNextStep()      
			end

			return true
		end

		do
			self.synthesis.objectiveFunctionEnd=false

			--ys
			local total_a = {}
			if useCase.optimizerMethod=='Optimizer.methods.CMAes_Adapt' and file_exists('muscles.dat') then
				mOsim:readMuscleProperty('muscles.dat')
				--print('read muscles.dat')
				--printtblh(mOsim.f_m_o)
				
				for i=1,mOsim:getNumMuscles() do
					total_a[i] = 0.
				end
			end

			-- dcjo record initial ground_pelvis position
			local prev_gp = mOsim:getGlobalPos('ground_pelvis', vector3(0,0,0))

			local prevFrac=-1
			
			function compareCurSegment(last)
				--debug
				--print(self.synthesis.numFrames)
				--local scoreCoef = 10e8
				local scoreCoef = useCase.poseRefDiffWeight

				local segIndex
				if last then
					segIndex = seq.curSeg
				else
					segIndex = seq.curSeg - 1
				end

				-- save marker points of segIndex segment
				local smp = vectorn()
				smp:assign(self.compareChain.points1)

				for i=0, smp:size()/3 - 1 do
					smp:setVec3(i*3, smp:toVector3(i*3) - segGPPoints[segIndex])
				end
				segMarkerPoints[segIndex] = smp

				local compareScore,nframes=self.compareChain:compareQueue(self.objectList, seq[segIndex],self.objectCost,flush) -- per segment
			
				assert(compareScore~=nil)
				--MSE=MSE+compareScore
--				print("cur seg 	:	"..seq.curSeg)
--				print("compareScore	:	"..compareScore)
--				print("nMSE	:	"..nMSE)
--				print("nframes	:	"..nframes)
				nMSE=nMSE+nframes
				RE.output("compareScore", tostring(compareScore))
				--fineLog("compareScore", tostring(compareScore), nframes)
				
				-- dcjo calc direction difference of character per segment
				local gp = mOsim:getGlobalPos('ground_pelvis', vector3(0,0,0))
				local diff_gp = gp - prev_gp
				diffDir = diffDir + math.deg(math.acos(vector3.dotProduct(diff_gp,vector3(1,0,0))/diff_gp:length()))/180
				prev_gp = gp
				-- dcjo
				poseDiff = poseDiff + compareScore
				poseAvgDiff = poseAvgDiff + compareScore/nframes
				
				local scorePerFrame = compareScore/nframes * scoreCoef
				if scorePerFrame > 10000 then
					self.synthesis.objectiveFunctionEnd = true
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
					--local scoreCoef = 3 * 10e8
					local scoreCoef = useCase.posePrevDiffWeight
					local npoints = math.min(segMarkerPoints[segIndex]:size(),segMarkerPoints[segIndex-2]:size())
					local compareScore = self.compareChain.metric:calcDistance(segMarkerPoints[segIndex]:range(0,npoints-1), segMarkerPoints[segIndex-2]:range(0,npoints-1))
					local nframes = npoints/3

					poseDiff2 = poseDiff2 + compareScore
					poseAvgDiff2 = poseAvgDiff2 + compareScore/nframes

					local scorePerFrame = compareScore/nframes * scoreCoef
					if scorePerFrame > 10000 then
						self.synthesis.objectiveFunctionEnd = true
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
			
			while self.synthesis.objectiveFunctionEnd==false do
				local pcall_ok, errMsg=pcall(OneStep)
				if pcall_ok then 

					--ys
					if useCase.cmaEffortWeight~=nil or useCase.cmaEffortDistWeight~=nil then
						int_energy = int_energy + mOsim:getMetabolicEnergyRate()
					end
					if useCase.cmaLFootCFWeight~=nil or useCase.cmaLRDiffWeight~=nil then
						int_lfootcf = int_lfootcf + mOsim:getLFootCF():length()
						int_rfootcf = int_rfootcf + mOsim:getRFootCF():length()
					end
					if useCase.cmaLAnkPFFoWeight~=nil then
						int_lankforce = int_lankforce + mOsim:getLAnkPFForce()
					end
					if useCase.cmaRAnkPFFoWeight~=nil then
						int_rankforce = int_rankforce + mOsim:getRAnkPFForce()
					end
					if useCase.cmaLRKneeLimitWeight~=nil then
						local lexcess = 0
						local rexcess = 0
						if mOsim:getLKneeExtAng() > useCase.cmaLRKneeMaxExtAng then
							lexcess = mOsim:getLKneeExtAng() - useCase.cmaLRKneeMaxExtAng
						end
						if mOsim:getRKneeExtAng() > useCase.cmaLRKneeMaxExtAng then
							rexcess = mOsim:getRKneeExtAng() - useCase.cmaLRKneeMaxExtAng
						end
						int_lkneeExtExcess = int_lkneeExtExcess + lexcess 
						int_rkneeExtExcess = int_rkneeExtExcess + rexcess
					end
					if useCase.cmaLRHipLimitWeight~=nil then
						local lexcess = 0
						local rexcess = 0
						if mOsim:getLHipExtAng() > useCase.cmaLRHipMaxExtAng then
							lexcess = mOsim:getLHipExtAng() - useCase.cmaLRHipMaxExtAng
						end
						if mOsim:getRHipExtAng() > useCase.cmaLRHipMaxExtAng then
							rexcess = mOsim:getRHipExtAng() - useCase.cmaLRHipMaxExtAng
						end
						int_lhipExtExcess = int_lhipExtExcess + lexcess 
						int_rhipExtExcess = int_rhipExtExcess + rexcess 
					end
						
					-- when EFM.NONE occur or synthesis.errorOccurred == true, errMsg = false
					if errMsg~=true then
						print("curseg	:	"..seq.curSeg.."	error i :	"..self.synthesis.numFrames)
						util.outputToFile(Optimizer.outFile, 'curSeg '..self.sequence.curSeg..'	--error i '..i)
						errorOccurred=true
						break
					end

				else
					fineLog("objFn:pcall_error", errMsg)
					print("curseg	:	"..seq.curSeg.."i pcall	error i :	"..self.synthesis.numFrames)
					util.outputToFile(Optimizer.outFile, 'curSeg '..self.sequence.curSeg..'	-- pcall_error i '..i)
					errorOccurred=true
					break
				end	    

				--ys
				if useCase.optimizerMethod=='Optimizer.methods.CMAes_Adapt' then
					t_add_t_update(total_a, mOsim:getActivations())
					--printtblh(total_a)
				end


				local event=self:updateCurrSeg(i, self.synthesis, self.endSeg)

				if event then
					if event=="error" then errorOccurred=true end
					break
				end
			

				do
					if segGPPoints[seq.curSeg]==nil then
						segGPPoints[seq.curSeg] = vector3()
						segGPPoints[seq.curSeg]:assign(mOsim:getGlobalPos('ground_pelvis', vector3(0,0,0)))
					end


					if objectiveFunctionType==objectiveFunctionTypes.POSE then
						local info2
						do -- calc info without PDservoLatency 
							currFrame=self.synthesis.numFrames-1
							local seg, refTime, f, l, frac1, weight=self.synthesis:calcWeights(math.max(currFrame,0))
							info2={currSeg=seg, frac=frac1}

						end

						local MSEp=self:comparePose(skin,info2)
						--	    self.compareChain:drawMatching(self.objectList)
						RE.output("MSE", tostring(MSEp))
						--MSE=MSE+MSEp
						MAX_E=math.max(MAX_E, MSEp)
						nMSE=nMSE+1

						-- dcjo
						poseDiff = poseDiff + MSEp

					else

						local skel=self.synthesis.skel_withoutCOMjoint
						local nf=origDMot:numFrames()+1
						origDMot:resize(nf)
						local dpose=origDMot:row(nf-1)

						if useGraph=="Loco" then

							local deltaSeq=self.synthesis.deltaSeq
							if deltaSeq==nil then deltaSeq=0 end

							if self.refTrajectory then
								local refFrame=(self.synthesis.outputGlobal.refTime(self.synthesis.numFrames-1)+deltaSeq-1)*self.refTrajectory.nsample
								self.refTrajectory.mot:sampleRow(refFrame, dpose)
							end
						else
							self.synthesis.outputLocal.synLMot:sampleRow(self.synthesis.outputGlobal.refTime(self.synthesis.numFrames-1), dpose)
						end
						origMot:resize(nf)
						origMot:row(nf-1):assign(origDMot:row(nf-1))

						local startTransf=vector3(0,0,0)
						if nf>2 then
							startTransf=origMot:rootTransformation(nf-2):encode2D()
							origDMot:range(nf-2,nf):reconstructData(startTransf, origMot:range(nf-2, nf))
						else
							origDMot:range(nf-1,nf):reconstructData(startTransf, origMot:range(nf-1, nf))
						end		     


						local pose=origMot:row(nf-1)

						--local pose=info.currSeg.refSeg:pose(info.frac)

						skin:setPoseDOF(pose)
						skel:setPoseDOF(pose)


						local info=self.synthesis.trackingInfo -- targetPose foregoes currentPose by PDservoLatency

						-- frac : refTime - segIndex			-- increasing
						-- print("info.frac"..info.frac)
						if info.frac<prevFrac  then
							compareCurSegment()
						end
						self.compareChain:addQueue(mrd_info.outputMotion, self.synthesis)

						prevFrac=info.frac
					end
				end
				if math.mod(i,renderFreq)==0 then
					RE.output("objectiveFunction", util.tostring(i, renderFreq))
					self.synthesis.simulator:drawDebugInformation()
					renderOneFrame()
				end

			end

			--ys
			if useCase.optimizerMethod=='Optimizer.methods.CMAes_Adapt' then
				if g_jobId~=nil then
					--print('g_jobId', g_jobId, pos)
					--
					util.saveTable(total_a, 'total_a_jobId'..g_jobId..'.dat')
					--print('save', 'total_a_jobId'..g_jobId..'.dat')
					--printtblh(total_a)
				else
					--print('g_jobId==nil')
				end
			end

			if errorOccurred then
				print("errorOccurred")
			else
				print("not errorOccurred")
			end
			
			if objectiveFunctionType~=objectiveFunctionTypes.POSE and self.synthesis.trackingInfo.frac < prevFrac then
				if not errorOccurred then
					-- last segment
					compareCurSegment(true)
				else
					compareCurSegment()
				end
			end
		end
		collectgarbage("collect")
		collectgarbage("collect")
		collectgarbage("collect")

	end
	if debugMode then
		local tbl={}
		tbl.errorOccurred=errorOccurred
		tbl.MSE=MSE
		tbl.nMSE=nMSE
		util.saveTable(tbl, 'debugInfo_optResult'..g_i..'.tbl') 
		print('objFcn: '..g_i)
		g_i=g_i+1
	end


	--ys
	--add metabolic energy consumption
	--for successful 10 seg simulation of g2592_soldier
	--MSE	0.0024659395252409
	--int_energy	568986.54503955
	--horz pelvis pos	3.63131 0 1.84293, length 4.07
	--int_energy/dist	139724.55112438
	--int_lfootcf		about 300000
	--int_lankforce			about 360000
	--int_l,rkneelimit			about 20000
	--cmaEffortWeight that makes int_energy similar to MSE : 4.33e-09
	--cmaEffortDistWeight that makes int_energy similar to MSE : 1.76e-08
	--
	if useCase.cmaPoseClamp~=nil then
		if nMSE~=0 and MSE/nMSE < useCase.cmaPoseClamp then
			MSE = nMSE * useCase.cmaPoseClamp
		end
	end
	
	if useCase.cmaEffortWeight~=nil then
		--print('MSE', MSE)
		--print('int_energy', int_energy)
		--print('cmaEffortWeight*int_energy', useCase.cmaEffortWeight*int_energy)
		MSE = MSE + useCase.cmaEffortWeight * int_energy
		
		-- dcjo
		totalEffort = totalEffort + (useCase.cmaEffortWeight * int_energy)
	end
	if useCase.cmaEffortDistWeight~=nil then
		local j = mOsim.mLoader:getBoneByName('ground_pelvis')
		local gp = mOsim.bfk:globalFrame(j):toGlobalPos(vector3(0,0,0))
		gp:setY(0)
		----print('horz pelvis pos', gp)
		--print('MSE', MSE)
		--print('int_energy/dist', int_energy/gp:length())
		--print('cmaEffortDistWeight*int_energy/dist', useCase.cmaEffortDistWeight*int_energy/gp:length())
		MSE = MSE + useCase.cmaEffortDistWeight * int_energy/gp:length()

		-- dcjo
		totalEffortPerDist = totalEffortPerDist + (useCase.cmaEffortDistWeight * int_energy/gp:length())

	end
	if useCase.cmaLFootCFWeight~=nil then
		MSE = MSE + useCase.cmaLFootCFWeight * int_lfootcf
	end
	if useCase.cmaLRDiffWeight~=nil then
		MSE = MSE + useCase.cmaLRDiffWeight * (int_lfootcf-int_rfootcf)
	end
	if useCase.cmaDistWeight~=nil then
		local j = mOsim.mLoader:getBoneByName('ground_pelvis')
		local gp = mOsim.bfk:globalFrame(j):toGlobalPos(vector3(0,0,0))
		gp:setY(0)
		MSE = MSE + useCase.cmaDistWeight * gp:length()
	end
	if useCase.cmaLAnkPFFoWeight~=nil then
		MSE = MSE + useCase.cmaLAnkPFFoWeight * int_lankforce
	end
	if useCase.cmaRAnkPFFoWeight~=nil then
		MSE = MSE + useCase.cmaRAnkPFFoWeight * int_rankforce
	end
	if useCase.cmaLRKneeLimitWeight~=nil then
		MSE = MSE + useCase.cmaLRKneeLimitWeight * (int_lkneeExtExcess + int_rkneeExtExcess)
	end
	if useCase.cmaLRHipLimitWeight~=nil then
		MSE = MSE + useCase.cmaLRHipLimitWeight * (int_lhipExtExcess + int_rhipExtExcess)
	end

	-- dcjo for debugging
	util.outputToFile(Optimizer.outFile, 'curSeg '..self.sequence.curSeg..'	--endSeg '..self.endSeg)

	print("MSE	:	"..MSE)
--	MSE = 0

	-- dcjo report terms for debug
	function reportTerms()
		local energyTerms = util.loadTable("MSE/energyTerms_"..rank..".tbl")
		local i = #energyTerms + 1
		energyTerms[i] = {}
		if rank == 0 then
			energyTerms[i][1] = i-1
		else
			energyTerms[i][1] = i
		end

		energyTerms[i][2] = MSE
		energyTerms[i][3] = poseDiff
		energyTerms[i][4] = totalEffort
		energyTerms[i][5] = totalEffortPerDist
		energyTerms[i][6] = diffDir -- sumOffset
		energyTerms[i][7] = nMSE
		energyTerms[i][8] = self.sequence.curSeg
		util.saveTable(energyTerms, "MSE/energyTerms_"..rank..".tbl")
	end
	--reportTerms()

	-- poseDiff & falldown
	objFn = objFn + poseDiffPenalty
	if errorOccurred then
		fineLog("errorOccurred?")
		if bRestoreStateSkipped then
			self:restoreStates(self.stage, self.subStage) -- so that skipping restorestate at next step should not have problem
		end

		-- for balance
		--local objFn=100000+(self.endSeg-(self.sequence.curSeg+self.synthesis.trackingInfo.frac))*20000+2e5*MSE/nMSE + diffDir*10000
		-- + sumOffset * self.endSeg * 10000
--		local objFn=100000+(self.endSeg-(self.sequence.curSeg+self.synthesis.trackingInfo.frac))*20000+MSE/nMSE + diffDir*10000 + sumOffset * self.endSeg * 10000
		if nMSE==0 then
			objFn = objFn + 100000+self.endSeg*20000+100000
			util.outputToFile(Optimizer.outFile, '')
			util.outputToFile(Optimizer.outFile, 'nMSE == 0')
		else
			--objFn = objFn + 100000+(self.endSeg-(self.sequence.curSeg+self.synthesis.trackingInfo.frac))*20000
			objFn = objFn + 100000+(self.endSeg-self.sequence.curSeg)*20000
		end
	end

	-- metabolic energy
	objFn = objFn + MSE/nMSE

	self.compareChain=nil

	--local bound=math.max(math.abs(pos:minimum()), math.abs(pos:maximum()))
	--local boundEnergy=0
	--[[
	if bound>0.4 then
		boundEnergy=(bound-0.4) *(bound-0.4)*1e-5*6
	end
	]]--
	--local objFn = 2e10*MSE/nMSE+boundEnergy + diffDir*10000
	-- + sumOffset * self.endSeg * 10000
	
	util.outputToFile(Optimizer.outFile, '')
	util.outputToFile(Optimizer.outFile, 'curSeg	'..self.sequence.curSeg..'	numFrames	'..self.synthesis.numFrames)
	util.outputToFile(Optimizer.outFile, 'objfn '..objFn)
	util.outputToFile(Optimizer.outFile, 'MSE	'..MSE..'	nMSE	'..nMSE..'	MSE/nMSE	'..(MSE/nMSE))
	util.outputToFile(Optimizer.outFile, 'int_energy: '..int_energy..'	int_energy/nMSE: '..(int_energy/nMSE))
	if useCase.cmaEffortWeight~=nil then
		util.outputToFile(Optimizer.outFile, 'int_energy*cmaEffortWeight/nMSE: '..(int_energy/nMSE*useCase.cmaEffortWeight))
	end
	util.outputToFile(Optimizer.outFile, 'diffDir	'..diffDir)
--	util.outputToFile(Optimizer.outFile, 'sumOffset	'..sumOffset)
	
	fineLog("objFn"..tostring(objFn))
	--fineLog( MSE/nMSE)--+kenergy*1e-11)

	return objFn

end

function Tracker:comparePose(skin, info)

   local skel=self.synthesis.skel_withoutCOMjoint

   local pose=info.currSeg.refSeg:pose(info.frac)
   skin:setPoseDOF(pose)
   skel:setPoseDOF(pose)

   return self.compareChain:compare()
end

function Tracker:setDomain()
	local param=self._setStageParameters
	assert(self.stage==param[1])
	local stageParam=param[2]
   self.startSeg=stageParam.startSeg
   self.endSeg =stageParam.endSeg
   self.endFrac=stageParam.endFrac
   self.nextStartSeg=param[5]
end

function Tracker:_setStage(i, stageParam)-- , baseStage, baseStartSeg, nextStartSeg)

	self._setStageParameters={i, stageParam}--, baseStage, baseStartSeg, nextStartSeg}

	if i==1 and stageParam.startSeg~=1 then

		-- use prepareNextStageFunction 
		self.stage=0
		self.subStage=0

		fineLog( "prepareFirstStage",self.stage, self.subStage)
		renderFreq=4
		self:restoreStates(0, 0)
		local graph=self.synthesis.graph
		local seq=self.sequence
		seq.curSeg=1
		self.startSeg=1
		self.endSeg=100000000
		self.nextStartSeg=stageParam.startSeg

		RE.output("prepareFirstStage", seq.curSeg.." "..(self.nextStartSeg))

		self:__prepareNextStage()
	end
	local func=Tracker[stageParam.setStageFunc]
	local param=stageParam.param
	func(self, i, 0, param)
	self.objectCost=Tracker[stageParam.objCost]

   if useCase.noStateDependentRefTraj then
	   self:generateRefTrajectoryWithoutPoseGeneration()
   else
	   self:generateRefTrajectory()
   end
   
end

function Tracker:setStage(i, subi)
	self:_setStage(i, self.stageParam[i])--, baseStage, baseStartSeg, nextStartSeg)
end

function Tracker:generateRefTrajectory()
	--if true then return nil end
	print("genRef")

	local saved_state=deepCopyTable(self.synthesis:getStates())
	-- TODO

	genRefTrajectory=true
	local syn=GenRefTrajectory:new(self.synthesis)


	local seq=self.sequence
	local nn=math.min(self.endSeg+2, table.getn(self.sequence))

	seq.curSeg=1
	syn:initStates(self.sequence[0], self.sequence[1])

	local errorOccurred=false
	local function OneStep()

		syn:oneStep()

		if syn.errorOccurred==true then
			--dbg.console()
			fineLog( "genRef error")
			return false
		end
		return true
	end

	do
		local i=0
		local prevFrac=-1
		while true do

			if false then
				-- genRefTrajectory is safe. No pcall is needed here.
				local pcall_ok, errMsg=pcall(OneStep)
				if pcall_ok then 
					if errMsg~=true then
						--dbg.console()
						errorOccurred=true
						break
					end
				else
					--dbg.console()
					fineLog("pcall_error", errMsg)
					errorOccurred=true
					break
				end	    
			else
				OneStep()
			end

			i=i+1

			local event=self:updateCurrSeg(i, syn, nn, true)
			if event then
				if event=="error" then errorOccurred=true end
				break
			end

			local renderFreq=4
			if math.mod(i,renderFreq)==0 then
				renderOneFrame()
			end

		end
	end
	--assert(not errorOccurred)


	do -- get sparse sample of synthesized motion

		self.refTrajectory={}

		self.refTrajectory.motCon=MotionDOFcontainer(self.synthesis.skel_withoutCOMjoint.dofInfo)
		self.refTrajectory.mot=self.refTrajectory.motCon.mot
		local outputMotion=self.refTrajectory.mot
		local nsample=10
		self.refTrajectory.nsample=nsample
		outputMotion:resize(nn*nsample)

		syn.outputMotion:exportMot("debug_plot_mwo.dof")
		local lmot=MotionDOF(syn.outputMotion.mot)
		lmot:convertToDeltaRep()
		for i=1, nn do
			assert(self.sequence[i]==syn.outputLocal.segInfo(i).seg)
			for j=0, nsample-1 do
				local invRefTime=syn.outputGlobal:invRefTime(sop.map(j, 0, nsample, i, i+1))
				lmot:matView():sampleRow(invRefTime, outputMotion:row((i-1)*nsample+j))
			end
		end
	end

	syn:restore_globals()
	syn:__finalize()
	collectgarbage("collect")
	collectgarbage("collect")
	collectgarbage("collect")

	assert(genRefTrajectory==true)
	genRefTrajectory=false

	self.synthesis:_restoreStates(saved_state)
end

function Tracker:generateRefTrajectoryWithoutPoseGeneration()

   self.refTrajectory={}

   local nn=table.getn(self.sequence)
   self.refTrajectory.motCon=MotionDOFcontainer(self.synthesis.skel_withoutCOMjoint.dofInfo)
   self.refTrajectory.mot=self.refTrajectory.motCon.mot
   
   local outputMotion=self.refTrajectory.mot
   local outputL=LocoGraphOutputLocal:new(self.synthesis.graph)
   local nsample=10
   self.refTrajectory.nsample=nsample
   outputMotion:resize(nn*nsample)
   for i=1, nn do
      local seg=self.sequence[i]
	  for j=0, nsample-1 do
		  seg.locomot.lmot:matView():sampleRow(seg.first+seg.len*j*0.1, outputMotion:row((i-1)*nsample+j))
	  end
   end

   if false then -- only for debugging
	   local id=self.synthesis.graph.id
	   id:initFromDeltaRep(vector3(0,0,0),outputMotion:matView())
	   id:reconstruct(outputMotion:matView(), self.synthesis.outputGlobal:frameRate())
	   self.refTrajectory.motCon:exportMot("refMotion.dof")
   end


end

function Tracker:samplePose(ltime, outputPose)
end

function Tracker:setStage_constant1(i, subi) -- base controller.

   local starget={}
   local etarget={}
   local domains={}

   local sconsttarget={"ftLy", "ftRy", "rfLt", "rfRt"}
   local econsttarget={"f1Ly", "f1Ry", "f2Ly", "f2Ry", "r1Lt", "r1Rt","r2Lt", "r2Rt",}
   local gtarget={}
   --    {"torsoPgain", 10},
   --    {"torsoDgain",1}
   -- }

   self:setStage_general(i, subi, starget, etarget, domains, sconsttarget, econsttarget, gtarget)
end

function Tracker:setStage_constant2(i, subi) -- debug oscillation

   local starget={}
   local etarget={}
   local domains={}

   local sconsttarget={"velx", "velz"}
   local econsttarget={}
   local gtarget={}
   --    {"torsoPgain", 10},
   --    {"torsoDgain",1}
   -- }

   self:setStage_general(i, subi, starget, etarget, domains, sconsttarget, econsttarget, gtarget)
end

function Tracker:setStage_linear_base0(i, subi)
   
   local gtarget={
      {"spprty1StR", 0.001},
      {"spprty2StR", 0.001},
      {"spprty3StR", 0.001},
      {"swingy1StR", 0.001},
      {"swingy2StR", 0.001},
      {"swingy3StR", 0.001},
      {"spprtz1StR", 0.001},
      {"spprtz2StR", 0.001},
      {"spprtz3StR", 0.001},
      {"swingz1StR", 0.001},
      {"swingz2StR", 0.001},
      {"swingz3StR", 0.001},


      -- {"spprtz_StR_d",0.001}, 
      -- {"swingz_StR_d",0.001}

}
   self:setStage_general(i, subi, nil, nil, nil, nil, nil, gtarget)
end

function Tracker:makeTable(axis, grpName, isSymmetric)
   if isSymmetric==nil then isSymmetric=true end

   if isSymmetric then
      local gtarget={
	 {"spprt"..axis.."1"..grpName, 0.001},
	 {"spprt"..axis.."2"..grpName, 0.001},
	 {"spprt"..axis.."3"..grpName, 0.001},
	 {"swing"..axis.."1"..grpName, 0.001},
	 {"swing"..axis.."2"..grpName, 0.001},
	 {"swing"..axis.."3"..grpName, 0.001},
      }
      return gtarget
   end

   return table.ijoin(self:makeTableSeg(grpName, "LR", grpName, "RL", "LR", axis),
		      self:makeTableSeg(grpName, "RL", grpName, "LR", "RL", axis))
end


function Tracker:makeTableSeg(grpName1, seg1, grpName2, seg2, seg3 , axis)
   local gtarget={
      {"f2L"..axis.."_"..grpName1.."_"..seg1.."_"..seg2, 0.001},
      {"ftL"..axis.."_"..grpName2.."_"..seg2, 0.001},
      {"f1L"..axis.."_"..grpName2.."_"..seg2.."_"..seg3, 0.001},
      {"f2R"..axis.."_"..grpName1.."_"..seg1.."_"..seg2, 0.001},
      {"ftR"..axis.."_"..grpName2.."_"..seg2, 0.001},
      {"f1R"..axis.."_"..grpName2.."_"..seg2.."_"..seg3, 0.001},
   }
   return gtarget
end

function Tracker:setStage_y_all(i, subi)
   
   local gtarget=table.ijoin(self:makeTable("y","StR"), self:makeTable("y","run"))
   self:setStage_general(i, subi, nil, nil, nil, nil, nil, gtarget)
end
function Tracker:setStage_z_all(i, subi)
   
   local gtarget=table.ijoin(self:makeTable("z","StR"), self:makeTable("z","run"))
   self:setStage_general(i, subi, nil, nil, nil, nil, nil, gtarget)
end

function Tracker:setStage_xz_all(i, subi)
   
   local gtarget1=table.ijoin(self:makeTable("x","StR"), self:makeTable("x","run"))
   local gtarget2=table.ijoin(self:makeTable("z","StR"), self:makeTable("z","run"))
   local gtarget=table.ijoin(gtarget1, gtarget2)
   self:setStage_general(i, subi, nil, nil, nil, nil, nil, gtarget)
end

function Tracker:setStage_xyz_all(i, subi)
   
   local gtarget1=table.ijoin(self:makeTable("x","StR"), self:makeTable("x","run"))
   local gtarget2=table.ijoin(self:makeTable("y","StR"), self:makeTable("y","run"))
   local gtarget3=table.ijoin(self:makeTable("z","StR"), self:makeTable("z","run"))
   local gtarget=table.ijoin(gtarget1, gtarget2, gtarget3)
   self:setStage_general(i, subi, nil, nil, nil, nil, nil, gtarget)
end

function Tracker:setStage_xyz_stand_short(i, subi)

   local gtarget0=table.ijoin(
      self:makeTableSeg("STP", "st", "StR", "SR", "RL", "x"),
      self:makeTableSeg("STP", "st", "StR", "SR", "RL", "y"),
      self:makeTableSeg("STP", "st", "StR", "SR", "RL", "z"))
   
   
   local gtarget1=self:makeTable("x","StR")
   local gtarget2=self:makeTable("y","StR")
   local gtarget3=self:makeTable("z","StR")
   local gtarget=table.ijoin(gtarget0, gtarget1, gtarget2, gtarget3
			     -- ,{
			     -- 	{"spprt_StR_1_kneeControlTorque", 0.5},
			     -- 	{"spprt_StR_2_kneeControlTorque", 0.5},
			     -- 	{"spprt_StR_3_kneeControlTorque", 0.5},
			     -- 	{"swing_StR_1_kneeControlTorque", 0.5},
			     -- 	{"swing_StR_2_kneeControlTorque", 0.5},
			     -- 	{"swing_StR_3_kneeControlTorque", 0.5},
			     -- }
			  )
   self:setStage_general(i, subi, nil, nil, nil, nil, nil, gtarget)
end

function Tracker:setStage_xyz_run_short(i, subi)
   
   local gtarget1=self:makeTable("x","run")
   local gtarget2=self:makeTable("y","run")
   local gtarget3=self:makeTable("z","run")
   local gtarget4={}

   if useCase.numFootKeyFrames==10 then
      gtarget4={
	 {"keyframe_spprt_y_run_3", 0.005},
	 {"keyframe_spprt_y_run_4", 0.005},
	 {"keyframe_spprt_y_run_5", 0.005}
      }
   end
   local gtarget=table.ijoin(gtarget1, gtarget2, gtarget3, gtarget4,
			     {	
				-- {"velModx_run",0.005}, 
				-- {"spprt_run_1_scaleControlForce", 0.005},
				-- {"spprt_run_2_scaleControlForce", 0.005},
				-- {"spprt_run_3_scaleControlForce", 0.005},
--				{"spprt_run_all_scaleControlForce", 0.005},

-- 				{"spprt_run_1_kneeControlTorque", 0.5},
-- 				{"spprt_run_2_kneeControlTorque", 0.5},
-- 				{"spprt_run_3_kneeControlTorque", 0.5},
-- 				{"swing_run_1_kneeControlTorque", 0.5},
-- 				{"swing_run_2_kneeControlTorque", 0.5},
-- 				{"swing_run_3_kneeControlTorque", 0.5},
-- 
--				{"COMx_run", 0.001},
--				{"spprt_run_all_scaleControlForce", 0.005},

			     }
			     )
   self:setStage_general(i, subi, nil, nil, nil, nil, nil, gtarget)
end

function Tracker:setStage_param(i, subi, param)

	--dbg.console()
   local gtarget={}
   for i, p in ipairs(param) do
      if p[1]=="makeTable" then
	 gtarget=table.ijoin(gtarget, self:makeTable(p[2],p[3]))
      else
	 gtarget=table.ijoin(gtarget, {p})
      end
   end
   self:setStage_general(i, subi, nil, nil, nil, nil, nil, gtarget)

end

function Tracker:setStage_x_run_short(i, subi)
    
   local gtarget1=self:makeTable("x","run")
   -- local gtarget2=self:makeTable("y","run")
   -- local gtarget3=self:makeTable("z","run")
   local gtarget=table.ijoin(gtarget1,--, gtarget2, gtarget3, 
			     {	
				-- {"velModx_run",0.005}, 
				-- {"spprt_run_1_scaleControlForce", 0.005},
				-- {"spprt_run_2_scaleControlForce", 0.005},
				-- {"spprt_run_3_scaleControlForce", 0.005},
--				{"spprt_run_all_scaleControlForce", 0.005},

				{"spprt_run_1_kneeControlTorque", 0.5},
				{"spprt_run_2_kneeControlTorque", 0.5},
				{"spprt_run_3_kneeControlTorque", 0.5},
				{"swing_run_1_kneeControlTorque", 0.5},
				{"swing_run_2_kneeControlTorque", 0.5},
				{"swing_run_3_kneeControlTorque", 0.5},
-- 
--				{"COMx_run", 0.001},
--				{"spprt_run_all_scaleControlForce", 0.005},

			     }
			     )
   self:setStage_general(i, subi, nil, nil, nil, nil, nil, gtarget)
end


function Tracker:setStage_xyz_stand_new(i, subi)

	local gtarget={
		{'keyframe,0,swingFootForce,StR,RL,x', 11},
		{'keyframe,1,swingFootForce,StR,RL,x', 11},
		{'keyframe,2,swingFootForce,StR,RL,x', 11},
		{'keyframe,0,swingFootForce,StR,LR,x', 11},
		{'keyframe,1,swingFootForce,StR,LR,x', 11},
		{'keyframe,2,swingFootForce,StR,LR,x', 11},
		{'keyframe,0,swingFootForce,StR,RL,y', 11},
		{'keyframe,1,swingFootForce,StR,RL,y', 11},
		{'keyframe,2,swingFootForce,StR,RL,y', 11},
		{'keyframe,0,swingFootForce,StR,LR,y', 11},
		{'keyframe,1,swingFootForce,StR,LR,y', 11},
		{'keyframe,2,swingFootForce,StR,LR,y', 11},
		{'keyframe,0,swingFootForce,StR,RL,z', 11},
		{'keyframe,1,swingFootForce,StR,RL,z', 11},
		{'keyframe,2,swingFootForce,StR,RL,z', 11},
		{'keyframe,0,swingFootForce,StR,LR,z', 11},
		{'keyframe,1,swingFootForce,StR,LR,z', 11},
		{'keyframe,2,swingFootForce,StR,LR,z', 11},
	}
	self:setStage_general(i, subi, nil, nil, nil, nil, nil, gtarget)
end
function Tracker:setStage_xyz_run_new(i, subi)
    
	local gtarget={
		{'keyframe,0,swingFootForce,run,RL,x', 11},
		{'keyframe,1,swingFootForce,run,RL,x', 11},
		{'keyframe,2,swingFootForce,run,RL,x', 11},
		{'keyframe,0,swingFootForce,run,LR,x', 11},
		{'keyframe,1,swingFootForce,run,LR,x', 11},
		{'keyframe,2,swingFootForce,run,LR,x', 11},
		{'keyframe,0,swingFootForce,run,RL,y', 11},
		{'keyframe,1,swingFootForce,run,RL,y', 11},
		{'keyframe,2,swingFootForce,run,RL,y', 11},
		{'keyframe,0,swingFootForce,run,LR,y', 11},
		{'keyframe,1,swingFootForce,run,LR,y', 11},
		{'keyframe,2,swingFootForce,run,LR,y', 11},
		{'keyframe,0,swingFootForce,run,RL,z', 11},
		{'keyframe,1,swingFootForce,run,RL,z', 11},
		{'keyframe,2,swingFootForce,run,RL,z', 11},
		{'keyframe,0,swingFootForce,run,LR,z', 11},
		{'keyframe,1,swingFootForce,run,LR,z', 11},
		{'keyframe,2,swingFootForce,run,LR,z', 11},
	}
   self:setStage_general(i, subi, nil, nil, nil, nil, nil, gtarget)
end

function Tracker:setStage_yz_run_short(i, subi)
   
   local gtarget2=self:makeTable("y","run")
   local gtarget3=self:makeTable("z","run")
   local gtarget=table.ijoin(gtarget2, gtarget3, {{"velModx_run",0.005}, {"velModz_run",0.005}})
   self:setStage_general(i, subi, nil, nil, nil, nil, nil, gtarget)
end

function Tracker:setStage_xyz_run(i, subi)
   
   local gtarget1=self:makeTable("x","run")
   local gtarget2=self:makeTable("y","run")
   local gtarget3=self:makeTable("z","run")
   local gtarget=table.ijoin(gtarget1, gtarget2, gtarget3, {{"velModx_run",0.005}}, {{"COMx_run", 0.0005}, {"COMy_run", 0.0005}, {"COMz_run", 0.0005}})
   self:setStage_general(i, subi, nil, nil, nil, nil, nil, gtarget)
end


function Tracker:setStage_xyz_stand(i, subi)

   local gtarget0=table.ijoin(
      self:makeTableSeg("STP", "st", "StR", "SR", "RL", "x"),
      self:makeTableSeg("STP", "st", "StR", "SR", "RL", "y"),
      self:makeTableSeg("STP", "st", "StR", "SR", "RL", "z"))
   
   
   local gtarget1=self:makeTable("x","StR")
   local gtarget2=self:makeTable("y","StR")
   local gtarget3=self:makeTable("z","StR")
   local gtarget=table.ijoin(gtarget0, gtarget1, gtarget2, gtarget3, {{"COMx_StR", 0.0005}, {"COMy_StR", 0.0005}, {"COMz_StR", 0.0005}})
   self:setStage_general(i, subi, nil, nil, nil, nil, nil, gtarget)
end

function Tracker:setStage_linear_baseY(i, subi)
   
   local gtarget={
      {"spprty1run", 0.001},
      {"spprty2run", 0.001},
      {"spprty3run", 0.001},
      {"swingy1run", 0.001},
      {"swingy2run", 0.001},
      {"swingy3run", 0.001},
   }

      -- {"spprtz_run_comvel.z",0.001}, {"spprtz_run_angle.x",0.001}, {"spprtz_run_d",0.001},
      -- {"swingz_run_comvel.z",0.001}, {"swingz_run_angle.x",0.001}, {"swingz_run_d",0.001}}
   self:setStage_general(i, subi, nil, nil, nil, nil, nil, gtarget)
end

function Tracker:setStage_linear_baseZ(i, subi)
   
   local gtarget={
      {"spprtz1run", 0.001},
      {"spprtz2run", 0.001},
      {"spprtz3run", 0.001},
      {"swingz1run", 0.001},
      {"swingz2run", 0.001},
      {"swingz3run", 0.001},
   }

      -- {"spprtz_run_comvel.z",0.001}, {"spprtz_run_angle.x",0.001}, {"spprtz_run_d",0.001},
      -- {"swingz_run_comvel.z",0.001}, {"swingz_run_angle.x",0.001}, {"swingz_run_d",0.001}}
   self:setStage_general(i, subi, nil, nil, nil, nil, nil, gtarget)
end

function Tracker:setStage_linear_baseX(i, subi)
   
   local gtarget={
      {"spprtx1run", 0.001},
      {"spprtx2run", 0.001},
      {"spprtx3run", 0.001},
      {"swingx1run", 0.001},
      {"swingx2run", 0.001},
      {"swingx3run", 0.001},
   }

      -- {"spprtz_run_comvel.z",0.001}, {"spprtz_run_angle.x",0.001}, {"spprtz_run_d",0.001},
      -- {"swingz_run_comvel.z",0.001}, {"swingz_run_angle.x",0.001}, {"swingz_run_d",0.001}}
   self:setStage_general(i, subi, nil, nil, nil, nil, nil, gtarget)
end


function Tracker:setStage_linear_base1(i, subi)
   
   local gtarget={
      {"spprty1run", 0.001},
      {"spprty2run", 0.001},
      {"spprty3run", 0.001},
      {"swingy1run", 0.001},
      {"swingy2run", 0.001},
      {"swingy3run", 0.001},
      {"spprtz1run", 0.001},
      {"spprtz2run", 0.001},
      {"spprtz3run", 0.001},
      {"swingz1run", 0.001},
      {"swingz2run", 0.001},
      {"swingz3run", 0.001},
   }

      -- {"spprtz_run_comvel.z",0.001}, {"spprtz_run_angle.x",0.001}, {"spprtz_run_d",0.001},
      -- {"swingz_run_comvel.z",0.001}, {"swingz_run_angle.x",0.001}, {"swingz_run_d",0.001}}
   self:setStage_general(i, subi, nil, nil, nil, nil, nil, gtarget)
end

function Tracker:setStage_linear_base_internal(i, subi)
   local gtarget={
      {"torqueSwingSrun_d", 0.5}, {"torqueSwingErun_d", 0.5},
      {"torqueSpprtSrun_d", 0.5}, {"torqueSpprtErun_d", 0.5},
      {"torquePelvsSrun_d", 0.5}, {"torquePelvsErun_d", 0.5},
   }
   -- local gtarget={
   --    {"spprtx_run_comvel.x",0.001}, {"spprtx_run_angle.z",0.001}, {"spprtx_run_d",0.001},
   --    {"swingx_run_comvel.x",0.001}, {"swingx_run_angle.z",0.001}, {"swingx_run_d",0.001}}

   self:setStage_general(i, subi, nil, nil, nil, nil, nil, gtarget)
end

function Tracker:setStage_linear_base2(i, subi)
   local gtarget={
      {"spprty1run", 0.001},
      {"spprty2run", 0.001},
      {"spprty3run", 0.001},
      {"swingy1run", 0.001},
      {"swingy2run", 0.001},
      {"swingy3run", 0.001},
      {"spprtx1run", 0.001},
      {"spprtx2run", 0.001},
      {"spprtx3run", 0.001},
      {"swingx1run", 0.001},
      {"swingx2run", 0.001},
      {"swingx3run", 0.001},
   }
   
   self:setStage_general(i, subi, nil, nil, nil, nil, nil, gtarget)
end

function Tracker:setStage_linear_base3(i, subi)
   
   local gtarget={
      {"spprty1run_controlforceMag",0.001}, {"spprty1run_d",0.001},
      {"swingy1run_controlforceMag",0.001}, {"swingy1run_d",0.001},
      {"spprty2run_controlforceMag",0.001}, {"spprty2run_d",0.001},
      {"swingy2run_controlforceMag",0.001}, {"swingy2run_d",0.001},
      {"spprty3run_controlforceMag",0.001}, {"spprty3run_d",0.001},
      {"swingy3run_controlforceMag",0.001}, {"swingy3run_d",0.001},

      {"spprtx1run_controlforce.x",0.001}, {"spprtx1run_d",0.001},
      {"swingx1run_controlforce.x",0.001}, {"swingx1run_d",0.001},
      {"spprtx2run_controlforce.x",0.001}, {"spprtx2run_d",0.001},
      {"swingx2run_controlforce.x",0.001}, {"swingx2run_d",0.001},
      {"spprtx3run_controlforce.x",0.001}, {"spprtx3run_d",0.001},
      {"swingx3run_controlforce.x",0.001}, {"swingx3run_d",0.001},

      {"spprtz1run_controlforce.z",0.001}, {"spprtz1run_d",0.001},
      {"swingz1run_controlforce.z",0.001}, {"swingz1run_d",0.001},
      {"spprtz2run_controlforce.z",0.001}, {"spprtz2run_d",0.001},
      {"swingz2run_controlforce.z",0.001}, {"swingz2run_d",0.001},
      {"spprtz3run_controlforce.z",0.001}, {"spprtz3run_d",0.001},
      {"swingz3run_controlforce.z",0.001}, {"swingz3run_d",0.001}}

   self:setStage_general(i, subi, nil, nil, nil, nil, nil, gtarget)
end
   
function Tracker:setStage_linear1(i, subi)

   local starget={"ftLy", "ftRy"}
   local etarget={"f1Ly", 
		  "f1Ry",
		  "f2Ly", 
		  "f2Ry", 
		  }
   local domains={ 
      default={"comvel.x", "comvel.z", "angle.x", "angle.z"},
      ftLy={"comvel.x", "comvel.z", "angle.x", "angle.z", "angleLen"},
      ftRy={"comvel.x", "comvel.z", "angle.x", "angle.z", "angleLen"},
      f1Ly={"comvel.x", "comvel.z", "angle.x", "angle.z", "angleLen"},
      f1Ry={"comvel.x", "comvel.z", "angle.x", "angle.z", "angleLen"},
      f2Ly={"comvel.x", "comvel.z", "angle.x", "angle.z", "angleLen"},
      f2Ry={"comvel.x", "comvel.z", "angle.x", "angle.z", "angleLen"},
   }

   local sconsttarget={"ftLy", "ftRy", "ftLx", "ftRx", "ftLz", "ftRz"}
   local econsttarget={"f1Ly", "f1Ry", "f2Ly", "f2Ry", "ftLx", "ftRx", "ftLz", "ftRz"}

   local gtarget={}
   --    {"torsoPgain", 10},
   --    {"torsoDgain",1}
   -- }

   self:setStage_general(i, subi, starget, etarget, domains, sconsttarget, econsttarget, gtarget)
end

function Tracker:setStage_linear3(i, subi)

   local starget={}
   local etarget={"f1Ly", "f1Ry", "f2Ly", "f2Ry", "r1Lt", "r1Rt","r2Lt", "r2Rt","ftLx", "ftLz", "ftRx", "ftRz"}
		  
   local domains={ 
      default={"comvel.x", "comvel.z", "angle.x", "angle.z"},
      -- ftLy={"comvel.x", "comvel.z", "angle.x", "angle.z", "angleLen"},
      -- ftRy={"comvel.x", "comvel.z", "angle.x", "angle.z", "angleLen"},
      -- f1Ly={"comvel.x", "comvel.z", "angle.x", "angle.z", "angleLen"},
      -- f1Ry={"comvel.x", "comvel.z", "angle.x", "angle.z", "angleLen"},
      -- f2Ly={"comvel.x", "comvel.z", "angle.x", "angle.z", "angleLen"},
      -- f2Ry={"comvel.x", "comvel.z", "angle.x", "angle.z", "angleLen"},
   }

   local sconsttarget={}
   local econsttarget={}

   local gtarget={}
   --    {"torsoPgain", 10},
   --    {"torsoDgain",1}
   -- }

   self:setStage_general(i, subi, starget, etarget, domains, sconsttarget, econsttarget, gtarget)
end

function Tracker:setStage_linear2(i, subi)

   local starget={}
   local etarget={}
   local domains={}

   local sconsttarget={"ftLx", "ftRx", "ftLz", "ftRz"}
   local econsttarget={"ftLx", "ftRx", "ftLz", "ftRz"}
   local gtarget={}
   --    {"torsoPgain", 10},
   --    {"torsoDgain",1}
   -- }

   self:setStage_general(i, subi, starget, etarget, domains, sconsttarget, econsttarget, gtarget)
end

function Tracker:setStage_general(i, subi, starget, etarget, domains, sconsttarget, econsttarget, gtarget)

	fineLog( "setStage",i,subi, self._setStageParameters)


	local graph=self.synthesis.graph

	self.synthesis:changeControlParameters(useCase.controlParm)
	self:accumulateResult(self:decr({i,subi}))
	self:updateConstraints()

	expandUseCase(useCase.controlParam)

	self.stage=i
	self.subStage=subi

	self:setDomain()
	print("setStage", i, subi)

	-- for those variables without linear feedback
	local function add2(name, max_step)
		local od={}
		od.title=name
		od.curval=graph:getControlParameterFromUseCase(name)
		--local test=graph:getControlParameterFromGraph(name)
		--assert(od.curval==test)
		assert(od.curval)
		print('add2',name, od.curval)
		od.max_step=max_step
		od.grad_step=max_step/2
		opt_dimension:pushBack(od)
	end

	if true then

		opt_dimension=array:new()
		-- opt_dimension varies depending on the subStage
		local function addCoef(name, domainName, max_step)
			local od={}
			od.title="regResult_"..domainName.."_"..name
			od.curval=0

			if useCase.controlParam[od.title]~=nil then
				od.curval=useCase.controlParam[od.title]
			end

			od.max_step=max_step
			od.grad_step=max_step/2
			opt_dimension:pushBack(od)
		end
		local function addConst(name, max_step)
			local od={}
			od.title="regResult_d_"..name
			od.curval=useCase.controlParam[name]
			if od.curval==nil then
				od.curval=0
			end

			if useCase.controlParam[od.title]~=nil then
				od.curval=useCase.controlParam[od.title]
			end

			od.max_step=max_step
			od.grad_step=max_step/2
			opt_dimension:pushBack(od)
		end

		local function add(target, postfix)
			local name=target.."_"..postfix
			local domain=domains[target]
			if domain==nil then
				domain=domains.default
				assert(domain)
			end

			local maxStep={0.01, 0.01, 0.01, 0.01, 0.01, 0.01}
			local domainN=table.getn(domain)
			if string.sub(name, 1,3)=="StR" then -- not enough samples for regression learning
				domain={}
				maxStep={maxStep[domainN+1]}
				domainN=0
			end

			for i=1, domainN do
				addCoef(name, domain[i], maxStep[i])
			end
			addConst(name, maxStep[domainN+1])
		end

		local endVar=0

		local param=self._setStageParameters
		local stageParam=param[2]
		if stageParam.nvar then endVar=self.startSeg+stageParam.nvar-1 end

		for i=self.startSeg+1, endVar+1 do
			local seg1=self.sequence[i-1]
			local seg2=self.sequence[i]
			local en=seg1.grpName.."_"..seg1.name.."_"..seg2.name
			-- edge dependent

			if etarget then
				for ietarget, etgt in ipairs(etarget) do
					add(etgt, en, 0.01)
				end
			end

			if econsttarget then
				for iectarget,ectarget in ipairs(econsttarget) do
					add2(ectarget.."_"..en, 0.01)
				end
			end
		end

		for i=self.startSeg, endVar do
			local seg=self.sequence[i]
			local segn=seg.grpName.."_"..seg.name
			-- segment dependent
			if starget then
				for istarget, starget in ipairs(starget) do
					add(starget, segn, 0.01)
				end
			end
			if sconsttarget then
				for isctarget, sctarget in ipairs(sconsttarget) do
					add2(sctarget.."_"..segn, 0.01)
				end
			end
		end
	end

	if gtarget then
		for igtarget, gt in ipairs(gtarget) do
			add2(gt[1], gt[2])
		end
	end

	-- self:restoreStates(self.stage, self.subStage) will be called later in objectivefunction
	--  if regResult==nil then
	--     regResult={}
	--     regResult.target=target
	--  end
	
end
    

function Tracker:updateCurrSeg(i, syn, endSeg, noConservativePrediction)
	local info=syn.trackingInfo -- targetPose foregoes currentPose by PDservoLatency

	if self.sequence~=nil then
		local seq=self.sequence
		if not noConservativePrediction and self.endFrac and seq.curSeg==endSeg and info.frac>self.endFrac then
			return "normal ending"
		end
		if info.currSeg~=seq[seq.curSeg] then

			fineLog( info.currSeg.name, syn:timeScale(), i, seq.curSeg.."/"..seq.N)
			if seq.curSeg==endSeg then
				return "normal ending"
			else
				seq.curSeg=seq.curSeg+1
				if info.currSeg~=seq[seq.curSeg] then
					--dbg.console()
					fineLog("error4")
					return "error"
				end

				local stageParam=self._setStageParameters[2]
				local synParams={'desiredSpeedZ', 'desiredTurningSpeed'}
				for isynParam, synParam in ipairs(synParams) do
					if stageParam and stageParam[synParam] then	       
						local dvzIndex=seq.curSeg
						local tn=table.getn(stageParam[synParam])
						if dvzIndex>tn then dvzIndex=tn end


						syn[synParam]=stageParam[synParam][dvzIndex] 
						assert(syn[synParam])

						if not noConservativePrediction and useCase.conservativePrediction then
							syn[synParam]=syn[synParam]*useCase.conservativePrediction
						end
						assert(syn[synParam])
					end
				end
			end
		end
	end
end
