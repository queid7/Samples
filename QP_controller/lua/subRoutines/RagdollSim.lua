
require("RigidBodyWin/subRoutines/PDservo")
require("RigidBodyWin/subRoutines/IDservo")
require("RigidBodyWin/subRoutines/QPservo")
require("RigidBodyWin/subRoutines/LQRservo")
require("RigidBodyWin/subRoutines/JTcontroller")

RagdollSim=LUAclass ()

SimulationMode={Ragdoll=0, PoseMaintain=1, TrackMotion=2}
function RagdollSim.createSimulatorParam()
	-- collect global variables
	local sp={}
	sp.simulator=simulator
	sp.useInverseDynamics=useInverseDynamics
	sp.useQPsolver=useQPsolver
	sp.timestep=timestep
	sp.integrator=integrator
	sp.debugContactParam=debugContactParam
	sp.simulationMode=simulationMode
	sp.useHighGain=useHighGain
	return sp
end
function RagdollSim:__init(loader, drawSkeleton, motdof, simulatorParam)
	if drawSkeleton==nil then drawSkeleton = true end
	simulatorParam=simulatorParam or self.createSimulatorParam()
	self.skin=RE.createVRMLskin(loader, drawSkeleton)
	self.skin:setThickness(0.03)
	self.skin:scale(100,100,100)

	self.simulator=createSimulator(simulatorParam.simulator)
	self.simulator:registerCharacter(loader)
	local floor=mFloor or VRMLloader("../Resource/mesh/floor_y.wrl")
	self.simulator:registerCharacter(floor)
	registerContactPairAll(model, loader, floor, self.simulator)   
	if simulatorParam.useInverseDynamics then
	--if true then
		self.simulator:registerContactQueryBone(0, loader:getBoneByVoca(MotionLoader.LEFTANKLE))
		self.simulator:registerContactQueryBone(1, loader:getBoneByVoca(MotionLoader.RIGHTANKLE))
	end

	self.simulator:init(simulatorParam.timestep, simulatorParam.integrator)

	self.simulator:setSimulatorParam("debugContact", simulatorParam.debugContactParam) 
	self.simulator:setSimulatorParam("contactForceVis", {0.001,0.001,0.001})
	self.simulator:setSimulatorParam("penaltyDepthMax", {0.0005})
	-- adjust initial positions

	self.motionDOF=motdof
	self.simulationParam=simulatorParam

	self.controlforce=vectorn(self.motionDOF:numDOF())
	if self.simulationParam.simulationMode==SimulationMode.PoseMaintain then
		if self.simulationParam.useInverseDynamics then
			self.poseMaintaner=PoseMaintainerID(loader)
		else
			self.poseMaintaner=PoseMaintainer()
		end
		self.poseMaintaner:init(loader, self.simulator, k_p, k_d)

	elseif self.simulationParam.simulationMode==SimulationMode.TrackMotion then
		for i=0, self.motionDOF:numFrames()-1 do
			self.motionDOF:row(i):set(1,self.motionDOF:row(i):get(1)+(initialHeight or 0) )
		end
		self.DMotionDOF=calcDerivative(self.motionDOF)
		for i=0, self.motionDOF:numFrames()-1 do
			self.DMotionDOF:row(i):assign(MotionDOF.convertDPoseToDState(self.motionDOF:row(i), self.DMotionDOF:row(i), 1))
		end
		self.DDMotionDOF=self.DMotionDOF:derivative(120)

		if self.simulationParam.useInverseDynamics then
			self.pdservo=IDservo(loader.dofInfo,simulatorParam.timestep, simulatorParam.integrator)
			self.pdservo:initIDservo(model.start, self.motionDOF:numFrames(),
			self.motionDOF, self.DMotionDOF, self.DDMotionDOF)
			if false then
				self.jcache=JacobianCache(loader, self.simulator)
				self.jtcontroller=JTcontroller(loader, self.jcache)
			end
		elseif self.simulationParam.useQPsolver then
			self.pdservo=QPservo(loader.dofInfo,simulatorParam.timestep, simulatorParam.integrator, self.simulator)
			self.pdservo:initQPservo(model.start, self.motionDOF:numFrames(),
			self.motionDOF, self.DMotionDOF, self.DDMotionDOF)
		else	 
			if true then
				self.pdservo=PDservo(loader.dofInfo)
				self.pdservo:initPDservo(model.start, self.motionDOF:numFrames(),
				self.motionDOF, self.DMotionDOF)
			else
				-- doesn't work well
				self.pdservo=LQRservo(loader,simulatorParam.timestep,100,10)
				self.pdservo:initLQRservo(model.start, self.motionDOF:numFrames(),
				self.motionDOF, self.DMotionDOF)
				self.pdservo.simulator=self.simulator
			end
		end
		--	   local dofInfo=loader.dofInfo
		--	   local ibone=loader:getBoneByName("LeftShoulder"):treeIndex()
		--	   local signal=self.motionDOF:matView():range(model.start, model.start+200,
		--						   dofInfo:startT(ibone),
		--						   dofInfo:endR(ibone))
		--	   math.plotMat(signal)

	end

	if motdof then
		model.start=math.min(model.start, self.motionDOF:numFrames()-1)
		initialState=vectorn()
		initialState:assign(self.motionDOF:row(model.start))
		-- set global position
		-- initialState:set(0,0)
		-- initialState:set(1,initialState:get(1)+(initialHeight or 0) )
		-- initialState:set(2,0)

		print("initialState=",initialState)
		self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, initialState)

		if self.DMotionDOF then
			local initialVel=self.DMotionDOF:row(model.start):copy()
			self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, initialVel)
		end
		self.simulator:initSimulation()
	elseif manualAdjust==true then
		-- manually adjust
		local wldst=self.simulator:getWorldState(0)
		wldst:localFrame(loader:getBoneByTreeIndex(1)).rotation:setRotation(vector3(1, 0, 0), toRadian(-10))
		wldst:localFrame(loader:getBoneByTreeIndex(1)).translation:radd(vector3(0, initialHeight, 0))
		wldst:localFrame(loader:getBoneByName(model.bones.left_knee)).rotation:setRotation(vector3(1, 0, 0), toRadian(10))
		wldst:forwardKinematics()

		self.simulator:setWorldState(0)
	else
		local wldst=self.simulator:getWorldState(0)
		wldst:localFrame(loader:getBoneByTreeIndex(1)).rotation:identity()
		wldst:localFrame(loader:getBoneByTreeIndex(1)).translation:radd(vector3(0, initialHeight, 0))
		wldst:forwardKinematics()
		self.simulator:setWorldState(0)
	end
	--	debug.debug()
	self.skin:setPose(self.simulator,0)

	self.skin:setMaterial("lightgrey_transparent")

	--self.simulator.setGVector(vector3(0,0,9.8))
	self.simulator:setGVector(vector3(0,9.8,0))
	self.simulator:initSimulation()
	self.loader=loader
	self.floor=floor -- have to be a member to prevent garbage collection
end
function RagdollSim:setFrame(iframe, initialHeight)
	self.pdservo.startFrame=iframe
	self.pdservo:rewindTargetMotion(self.simulator)
	local initialState=vectorn()
	initialState:assign(self.motionDOF:row(iframe))
	-- set global position
	initialState:set(1,initialState:get(1)+initialHeight)

	self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, initialState)

	if self.DMotionDOF then
		local initialVel=self.DMotionDOF:row(iframe):copy()
		self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, initialVel)
	end
	self.simulator:initSimulation()
end
function RagdollSim:__finalize()
	-- remove objects that are owned by C++
	if self.skin~=nill then
		RE.remove(self.skin)
		self.skin=nil
	end
	self.simulator=nil

end
function RagdollSim:frameMove(niter)
	--assert(math.floor(niter)==niter)
	--		debug.debug()
	temp=vectorn()
	self.controlforce:zero()
	self.simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, temp)

	for iter=1,niter do
		--	 print("stepSimul")
		if self.simulationParam.simulationMode==SimulationMode.PoseMaintain then
			self.poseMaintaner:generateTorque(self.simulator)
			self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_TORQUE, self.poseMaintaner.controlforce)
		elseif self.simulationParam.simulationMode==SimulationMode.TrackMotion then
			local maxForce=9.8*80
			if self.simulationParam.useHighGain then
				maxForce=9.8*3000000
			end
			if not self.pdservo:generateTorque(self.simulator, maxForce) then
				self.pdservo:rewindTargetmotion(self.simulator)
				self.pdservo:generateTorque(self.simulator, maxForce)
			end

			local controlforce=self.pdservo.controlforce
			if false then
				if self.simulator:queryContactAll():size()>0 then
					self.jtcontroller:generateTorque(self.simulator, vector3(0,-9.8*80,0))
					local comjt=self.jtcontroller.COMjt:Transpose()
					assert(comjt:rows()==3)

					comjt=comjt:range(1,2,0, comjt:cols())

					local projcf=math.nullspaceProjector(comjt)*controlforce:column()
					controlforce:assign(projcf:column(0))
					--controlforce:radd(self.jtcontroller.controlforce)
				end
			end

			if self.pdservo.stepSimul then
				self.pdservo:stepSimul(self.simulator)
			else
				self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_TORQUE, controlforce)
				if self.drawDebugInformation then
					self.simulator:drawDebugInformation()
				end
				self.simulator:stepSimulation()
			end
			self.controlforce:radd(controlforce)
		end
		self.skin:setPose(self.simulator,0)				
	end

	self.controlforce:rdiv(niter)
end

