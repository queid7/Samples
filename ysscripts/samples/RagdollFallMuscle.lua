require("config")

package.projectPath='../Samples/QP_controller/'
package.path=package.path..";../Samples/QP_controller/lua/?.lua" --;"..package.path
package.path=package.path..";../Samples/classification/lua/?.lua" --;"..package.path
package.path=package.path..";../Resource/classification/lua/?.lua" --;"..package.path
require('RagdollFall')
require('subRoutines/ConvexHull2D')
require("module")
--require('subRoutines/QPservo')

package.path=package.path..";../Samples/ysscripts/samples/?.lua" --;"..package.path
--require("IPC_based/useCases")
require("useMuscles")

package.path=package.path..";../Samples/ysscripts/lib/?.lua" --;"..package.path
require('QPservo2')
require("utilfunc")
require("OsModel")


-- servo method
servoMethod=ServoMethods.QP
timestep=1/120

-- rendering settings
--g_renderMotion = true
g_renderMotion = false

--local zeroInitVel = true
--local gravity=vector3(0,0,0)
--local initialHeight = .1

local zeroInitVel = false
local gravity=vector3(0,9.8,0)

do
	-- chain2
	--wrlpath = '../Resource/motion/opensim/chain2_ball.wrl'
	--wrlpath = '../Resource/motion/opensim/chain2_hinge.wrl'

	--wrlpath = '../Resource/motion/opensim/chain2_ball_mscl1.wrl'
	--luamsclpath = '../Resource/motion/opensim/chain2_ball_mscl1.luamscl'

	--wrlpath = '../Resource/motion/opensim/chain2_ball_mscl4.wrl'
	----luamsclpath = '../Resource/motion/opensim/chain2_ball_mscl4.luamscl'
	--luamsclpath = '../Resource/motion/opensim/chain2_ball_mscl4_weak.luamscl'

	--wrlpath = '../Resource/motion/opensim/chain2_ball_mscl4_end.wrl'
	--luamsclpath = '../Resource/motion/opensim/chain2_ball_mscl4_end.luamscl'

	--wrlpath = '../Resource/motion/opensim/chain2_hinge_mscl2_pathpoint.wrl'
	----luamsclpath = '../Resource/motion/opensim/chain2_hinge_mscl2_pathpoint.luamscl'
	--luamsclpath = '../Resource/motion/opensim/chain2_hinge_mscl2_conditional.luamscl'

	wrlpath = '../Resource/motion/opensim/chain3_hinge_mscl2_biarti.wrl'
	--luamsclpath = '../Resource/motion/opensim/chain3_hinge_mscl2_biarti.luamscl'
	luamsclpath = '../Resource/motion/opensim/chain3_hinge_mscl2_biarti_pp.luamscl'

	--wrlpath = '../Resource/motion/opensim/chain3_ball_mscl4end.wrl'
	--luamsclpath = '../Resource/motion/opensim/chain3_ball_mscl4end.luamscl'
	--luamsclpath = '../Resource/motion/opensim/chain3_ball_mscl4_1joint.luamscl'

	--wrlpath = '../Resource/motion/opensim/chain3_ball_mscl1end.wrl'
	--luamsclpath = '../Resource/motion/opensim/chain3_ball_mscl1end.luamscl'

	--motpath = '../Resource/motion/opensim/chain2_rotY.bvh'
	--motpath = '../Resource/motion/opensim/chain2_rotZ.bvh'
	--motpath = '../Resource/motion/opensim/chain2_sinZ90.bvh'
	--motpath = '../Resource/motion/opensim/chain2_sinZ180.bvh'
	--motpath = '../Resource/motion/opensim/chain2_rotYZ.bvh'
	--motpath = '../Resource/motion/opensim/chain3_rotZ.bvh'
	--motpath = '../Resource/motion/opensim/chain3_sinZ90.bvh'
	--motpath = '../Resource/motion/opensim/chain3_sinZ60.bvh'
	motpath = '../Resource/motion/opensim/chain3_sinZ60_norm.bvh'

	--zeroInitVel = true
	--gravity=vector3(0,0,0)
	--initialHeight = .5
end

function _start()
	RE.renderer():fixedTimeStep(true)
	mEventReceiver=EVR()

	do
		--use useCase
		model = scenarios.toModel(useCase.scenario)
		wrlpath = model.wrlpath
		luamsclpath = model.luamsclpath
		motpath = model.mot_file
	end
	
	do
		----use custom setting
		--model = model_files.default

		--useCase = {}
		--useCase.k_p_ID=120
		--useCase.k_d_ID=2.*math.sqrt(120)
		--useCase.ddqObjWeight=10000000

		----useCase.actuationType = ActuationType.tau
		----useCase.actuationType = ActuationType.ft
		----useCase.actuationType = ActuationType.a
		--useCase.actuationType = ActuationType.u

		--useCase.tauObjWeight=1
		--useCase.ftObjWeight=1
		--useCase.aObjWeight=10
		--useCase.lambdaObjWeight=10

		--useCase.excludeRoot = true

		--useCase.tauMax = 400
		--useCase.ftMax = 4000
	end

	mOsim = OsModel(wrlpath, luamsclpath, model)
	mLoader = mOsim.mLoader

	mLoader:printHierarchy()
	MotionLoader.setVoca(mLoader, model.bones)
	if useCapturedInitialPose and motpath~=nil then
		local container=MotionDOFcontainer(mLoader.dofInfo, motpath)
		mMotionDOF=container.mot
	end
	mFloor=MainLib.VRMLloader("../Resource/mesh/floor_y.wrl")

	---- fix to one pose
	--for r=1,mMotionDOF:rows()-1 do
		--mMotionDOF:row(r):assign(mMotionDOF:row(20))
	--end

	-- fill mot dof
	mOsim.mSkin:applyMotionDOF(mMotionDOF)
	RE.motionPanel():motionWin():detachSkin(mOsim.mSkin)
	RE.motionPanel():motionWin():addSkin(mOsim.mSkin)

	if g_renderMotion then
		mOsim_mot = OsModel(wrlpath, luamsclpath)
		mOsim_mot.mSkin:applyMotionDOF(mMotionDOF)
		RE.motionPanel():motionWin():addSkin(mOsim_mot.mSkin)
	end

	drawSkeleton=this:findWidget("draw skeleton"):checkButtonValue()

	mRagdoll= RagdollMuscle(mLoader, drawSkeleton, mMotionDOF)
	mRagdoll.drawDebugInformation=true

	mSkin2=RE.createVRMLskin(mFloor, false)
	mSkin2:scale(100,100,100)

	mObjectList=Ogre.ObjectList()
end

RagdollMuscle=LUAclass(RagdollSim)

function RagdollMuscle:__init(loader, drawSkeleton, motdof, simulatorParam)
	if drawSkeleton==nil then drawSkeleton = true end
	simulatorParam=simulatorParam or self.createSimulatorParam()

	-- ys
	--self.skin=RE.createVRMLskin(loader, drawSkeleton)
	--self.skin:setThickness(0.03)
	--self.skin:scale(100,100,100)
	self.skin = mOsim.mSkin

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
			self.motionDOF:row(i):set(1,self.motionDOF:row(i):get(1)+(initialHeight or model.initialHeight) )
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
			--self.pdservo=QPservo(loader.dofInfo,simulatorParam.timestep, simulatorParam.integrator, self.simulator)
			self.pdservo=QPservo2(loader.dofInfo,simulatorParam.timestep, simulatorParam.integrator, self.simulator, model)
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

			-- ys
			if zeroInitVel then
				initialVel:setAllValue(0)
				--initialVel:set(0,.52)
			end
			self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, initialVel)
		end
		-- ys
		mOsim:setBoneForwardKinematics(self.simulator:getWorldState(0))
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

	self.simulator:setGVector(gravity)
	--self.simulator:setGVector(vector3(0,0,0))
	
	self.simulator:initSimulation()
	self.loader=loader
	self.floor=floor -- have to be a member to prevent garbage collection	
end

function RagdollMuscle:frameMove(niter)
	--print()
	--RagdollSim.frameMove(mRagdoll, niter)
	--mOsim:drawMuscles(0, mObjectList)
	----mOsim:drawMuscleWithForces(ifr, mObjectList)
end

if EventReceiver then
	--class 'EVR'(EventReceiver)
	EVR=LUAclass(EventReceiver)
	function EVR:__init(graph)
		--EventReceiver.__init(self)
		self.currFrame=0
		self.cameraInfo={}
	end
end

g_prev_iframe = 0
function EVR:onFrameChanged(win, iframe)
	if iframe > g_prev_iframe then
		for f=1,iframe-g_prev_iframe do
			local ifr = g_prev_iframe + f
			print()
			print('simulate frame ', ifr)

			-- simulation
			RagdollSim.frameMove(mRagdoll, 1)
			
			--mOsim:drawMuscles(ifr, mObjectList)
			mOsim:drawMuscles2(ifr, mObjectList)
			--mOsim:drawMuscleWithForces(ifr, mObjectList)
		end	
		g_prev_iframe = iframe
	else
		mOsim:drawRecordedScene(iframe, mObjectList)
	end
end
