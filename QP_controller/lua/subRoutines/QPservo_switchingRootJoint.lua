require('subRoutines/ConvexHull2D')
require("module")
require('subRoutines/QPservo')

-- PDServo class
--class 'QPservo'
QPservo_switchingRootJoint=LUAclass()

-- the order in which functions are called:
--1. sampleCurrPose
--2. sampleTargetPoses
--3. QPsolve
--4. addBoneObjective2 (5 times)
--5. addMomentumObjective
--6. stepSimul

function QPservo_switchingRootJoint:computeContact(sim, swingFoot)
	self.servo:computeContact(sim, swingFoot)
	local servo=self.servo
	local servoL=self.servoL
	--print('theta before', self.servo.theta)
	--print('servo dtheta', servo.dtheta, servoL.dtheta)
	servoL.theta, servoL.dtheta=self.helperL:_copyStateFromSource(servo.theta, servo.dtheta)
	self.servoL:computeContact(sim, swingFoot)
	self.state=self.servo.state
end

function QPservo_switchingRootJoint:__init(dofInfo, timestep,integrator, simulator, locosim)
	--_checkpointsbak=_checkpoints
	--_checkpoints=array()
	self.servo=QPservo( dofInfo,  timestep, integrator, simulator)
	--_checkpoints=_checkpointsbak
	self.theta=self.servo.theta
	self.theta_d=self.servo.theta_d
	self.theta_d_pd=self.servo.theta_d_pd
	self.dtheta_d=self.servo.dtheta_d
	self.ddtheta_d=self.servo.ddtheta_d
	self.helperL=VRMLloaderViewHelper(simulator, MotionLoader.LEFTANKLE, vector3(0,0,0))
	--self.helperL=VRMLloaderViewHelper(simulator, MotionLoader.HIPS, vector3(0,0,0))
	local h=self.helperL
	self.floors=locosim:setFloor(h.simulator)
	if false then
		-- draw obstacles
		-- remove background so that obstacles can be seen.
		local rootnode =RE.ogreRootSceneNode()
		if rootnode~=nil then
			local bgnode=RE.createChildSceneNode(rootnode , "BackgroundNode")
			self.skinFloors={}
			for i=1,table.getn(self.floors) do
				self.skinFloors[i]=RE.createVRMLskin(self.floors[i], false)
				self.skinFloors[i]:scale(100,100,100)
				self.skinFloors[i]:setPose(h.simulator:getWorldState(i))
			end
		end
	end
	self.servoL=QPservo(h.skel.dofInfo,timestep, integrator, h.simulator)
	--self.servoL=self.servo
end

function QPservo_switchingRootJoint:initQPservo(startf, endf,motionDOF, dmotionDOF, ddmotionDOF, motionDOF_pdtarget)
	self.servo:initQPservo(startf, endf,motionDOF, dmotionDOF, ddmotionDOF, motionDOF_pdtarget)
end
function QPservo_switchingRootJoint:_calcDesiredAcceleration()
	self.servo:_calcDesiredAcceleration()
end
function QPservo_switchingRootJoint:sampleCurrPose(sim)
	self.servo.motionDOF=self.motionDOF
	self.servo.dmotionDOF=self.dmotionDOF
	self.servo.ddmotionDOF=self.ddmotionDOF
	self.servo.motionDOF_pdtarget=self.motionDOF_pdtarget
	self.servo:sampleCurrPose(sim)
end
function QPservo_switchingRootJoint:sampleTargetPoses( frame)
	self.servo:sampleTargetPoses(frame)
end
function QPservo_switchingRootJoint:stepSimul(sim, impulse)
	self.servoL:stepSimul(sim, impulse)
	self.helperL:copyStateToSource()
	self.servo:sampleCurrPose()
	--print('theta after', self.servo.theta)

end
function QPservo_switchingRootJoint:addBoneObjective(sim, ibone, localpos, desiredAcc, weight, onlyHorizontal)
	self.servoL:addBoneObjective(sim, ibone, localpos, desiredAcc, weight, onlyHorizontal)
end

function QPservo_switchingRootJoint:addBoneObjective2(sim, ibone, localpos, desiredAccRot, desiredAcc, weight, weightLin)
	self.servoL:addBoneObjective2(sim, ibone, localpos, desiredAccRot, desiredAcc, weight, weightLin)
end
function QPservo_switchingRootJoint:addMomentumObjective(sim,desiredDotAngMomentum, desiredDotLinMomentum,weight_ang, weight_lin)
	self.servoL:addMomentumObjective(sim,desiredDotAngMomentum, desiredDotLinMomentum,weight_ang, weight_lin)
end
function QPservo_switchingRootJoint:QPsolve(sim, state, dstate, spprtImportance)
	-- input: self.theta, self.dtheta, self.desiredacceleration
	--        simulator state,  contact
	local servo=self.servo
	local servoL=self.servoL
	self.helperL.skel:convertSourceDOFexceptRoot(servo.desiredacceleration, servoL.desiredacceleration)
	--print('servo desiredacc',servo.desiredacceleration)
	--print('servoL desiredacc',servoL.desiredacceleration)
	assert(servoL.simulator==self.helperL.simulator)
	servoL:QPsolve(servoL.simulator, servoL.theta , servoL.dtheta, spprtImportance)
end
