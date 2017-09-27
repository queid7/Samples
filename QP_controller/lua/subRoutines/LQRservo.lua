require('RigidBodyWin/subRoutines/control/FullbodyLQR')
-- LQRservo class
LQRservo=LUAclass(FullbodyLQR)

function LQRservo:__init(skel, dt, q, qd)
	local dofInfo=skel.dofInfo
	FullbodyLQR.__init(self,skel, 9.8, dt, q, qd)
end

function LQRservo:initLQRservo(startf, endf,motionDOF, dmotionDOF)
	self.startFrame=startf
	self.endFrame=endf
	self.currFrame=startf
	self.deltaTime=0
	self.motionDOF=motionDOF
	self.dmotionDOF=dmotionDOF
end

-- generate FBtorque
function LQRservo:generateTorque(simulator)

	self.currFrame=(simulator:currentTime()+self.deltaTime)*model.frame_rate+self.startFrame
	--print(self.currFrame) -- extremely slow.
	if self.currFrame>self.endFrame-1 then
		simulator:getLinkData(0, DynamicsSimulator.JOINT_VALUE, self.theta)
		simulator:getLinkData(0, DynamicsSimulator.JOINT_VELOCITY, self.dtheta)
		return false
	end

	self:_generateTorque(simulator, self.currFrame)
	return true
end

function LQRservo:_generateTorque(simulator, frame)

	simulator:getLinkData(0, DynamicsSimulator.JOINT_VALUE, self.theta)
	simulator:getLinkData(0, DynamicsSimulator.JOINT_VELOCITY, self.dtheta)

	-- desired (target) pose
	self.motionDOF:samplePose(frame, self.theta_d)
	self.dmotionDOF:sampleRow(frame, self.dtheta_d)

	self:calcMassMatrix()
	self:LQRplanning() 
end

function LQRservo:rewindTargetMotion(simulator)
	self.deltaTime=-1*simulator:currentTime()
end
