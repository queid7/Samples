--require("OsModel")

WalkingDeviceBase = LUAclass()


function WalkingDeviceBase:__init(mSynthesis, mOsim)

	print("walkingDeviceBase: init()")

	-- OsModel
	self.mOsim = mOsim or nil
	assert(self.mOsim ~= nil)
	self.dofInfo = mOsim.mLoader.dofInfo
	self.skel = self.dofInfo:skeleton()
	-- LocoSimMuscle, LocoSynthesis
	self.mSynthesis = mSynthesis or nil
	assert(self.mSynthesis ~= nil)
	-- simulator
	self.simulator = self.mSynthesis.simulator or nil
	assert(self.simulator ~= nil)

	-- DOF of simulation
	self.numDof = self.mOsim:getNumActualDof()

	-- valid index of actual DOF controlled by device
	-- hip_r_x, hip_l_x
	self.validJoint = {"hip_r", "hip_l"}
	self.validDofIndices = {}
	for k, v in pairs(self.validJoint) do
		local bone = self.skel:getBoneByName(v):treeIndex()
		print(bone)
		self.validDofIndices[v.."_x"] = self.dofInfo:DOFindex(bone, 0) - 1  -- ddq format

		print(self.validDofIndices[v.."_x"])
	end

	-- output
	-- outputTorques = { frame1 output = vectorn(), frame2 output = vectorn(),.. curframe output = vectorn()}
	self.outputTorques = {}

	-- internal values
	self.curframe = nil
	self.poses = {}

end

function WalkingDeviceBase:__finalize()


end


function WalkingDeviceBase:setCurPose()

	self.poses[self.curframe] = vectorn(self.numDof)
	self.simulator:getWorldState(0):getPoseDOFfromGlobal(self.poses[self.curframe])

end


function WalkingDeviceBase:generateTorque(frame)
	-- have to override at inherited class

	ddq = vectorn()

	ddq:setSize(self.numDof)
	ddq:setAllValue(0.0)

	--								| z
	--								|
	--								|________  y
	--								/
	--							   /
	--							  / x
	-- hip_r 6~8, hip_l 14~16
	-- ddq:set(8,50)
	print("make device torque")
--	ddq:set(14,50)

--	self.outputTorques[1] = vectorn()
--	self.outputTorques[1]:setSize(self.numDof)
--	self.outputTorques[1]:setAllValue(0.0)

	if frame == nil then
		assert(self.curframe > 0, "self.curframe must be > 0")
		self.outputTorques[self.curframe] = ddq
	else
		assert(frame > 0, "frame must be > 0")
		self.outputTorques[frame] = ddq
	end
end


function WalkingDeviceBase:getTorque(output, frame)
-- get generated torque at frame or curframe


	local torque

	if frame == nil then
		assert(self.curframe > 0, "self.curframe must be > 0")
		torque = self.outputTorques[self.curframe]
	else
		assert(frame > 0, "frame must be > 0")
		torque = self.outputTorques[frame]
	end

	if torque ~= nil then
		output:assign(torque)
	else
		output:setAllValue(0.0)
	end

end


function WalkingDeviceBase:oneStep()
-- proceed ont step of a walking device

	if self.curframe == nil then
		self.curframe = 1
	else
		assert(self.curframe > 0, "self.curframe must be > 0")
		self.curframe = self.curframe + 1
	end

	self:setCurPose()
	self:generateTorque()

end
