require("WalkingDeviceBase")

WalkingDeviceRAA = LUAclass(WalkingDeviceBase)

function WalkingDeviceRAA:__init(mSynthesis, mOsim, timestep)

	print("WalkingDeviceRAA : __init()")
	WalkingDeviceBase.__init(self, mSynthesis, mOsim)

	self.dPoses = {}
	self.ddPoses = {}

	self.timestep = timestep

	-- pattern
	self.valid = false
	self.outputGlobal = self.mSynthesis.outputGlobal
	self.refTime = {}			-- key : frame, value : refTime
	self.refTimeInvInteger = {}		-- key : reftime (interger), value : floor(frame)



	-- plot
	self.plotdata = true
	if self.plotdata then
		local dataname = useCase.modelName..'__'..g_mode
		mFiles = {}
		for k,v in pairs(self.validDofIndices) do
			mFiles[v] = io.open(dataname..'__deviceRAA'..'__'..k, 'w')
			mFiles[v]:write('frame\t')
			mFiles[v]:write('ddPoses\t')
			mFiles[v]:write('generated ddPoses\t')
		end
	end


end

function WalkingDeviceRAA:__finalize()

	if self.plotdata and mFiles ~= nil then
		for k,v in pairs(mFiles) do
			io.close(v)
		end
	end

	WalkingDeviceBase.__finalize(self)

end

function WalkingDeviceRAA:setRefTime(frame)
	-- set RefTime table and RefTimeInv table

	local f = frame or self.curframe

	assert(f > 0)

	self.refTime[f] = self.outputGlobal:getRefTime(f-1)

	if f == 1 then
		self.refTimeInvInteger[math.floor(self.refTime[f])] = f
	elseif math.floor(self.refTime[f-1]) < math.floor(self.refTime[f]) then
		self.refTimeInvInteger[math.floor(self.refTime[f])] = f
	end

end

function WalkingDeviceRAA:getRefFrame(time)
	-- search and return related frame (real value)
	
	local maxi = table.maxn(self.refTimeInvInteger)

	if maxi == 0 then
		return nil
	end

	local si = math.floor(time)
	local sf = self.refTimeInvInteger[si]
	local ef = self.refTimeInvInteger[si+1] or table.maxn(self.refTime)

	while (sf + 1) < ef do
		if time < self.refTime[math.floor((sf+ef)/2)] then
			ef = math.floor((sf+ef)/2)
		else
			sf = math.floor((sf+ef)/2)
		end
	end

	local diff = self.refTime[ef] - self.refTime[sf]
	local f = (self.refTime[ef] - time) / diff * sf + (time - self.refTime[sf]) / diff * ef

	return f
end

function WalkingDeviceRAA:calcdPose(frame)

	assert(frame ~= nil)
	assert(frame > 0)

	if frame == 1 then
		return
	end

	local dPose = vectorn()
	dPose:setSize(self.numDof)
	dPose:setAllValue(0)

	assert(self.poses[frame] ~= nil and self.poses[frame-1] ~= nil)

	dPose:assign((self.poses[frame] - self.poses[frame-1])/self.timestep)
	self.dPoses[frame-1] = dPose

end

function WalkingDeviceRAA:calcCurdPose()
	self:calcdPose(self.curframe)
end

function WalkingDeviceRAA:calcddPose(frame)

	assert(frame ~= nil)
	assert(frame > 0)

	if frame == 1 then
		return
	end

	local ddPose = vectorn()
	ddPose:setSize(self.numDof)
	ddPose:setAllValue(0)

	assert(self.dPoses[frame] ~= nil and self.dPoses[frame-1] ~= nil)

	ddPose:assign((self.dPoses[frame] - self.dPoses[frame-1])/self.timestep)
	self.ddPoses[frame-1] = ddPose

end

function WalkingDeviceRAA:calcCurddPose()
	if self.curframe > 2 then
		self:calcddPose(self.curframe - 1)
	end
end

function WalkingDeviceRAA:getInterpolatedddPose(frame)
	-- frame is real value
	
	assert(frame ~= nil)
	assert(frame >= 1)

	if math.floor(frame) == math.ceil(frame) then
		return self.ddPoses[frame]:copy()
	else
		return (math.ceil(frame) - frame) * self.ddPoses[math.floor(frame)] + (frame - math.floor(frame)) * self.ddPoses[math.ceil(frame)]
	end
end

function WalkingDeviceRAA:generateTorque()

	local torque = vectorn()
	torque:setSize(self.numDof)
	torque:setAllValue(0)
	
	if self.valid == false then
		self.outputTorques[self.curframe] = torque
		return
	end

	local curRefTime = self.refTime[self.curframe]
	local patternRefTime = curRefTime - math.floor((math.floor(curRefTime)-1)/2) * 2

	print(patternRefTime)


	local ddPose = self:getInterpolatedddPose(self:getRefFrame(patternRefTime))

	for k, v in pairs(self.validDofIndices) do	-- for hip_l_x, hip_r_x (defined in WalkingDeviceBase)
--	for v = 6,21 do								-- means all joints
		torque:set(v, ddPose:get(v))
	end

	print(torque)

	self.outputTorques[self.curframe] = torque
	
--	WalkingDeviceBase.generateTorque(self, nil)

end

function WalkingDeviceRAA:findPattern()

	if self.refTime[self.curframe] > 3 then
		self.valid = true
	end
end

function WalkingDeviceRAA:oneStep()

	if self.curframe == nil then
		self.curframe = 1
	else
		assert(self.curframe > 0, "self.curframe must be > 0")
		self.curframe = self.curframe + 1
	end

	self:setCurPose()
	-- refTime system
	self:setRefTime()

	-- calculate dPose(dq), ddPose(ddq)
	self:calcCurdPose()
	self:calcCurddPose()
	-- find pattern
	self:findPattern()



	self:generateTorque()


	if self.plotdata and self.curframe > 2 then
		local frame = self.curframe - 2
		for k, v in pairs(mFiles) do
			v:write(frame..'\t')
			v:write(self.ddPoses[frame]:get(k)..'\t')
			v:write(self.outputTorques[frame]:get(k)..'\t')
			v:write('\n')
		end
	end


end
