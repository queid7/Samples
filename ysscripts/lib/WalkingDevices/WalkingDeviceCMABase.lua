require("WalkingDeviceBase")

WalkingDeviceCMABase = LUAclass(WalkingDeviceBase)

function WalkingDeviceCMABase:__init(mSynthesis, mOsim, timestep)

	print("WalkingDeviceCMABase : __init()")
	WalkingDeviceBase.__init(self, mSynthesis, mOsim)

	self.timestep = timestep

	self.synthesis = mSynthesis
	self.osim = mOsim
	
--	self.parts = {"hipy"}
	self.parts = useCase.devParts
	self.partsDOFindex = {hipx = {R = 6, L = 14}, hipy = {R = 7, L = 15}, hipz = {R=8, L = 16}, knee = {R=9, L=17}, anklex = {R=10, L=18}, ankley = {R=11, L=19}, anklez = {R=12, L=20}, mtp = {R=13, L=21}}
	self.partScales = useCase.partScales

	-- plot
	self.plotdata = false
	if self.plotdata then
		local dataname = useCase.modelName..'__'..g_mode
		mFiles = {}
		for k,v in pairs(self.validDofIndices) do
			mFiles[v] = io.open(dataname..'__deviceCMABase'..'__'..k, 'w')
			mFiles[v]:write('frame\t')
			mFiles[v]:write('ddPoses\t')
			mFiles[v]:write('generated ddPoses\t')
		end
	end

end

function WalkingDeviceCMABase:__finalize()

	if self.plotdata and mFiles ~= nil then
		for k,v in pairs(mFiles) do
			io.close(v)
		end
	end

	WalkingDeviceBase.__finalize(self)

end

function WalkingDeviceCMABase:generateTorque()

	local torque = vectorn()
	torque:setSize(self.numDof)
	torque:setAllValue(0.0)
	

	local curRefTime = self.synthesis.outputGlobal:getRefTime()
	local qRefTime = math.floor(curRefTime)
	local rRefTime = curRefTime % 1.0

	local dev = "dev"
	local t,d,f,scale
	for i, p in ipairs(self.parts) do
		local DOFindex = self.partsDOFindex[p]
		scale = self.partScales[p]
		for s, idx in pairs(DOFindex) do
			t = self.synthesis.outputLocal:sampleKey(qRefTime, dev..p..'t'..s..'mod')
			d = self.synthesis.outputLocal:sampleKey(qRefTime, dev..p..'d'..s..'mod')
			f = self.synthesis.outputLocal:sampleKey(qRefTime, dev..p..'f'..s..'mod')
	--[[
			if t < 0.0 then t = 0.0 elseif t > 1.0 then t = 1.0 end
			if d < 0.0 then d = 0.0 elseif d > 1.0 then d = 1.0 end
			if rRefTime >= t and rRefTime <= t + d then
				torque:set(idx, f * scale)
			end
			]]

--[[
			-- method 1
			if t < -3.0 then t = -3.0 elseif t > 3.0 then t = 3.0 end
			if d < -3.0 then d = -3.0 elseif d > 3.0 then d = 3.0 end
			t = (t + 3.0) / 12.0
			d = 0.5 + (d + 3.0) / 12.0
			if rRefTime >= t and rRefTime <= d then
				torque:set(idx, f * 30)
			end
]]--
			-- method 2
			if t < -1.0 then t = -1.0 elseif t > 1.0 then t = 1.0 end
			if d < -1.0 then d = -1.0 elseif d > 1.0 then d = 1.0 end
			t = t/2.0 + 0.5
			d = d/4.0 + 0.25
			st = t - d
			if st < 0 then st = st + 1 end
			et = t + d
			if et > 1 then et = et - 1 end
			if rRefTime >= st and rRefTime <= et then
				torque:set(idx, f * scale)
			end
		end
	end

	self.outputTorques[self.curframe] = torque
	
--	WalkingDeviceBase.generateTorque(self, nil)

end

function WalkingDeviceCMABase:oneStep()

	if self.curframe == nil then
		self.curframe = 1
	else
		assert(self.curframe > 0, "self.curframe must be > 0")
		self.curframe = self.curframe + 1
	end

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
