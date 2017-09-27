-- namespace
motiongraph={}	

-- a motion graph is a table containing segments.
-- e.g. graph={ jump={ mot=mot1, dmot=dmot1}, walk1={ mot=mot2, dmot=dmot2}, ...}

-- TODO list
-- global variable motiongraph.mSkel should be removed.

-- stitch motions 
function motiongraph.stitch_mot(mot1, mot2, spread)


	local len1=mot1:length()
	local len2=mot2:length()
	local len=len1+len2
	local seg1start=mot1:convertToDeltaRep()
	local seg2start=mot2:convertToDeltaRep()
	
	mot=MotionDOF(mot1.dofInfo)
	mot:changeLength(len)
	
	assert(len1>spread)
	--assert(len2>spread)
	if len2<=spread then
		spread=len2-1
	end
	
	local concatOnly=false
	mot:range_c(0, len1-spread):assign(mot1:range_c(0, len1-spread))
	mot:range_c(len1+spread, len):assign(mot2:range_c(spread, len2))
	
	
	if concatOnly then
		mot:range_c(len1-spread, len1):assign(
			mot1:range_c(len1-spread, len1))
		mot:range_c(len1, len1+spread):assign(
			mot2:range_c(0, spread))
	else
		mot:range_c(len1-spread, len1+spread):stitchDeltaRep(
		mot1:range_c(len1-spread, len1),
		mot2:range_c(0, spread))
	end
	
	
	
	mot1:reconstructData(seg1start)
	mot2:reconstructData(seg2start)
	mot:reconstructData(seg1start)


	return mot
end

function motiongraph.stitch_lmot(mot1, mot2, spread)
	local len1=mot1:length()
	local len2=mot2:length()
	local len=len1+len2
	
	mot=MotionDOF(mot1.dofInfo)
	mot:changeLength(len)
	
	assert(len1>spread)
	--assert(len2>spread)
	if len2<=spread then
		spread=len2-1
	end
	
	local concatOnly=false
	mot:range_c(0, len1-spread):assign(mot1:range_c(0, len1-spread))
	mot:range_c(len1+spread, len):assign(mot2:range_c(spread, len2))
	
	if concatOnly then
		mot:range_c(len1-spread, len1):assign(
			mot1:range_c(len1-spread, len1))
		mot:range_c(len1, len1+spread):assign(
			mot2:range_c(0, spread))
	else
		mot:range_c(len1-spread, len1+spread):stitchDeltaRep(
		mot1:range_c(len1-spread, len1),
		mot2:range_c(0, spread))
	end
	
	return mot
end

function motiongraph.stitch_lmot_online(mot1, mot2, spread)
	local len1=mot1:length()
	local len2=mot2:length()
	local len=len1+len2
	
	mot=MotionDOF(mot1.dofInfo)
	mot:changeLength(len)
	
	if len2<=spread then
		spread=len2-1
	end
	
	mot:range_c(0, len1-1):assign(mot1:range_c(0, len1-1))
	mot:range_c(len1+spread, len):assign(mot2:range_c(spread, len2))
	
	mot:range_c(len1-1, len1+spread):stitchDeltaRep(
	mot1:range_c(len1-1, len1),
	mot2:range_c(0, spread))
	
	return mot
end
	
function motiongraph.append(mot, mot2, spread)

	local mot_prevLen=mot:length()
	assert(mot_prevLen>spread)
	
	mot:changeLength(mot_prevLen+mot2:length())
		
	local mot1=mot:range_c(mot_prevLen-spread, mot_prevLen)
	
	mot:range_c(mot_prevLen-spread, mot:length()):assign(
		motiongraph.stitch_mot(mot1,mot2, spread-1))
	
	--[[local seg2start=mot2:convertToDeltaRep()
	mot2:reconstructData(seg2start)
	
	--local id=InterframeDifference(mot2)
	--id:reconstruct(mot2)
	mot:range_c(mot_prevLen, mot:length()):assign(mot2)]]
end

function motiongraph.append_lmot(mot, mot2, spread)

	local mot_prevLen=mot:length()
	assert(mot_prevLen>spread)
	
	mot:changeLength(mot_prevLen+mot2:length())
		
	local mot1=mot:range_c(mot_prevLen-spread, mot_prevLen)
	
	mot:range_c(mot_prevLen-spread, mot:length()):assign(
		motiongraph.stitch_lmot(mot1,mot2, spread-1))
	
end

function motiongraph.append_lmot_online(mot, mot2, spread)

	local mot_prevLen=mot:length()
	assert(mot_prevLen>spread)
	
	mot:changeLength(mot_prevLen+mot2:length())
		
	local mot1=mot:range_c(mot_prevLen-1, mot_prevLen)
	
	mot:range_c(mot_prevLen-1, mot:length()):assign(
		motiongraph.stitch_lmot_online(mot1,mot2, spread-1))
	
end

-- stitch segments
function motiongraph.stitch(segment1, segment2, spread)
	seg={}
	seg.mot=motiongraph.stitch_mot(segment1.mot, segment2.mot, spread)
	seg.dmot=calcDerivative(seg.mot)
	seg.COMcalculator=COMcalculator:new(motiongraph.mSkel, seg.mot, seg.dmot)
	return seg
end

function motiongraph.createSegmentFromFile(skel, mot_file_name, startFrame)
	seg={}
	local motion=Motion(skel)
	skel:loadAnimation(motion, mot_file_name)
	seg.mot=MotionDOF(skel.dofInfo)
	
	if startFrame~=nil then
		seg.mot:set(motion:range(startFrame, motion:numFrames(),1))
	else
		seg.mot:set(motion)
	end
	seg.mVelocity=matrixn()
	if motion:frameRate()==120 then
		MotionUtil.noiseReduction(seg.mot, 4, seg.mVelocity)
	end
	seg.dmot=calcDerivative(seg.mot)
	return seg
end

function motiongraph.segment(mot, dmot, firstFrame, lastFrame)
	seg={}
	seg.mot=MotionDOF(mot.dofInfo)
	seg.mot:assign(mot:range(firstFrame, lastFrame+1))
	seg.dmot=matrixn()
	seg.dmot:assign(dmot:range(firstFrame, lastFrame+1, 0, dmot:cols()))
	seg.COMcalculator=COMcalculator:new(motiongraph.mSkel, seg.mot, seg.dmot)
	return seg
end

function motiongraph.segment2(mot)

	seg={}
	seg.mot=MotionDOF(mot.dofInfo)
	seg.mot:assign(mot)
	
	seg.dmot=calcDerivative(seg.mot)
	
	seg.COMcalculator=COMcalculator:new(motiongraph.mSkel, seg.mot, seg.dmot)
	

	return seg
end

function motiongraph.saveGraph(graph, filename)
	file=util.BinaryFile(true, filename)
	
	for k,v in pairs(graph) do
		file:packInt(1)	-- exist
		file:pack(k)
		file:pack(v.mot)
		file:pack(v.dmot)
		
		if v.offsetTable~=nil and 
			v.offsetTable:rows()~=0 then
			file:packInt(1)
			file:pack(v.offsetTable)
		else
			file:packInt(0)
		end
	end
	
	file:packInt(0)
	
	-- store links
	
	for k,v in pairs(graph) do
		v.name=k
	end
	
	for k,v in pairs(graph) do
		if v.next~=nil then
			file:packInt(1)
			file:pack(k)
			file:pack(v.next.name)
		end
	end
	
	file:packInt(0)
	file:close()
end
function motiongraph.createGraphFromFile(skel, filename)
	file=util.BinaryFile(false, filename)
	
	motiongraph.mSkel=skel
	motions={}
	
	while file:unpackInt()==1 do
		k=file:unpackStr()
		motions[k]={}
		local seg=motions[k]
		seg.mot=MotionDOF(skel.dofInfo)
		seg.dmot=matrixn()
		file:unpack(seg.mot)
		file:unpack(seg.dmot)
		
		if file:unpackInt()==1 then
			seg.offsetTable=matrixn()
			file:unpack(seg.offsetTable)
		end
		
		seg.COMcalculator=COMcalculator:new(motiongraph.mSkel, seg.mot, seg.dmot)
	end
	
	-- load links
	while file:unpackInt()==1 do
		k=file:unpackStr()
		n=file:unpackStr()
		motions[k].next=motions[n]
	end
	
	file:close()
	return motions
end

function motiongraph.createHoppingGraph(skel,motionDOF)
	
	motiongraph.mSkel=skel
	motions={} 
	motions.original={}
	motions.original.mot=motionDOF
	motions.original.dmot=calcDerivative(motionDOF)
	
	local mot=motions.original.mot
	local dmot=motions.original.dmot
	
	
	local temp=motiongraph.stitch_mot(mot:range_c(405+19*2, 482), mot:range_c(405, 405+19),18)
	
	motions.stand=motiongraph.segment2(motiongraph.stitch_mot(mot:range_c(405+19, 405+19*2), temp,18)) -- cyclic standing motion
	
	
	motions.stand_to_jump=motiongraph.segment(mot, dmot, 0, 107)
	
	motions.jump_updown=motiongraph.segment(mot, dmot, 107, 179)
	
	motions.jump_after=motiongraph.segment(mot, dmot, 179, 245)
	
	motions.jump_to_jump=motiongraph.stitch(motiongraph.segment(mot, dmot,179, 223), motiongraph.segment(mot, dmot, 70, 105), 30)
	
	-- graph construction
	motions.stand.next=motions.stand
	
	motions.stand_to_jump.next=motions.jump_updown
	motions.jump_updown.next=motions.jump_to_jump
	motions.jump_to_jump.next=motions.jump_updown
	--[[
	motions.stand_to_jump.next=motions.jump_updown
	motions.jump_updown.next=motions.jump_after
	motions.jump_after.next=motions.stand_to_jump]]--	
	
	return motions

end

function motiongraph.createWalkingGraph(skel,motionDOF)
	
	motiongraph.mSkel=skel
	motions={} 
	local original={}
	original.mot=motionDOF
	original.dmot=calcDerivative(motionDOF)
	
	local mot=original.mot
	local dmot=original.dmot
	
	
	local temp=motiongraph.stitch_mot(mot:range_c(66, 77), mot:range_c(55, 66),8)
	
	motions.stand=motiongraph.segment2(temp) -- cyclic standing motion
	
	motions.SRDL=motiongraph.segment(mot, dmot,77 , 151)
	
	motions.RDL=motiongraph.segment(mot, dmot, 352, 370)
	
	motions.LDR=motiongraph.segment(mot, dmot, 370, 388)
		
	-- graph construction
	motions.stand.next=motions.stand
	motions.SRDL.next=motions.LDR
	motions.LDR.next=motions.RDL
	
	return motions
end


COMcalculator={
registered=false
}
function COMcalculator:new(skel, motionDOF, dmotionDOF, o)
	o=o or {}
	assert(skel, "skel is nil ")
	setmetatable(o,self)
	self.__index=self
	o.motionDOF=motionDOF
	o.dmotionDOF=dmotionDOF
	-- you cannot use DynamicsSimulator_SDFAST.
	--o.simulator=DynamicsSimulator_AIST_penalty()  -- buggy. getWorldVelocity?
	
	

	if not o.registered then 
--self.simulator=DynamicsSimulator_UT_penalty() , --언제부턴가 죽음.

	   self.simulator=DynamicsSimulator_AIST_penalty() 
		o.simulator:registerCharacter(skel)
		self.registered=true
	end
	
	o.skel=skel
	o.com=vector3N()
	o.comvel=vector3N()
	
	
	o.com:setSize(motionDOF:numFrames())
	o.comvel:setSize(motionDOF:numFrames())
	
	for i=0, motionDOF:numFrames()-1 do
		local com, vel=COMcalculator._calcCOMvel(o, i)
		o.com:at(i):assign(com)
		o.comvel:at(i):assign(vel)
	end
	
	
	return o
end

function COMcalculator:_calcCOM(iframe)
	self.simulator:setLinkData(0, DynamicsSimulator.JOINT_VALUE, self.motionDOF:row(iframe))
	self.simulator:initSimulation()
	return self.simulator:calculateCOM(0)
end


function COMcalculator:_calcCOMvel(iframe)
	self.simulator:setLinkData(0, DynamicsSimulator.JOINT_VALUE, self.motionDOF:row(iframe))
	self.simulator:setLinkData(0, DynamicsSimulator.JOINT_VELOCITY, self.dmotionDOF:row(iframe))
	self.simulator:initSimulation()
	return self.simulator:calculateCOM(0), self.simulator:calculateCOMvel(0)
end

