--class 'Markers'
--class 'CompareChain'

CompareChainHead={}
function CompareChainHead.__init(self, vrmlLoader)
		local headBone=vrmlLoader:getBoneByVoca(MotionLoader.HEAD)
		self.headIndex=headBone:treeIndex()-1
		self.headBone=self.skel:VRMLbone(self.headIndex)
		self.acc=vector3N()
		self.headposError=vector3N()
end
function CompareChainHead.compareQueue(self)
	local cost_acc=0
	local cost_head=0
	for i=0,self.acc:size()-1 do
		local a=self.acc(i)
		local b=self.headposError(i)
		cost_acc=cost_acc+a.x*a.x+a.y*a.y+a.z*a.z;
			-- acc: /500
			-- vel: *4
			--cost=cost+cost_acc*4/100+cost_head*10
		--cost_acc=math.max(cost_acc,a.x*a.x+a.z*a.z) -- acc
		cost_head=cost_head+b.x*b.x+b.y*b.y+b.z*b.z;
	end
	--cost=cost+cost_acc*4/100+cost_head*10
	--cost=cost+cost_head*5
	self.acc:setSize(0)
	self.headposError:setSize(0)
	--print('vel,head',cost_acc, cost_head)
	return cost_acc*4+cost_head*5
end
function CompareChainHead.addQueue(self, mrdMotion, synthesis)
-- don't use
-- makes simulator states changed
	local simulator=synthesis.simulator
	local skel=self.skel
	--local acc=simulator:getWorldAcceleration(0, self.headBone, self.headBone:localCOM())
	local acc=simulator:getWorldVelocity(0, self.headBone, self.headBone:localCOM())
	self.acc:resize(self.acc:size()+1)
	do
		local headPos=simulator:getWorldState(0):globalFrame(self.headBone):toGlobalPos(self.headBone:localCOM())
		local pelvis=simulator:getWorldState(0):globalFrame(1)
		local currFrame=synthesis.numFrames-1
		local rotY=synthesis:calcRotY(currFrame)
		pelvis.rotation:assign(rotY)
		local refTime=synthesis.outputGlobal.refTime(currFrame)
		local headVel=synthesis.outputLocal:sampleV3(refTime,'lheadvel')
		headVel:rotate(rotY)
		--print(acc, headVel)
		self.acc(self.acc:size()-1):assign(acc-headVel)
		local headposErr=pelvis:toLocalPos(headPos)-synthesis.outputLocal:sampleV3(refTime, 'lhead')
		self.headposError:resize(self.headposError:size()+1)
		self.headposError(self.headposError:size()-1):assign(headposErr)
	end
end

CompareChain=LUAclass()
function CompareChain:__init(vrmlLoader, boneForwardKinematics1, boneForwardKinematics2, weights)
	self.skel=vrmlLoader
	self.chain1=boneForwardKinematics1
	self.chain2=boneForwardKinematics2
	self.points1=vectorn()
	self.points2=vectorn()

	local markerFile
	if type(weights)=="string" then
		markerFile=weights
		weights=nil
	end


	self.markers1=Markers(vrmlLoader, boneForwardKinematics1, markerFile, nil)  
	self.markers2=Markers(vrmlLoader, boneForwardKinematics2, markerFile, nil)  

	if weights~=nil then
		self.weights=weights:copy()
	else
		self.weights=vectorn(self.markers1.markers:size())
		self.weights:setAllValue(1/self.weights:size())
	end
	--   self.metric=math.PointCloudMetric()
	self.metric=math.WeightedPointCloudMetric(self.weights)
end

function CompareChain:compare()

	local skel=self.skel

	local function updatePoints(chain, markers, points)
		local numPoints=markers:numMarkers()
		points:setSize(numPoints*3)
		local pointsView=points:vec3View()

		for imarker=1,numPoints do

			local gpos=markers:calcMarkerPos(chain, imarker)
			pointsView(imarker-1):assign(gpos)
		end
	end

	updatePoints(self.chain1, self.markers1, self.points1)
	updatePoints(self.chain2, self.markers2, self.points2)

	self.metric:calcDistance(self.points1, self.points2)

	if self.metric.errorOccurred==true then
		dbg.console()
	end

	return transf(self.metric.transfB)
end

--class 'CompareChainOri'
CompareChainOri=LUAclass()

function CompareChainOri:__init(vrmlLoader, boneForwardKinematics1, boneForwardKinematics2, weights)
	self.skel=vrmlLoader
	self.chain1=boneForwardKinematics1
	self.chain2=boneForwardKinematics2
	self.compareChain=CompareChain(vrmlLoader, boneForwardKinematics1, boneForwardKinematics2, weights)
	self.metric={} -- only for compatibility
end

function CompareChainOri:compare()

	if true then
		return self.compareChain:compare()
	end

	local skel=self.skel

	local bone=skel:VRMLbone(skel:getTreeIndexByVoca(MotionLoader.CHEST))

	local diff=transf()
	diff:difference(self.chain2:globalFrame(bone), self.chain1:globalFrame(bone))

	local q=quater()

	local avgRDiff=vector3(0,0,0)
	local totalMass=0

	local function isDescendent(child, parent)
		return child:isDescendent(parent)
	end

	for i=1,skel:numBone()-1 do
		local bone=skel:VRMLbone(i)
		q:difference(self.chain2:globalFrame(bone).rotation, self.chain1:globalFrame(bone).rotation)

		local mass=bone:mass()

		if isDescendent(bone, skel:VRMLbone(skel:getTreeIndexByVoca(MotionLoader.LEFTHIP))) or isDescendent(bone, skel:VRMLbone(skel:getTreeIndexByVoca(MotionLoader.LEFTHIP))) then
			mass=0 
		end
		avgRDiff:radd(q:rotationVector()*mass)

		totalMass=totalMass+mass
	end

	diff.rotation:setRotation(avgRDiff/totalMass)
	return diff
end


CompareChain2D=LUAclass()


math.KovarMetric2=LUAclass()

function math.KovarMetric2:__init(skel, npoints)
	self.skel=skel
	--   self.numPoints=skel:numBone()-1
	self.numPoints=npoints
	self.weight=vectorn(self.numPoints)
	-- for i=1, skel:numBone()-1 do
	--    self.weight:set(i-1, skel:VRMLbone(i):mass())
	-- end
	self.weight:setAllValue(1)


	-- local headBone=skel:getBoneByVoca(MotionLoader.HEAD)-- FIX ME
	-- self.headIndex=headBone:treeIndex()-1
	-- self.weight:set(self.headIndex, self.weight(self.headIndex)*3)

	self.weight:rmult(self.numPoints/self.weight:sum())
	local firstAlignY=false
	self.metric=math.KovarMetric(firstAlignY)
	--self.posemetric=math.KovarMetric(true)
	self.posemetric=math.KovarMetric(false)

	self.srcA=matrixn()
	self.srcB=matrixn()
end

function math.KovarMetric2:calcDistance(points1_, points2_)

	local points1=points1_:copy()
	local points2=points2_:copy()

	local weight_y=1
	for i=1,points1:size()-1,3 do
		points1:set(i, points1(i)*weight_y)
		points2:set(i, points2(i)*weight_y)
	end

	local dist=self.metric:calcDistance(points1, points2)
	self.transformedB=self.metric.transformedB
	self.errorOccurred=self.metric.errorOccurred

	local numFrames=points1:size()/3/self.numPoints
	self.srcA:resize(numFrames, self.numPoints*3)
	self.srcB:resize(numFrames, self.numPoints*3)


	local weight=self.weight

	local curc=0
	-- for i=0, numFrames-1 do
	--    for j=0, self.numPoints-1 do
	for i=0, numFrames-1 do
		for j=0, self.numPoints-1 do
			self.srcA:row(i):range(j*3, (j+1)*3):assign(points1:range((curc*3), (curc+1)*3)*weight(j))
			self.srcB:row(i):range(j*3, (j+1)*3):assign(self.transformedB:row(curc)*weight(j))
			curc=curc+1
		end
	end

	local pos=0
	local traj=0
	local diff=vectorn()

	local trajA=vectorn(numFrames*3)
	local trajB=vectorn(numFrames*3)
	for i=0, numFrames-1 do

		local centerA=vector3(0,0,0)
		local centerB=vector3(0,0,0)
		for j=0, self.numPoints-1 do
			centerA:radd(self.srcA:row(i):toVector3(j*3))
			centerB:radd(self.srcB:row(i):toVector3(j*3))
		end
		centerA:scale(1/self.numPoints)
		centerB:scale(1/self.numPoints)
		centerA.y=0
		centerB.y=0

		trajA:setVec3(i*3,centerA)
		trajB:setVec3(i*3,centerB)
		if false then
			local MSE_frame=0
			for j=0, self.numPoints-1 do
				MSE_frame=MSE_frame+(self.srcA:row(i):toVector3(j*3)-centerA):squaredDistance(
				self.srcB:row(i):toVector3(j*3)-centerB)
			end
			pos=pos+MSE_frame/self.numPoints-1 --*MSE_frame
		else
			pos=pos+self.posemetric:calcDistance(self.srcA:row(i), self.srcB:row(i))
		end

	end
	--   g_objectList:registerObject("hhhh", "Curve", "solidblue", self.transformedB:range(0, self.numPoints, 0, 3)*100,0)


	traj=self.metric:calcDistance(trajA*100, trajB*100)

	local vel=0
	for i=1, numFrames-2 do
		local velA=self.srcA:row(i+1)-self.srcA:row(i-1)
		local velB=self.srcB:row(i+1)-self.srcB:row(i-1)
		diff:sub(velB, velA)
		local len=diff:length()
		vel=vel+len*len
	end


	-- local headacc=0


	-- g_objectList:registerObject("head", "LineList", "solidgreen", self.srcA:range(0, self.srcA:rows(), 0, 3)*100,0)

	-- g_objectList:registerObject("head2", "LineList", "solidred", self.srcB:range(0, self.srcA:rows(), 0, 3)*100,0)



	-- for i=1, numFrames-2 do
	--    local headp0=self.srcA:row(i-1):toVector3(self.headIndex*3)
	--    local headp1=self.srcA:row(i):toVector3(self.headIndex*3)
	--    local headp2=self.srcA:row(i+1):toVector3(self.headIndex*3)

	--    headp0.y=0
	--    headp1.y=0
	--    headp2.y=0
	--    -- local headq0=self.srcB:row(i-1):toVector3(self.headIndex*3)
	--    -- local headq1=self.srcB:row(i):toVector3(self.headIndex*3)
	--    -- local headq2=self.srcB:row(i+1):toVector3(self.headIndex*3)

	--    headacc=headacc+(headp0+headp2-2*headp1):length()
	-- end
	--   print("pos, vel=", dist*10000, vel)--, headacc) 
	--   print("dist, pos", dist, pos)
	--   return dist*10000--+vel--+headacc*10
	--   print(pos/numFrames, traj/numFrames)
	return pos/numFrames+traj/numFrames*0.1
end



function CompareChain2D:__init(vrmlLoader, boneForwardKinematics1, boneForwardKinematics2, markerFile)
	self.skel=vrmlLoader
	self.chain1=boneForwardKinematics1
	self.chain2=boneForwardKinematics2
	self.points1=vectorn()
	self.points2=vectorn()
	self.frames=vectorn()

	self.markers1=Markers(vrmlLoader, boneForwardKinematics1, markerFile, nil)  
	self.markers2=Markers(vrmlLoader, boneForwardKinematics2, markerFile, nil)  


	self.lfootIndex=self.markers1:findMarker(MotionLoader.LEFTANKLE)
	self.rfootIndex=self.markers1:findMarker(MotionLoader.RIGHTANKLE)

	--assert(self.lfootIndex~=-1 and self.rfootIndex~=-1)
	--   self.metric=math.KovarMetric(true)

	self.metric=math.KovarMetric2:new(self.skel, self.markers1.markers:size())
	CompareChainHead.__init(self,vrmlLoader)
end

function CompareChain2D.updatePoints(skel,chain, points, markers, baseBoneIndex)

	local numPoints=markers:numMarkers()
	points:setSize(numPoints*3)
	local pointsView=points:vec3View()

	if baseBoneIndex ~= nil then
		for imarker=1,numPoints do
			local gpos=markers:calcMarkerPosFrom(chain, imarker, baseBoneIndex)
			--gpos.y=gpos.y*0.3 -- lower weight for height 
			pointsView(imarker-1):assign(gpos)
			--print(gpos)
		end
	else
		for imarker=1,numPoints do
			local gpos=markers:calcMarkerPos(chain, imarker)
			--gpos.y=gpos.y*0.3 -- lower weight for height 
			pointsView(imarker-1):assign(gpos)
			--print(gpos)
		end
	end
end

function CompareChain2D:drawMatching(objectList)

	local points1=objectList:registerObject("points1", "QuadListZ", "solidred", 
	self.points1:matView(3)*100, 0)

	local points2=objectList:registerObject("points2", "QuadListZ", "solidblue", 
	self.points2:matView(3)*100, 0)

	local points3=objectList:registerObject("points3", "QuadListZ", "solidblue", 
	self.metric.transformedB*100, 0)

	points1:translate(-200,0,0)
	points3:translate(-200,0,0)
end

function CompareChain2D:compareQueue(objectList, seg, objCost,flush)
	RE.output("points1 size", tostring(self.points1:size())..","
	..tostring(self.points2:size())..tostring(self.points1:range(0,math.min(10,self.points1:size()))))

	if self.points1:size()<2  then
		dbg.console()
		return 0,0
	end

	-- dist<= sum of the square distances between all corresponding points

	local penalty=0
	local skipCalcDist=false
	if objCost then
		local cost,_skip=objCost(self,seg,dist)
		--print('cs',cost,_skip)
		penalty=penalty+cost
		
		if _skip~=nil and not flush then
			skipCalcDist=_skip
		end
	end
	local cost=penalty
	--print('penalty', penalty)
--	local cost=0
	local numFrames=0
	if not skipCalcDist then

		local dist=self.metric:calcDistance(self.points1, self.points2)
		-- if g_cdbg then
		-- 	local cdbg=g_cdbg
		-- 	assert(cdbg)
		-- 	array.pushBack(cdbg, {"points", cost, dist,self.points1:copy(), self.points2:copy()})
		-- end

		if self.metric.errorOccurred==true then
			--      debug.debug()
		end

		--   print(penalty*0.01, dist)
		cost=cost+dist
		local dist_head=CompareChainHead.compareQueue(self)
		--print('dist', dist, dist_head*1e-6)
		--cost=cost+dist_head*1e-6

		if objectList~=nil then
			self:drawMatching(objectList)
		end
		numFrames=self.points1:size()/3/self.markers1:numMarkers()
		--   dist=dist*numFrames
		self.points1:setSize(0)
		self.points2:setSize(0)
	end
	self.frames:setSize(0)
	return cost, numFrames
end

function CompareChain2D:compare()

	local skel=self.skel

	local updatePoints=self.updatePoints
	local baseBoneIndex
	if self.baseBoneName ~= nil then
		baseBoneIndex = self.skel:getTreeIndexByName(self.baseBoneName)
	end
	updatePoints(skel,self.chain1, self.points1, self.markers1.markers, baseBoneIndex)
	updatePoints(skel,self.chain2, self.points2, self.markers2.markers, baseBoneIndex)

	local dist=self.metric:calcDistance(self.points1, self.points2)
	if self.metric.errorOccurred==true then
		--      debug.debug()
	end
	return dist
end


function CompareChain2D:addQueue(mrdMotion, synthesis)
	local a=vectorn()
	local b=vectorn()
	local skel=self.skel

	local updatePoints=self.updatePoints
	local baseBoneIndex
	if self.baseBoneName ~= nil then
		baseBoneIndex = self.skel:getTreeIndexByName(self.baseBoneName)
	end
	updatePoints(skel,self.chain1, a, self.markers1, baseBoneIndex)
	updatePoints(skel,self.chain2, b, self.markers2, baseBoneIndex)

	self.points1:concat(a)
	self.points2:concat(b)
	self.mrdMotion=mrdMotion
	if mrdMotion then
		self.frames:pushBack(mrdMotion:numFrames())
	end
	--CompareChainHead.addQueue(self,mrdMotion,synthesis)
end



CompareChain2DRot=LUAclass()

function CompareChain2DRot:__init(vrmlLoader, boneForwardKinematics1, boneForwardKinematics2, syn)
	self.skel=vrmlLoader
	self.chain1=boneForwardKinematics1
	self.chain2=boneForwardKinematics2
	self.synthesis=syn
	self.simulator=syn.simulator

	self.numFrames=0
	self.comvel=vector3N()
	self.com=vector3N()
	self.head=vector3N()
	self.footL=vector3N()
	self.footR=vector3N()

	self.prevcomvel=vector3N()
	self.prevcom=vector3N()
	self.prevhead=vector3N()
	self.prevNumFrames=0
end


function CompareChain2DRot:drawMatching(objectList)
end

function CompareChain2DRot:compareQueue(objectList, seg)
	--   print(seg.grpName, seg.name, seg.nextLast-seg.first, self.numFrames)
	local cost1=math.SQR(seg.nextLast-seg.first+1-self.numFrames)


	local cost2=0 -- measures self similarity
	local cost3=0 -- measures 


	assert(self.com:size()==self.numFrames)

	local samplePrec=20.0
	for i=0, samplePrec do -- sampling interval
		local frac=i/samplePrec
		local frac1=frac*(self.com:size()-1)
		local fracd=frac*(seg.nextLast-seg.first)
		local com=self.com:sampleRow(frac1)
		local head=self.head:sampleRow(frac1)
		local comvel=self.comvel:sampleRow(frac1)

		local dcom=seg.lcom:sampleRow(fracd)
		local dcomvel=seg.lcomvel:sampleRow(fracd)
		local dhead=seg.lhead:sampleRow(fracd)

		if self.synthesis.desiredTurningSpeed==0 then
			cost2=cost2+math.SQR((com-dcom):length())
			cost2=cost2+math.SQR((comvel-dcomvel):length())
			cost2=cost2+math.SQR((head-dhead):length())
		else
			cost2=cost2+math.SQR(comvel.x)
			cost2=cost2+math.SQR(comvel.z-1.8)
			cost2=cost2+math.SQR(head.x)
			cost2=cost2+math.SQR(head.z)
		end

		if self.prevNumFrames>0 then
			local frac2=frac*(self.prevNumFrames-1)
			local prevcom=self.prevcom:sampleRow(frac2)
			local prevhead=self.prevhead:sampleRow(frac2)
			local prevcomvel=self.prevcomvel:sampleRow(frac2)

			cost3=cost3+math.SQR((prevcom-com):length())
			cost3=cost3+math.SQR((prevcomvel-comvel):length())
			cost3=cost3+math.SQR((prevhead-head):length())
		end
	end
	cost2=cost2/samplePrec
	cost3=cost3/samplePrec

	--  print(cost1/2500, cost2, cost3)
	local numFrames=self.numFrames
	self.prevNumFrames=self.numFrames
	self.numFrames=0

	self.prevcomvel, self.comvel=   self.comvel, self.prevcomvel
	self.prevcom, self.com=   self.com, self.prevcom
	self.prevhead, self.head=   self.head, self.prevhead
	return cost1/2500+cost2+cost3, numFrames
end

function CompareChain2DRot:compare()
	assert(false)
end


function CompareChain2DRot:addQueue()


	self.numFrames=self.numFrames+1
	self.com:resize(self.numFrames)
	self.comvel:resize(self.numFrames)
	self.head:resize(self.numFrames)   
	self.footL:resize(self.numFrames)   
	self.footR:resize(self.numFrames)   

	local head=self.skel:getBoneByVoca(MotionLoader.HEAD)
	local footL=self.skel:getBoneByVoca(MotionLoader.LEFTANKLE)
	local footR=self.skel:getBoneByVoca(MotionLoader.RIGHTANKLE)
	local root_transf=self.chain1:globalFrame(1):copy()

	local rotY=self.synthesis:calcRotY(self.numFrames-1)
	local invRotY=rotY:inverse()
	root_transf.rotation:assign(rotY)
	self.com(self.numFrames-1):assign(root_transf:toLocalPos(self.simulator:calculateCOM(0)))
	self.comvel(self.numFrames-1):assign(root_transf:toLocalDir(self.simulator:calculateCOMvel(0)))
	self.head(self.numFrames-1):assign(root_transf:toLocalPos(self.chain1:globalFrame(head):toGlobalPos(head:localCOM())))
	self.footL(self.numFrames-1):assign(root_transf:toLocalPos(self.chain1:globalFrame(footL):toGlobalPos(footL:localCOM())))
	self.footR(self.numFrames-1):assign(root_transf:toLocalPos(self.chain1:globalFrame(footR):toGlobalPos(footR:localCOM())))

	--   print(self.com(self.numFrames-1), self.comvel(self.numFrames-1), self.head(self.numFrames-1))
end
do
	QuaterMetric=LUAclass()
	function QuaterMetric:__init()
	end
	function QuaterMetric:calcDistance(p1,p2,weights)
		assert(p1:size()==p2:size())
		assert(p1:size()==weights:size())
		local dist=0
		for i=0,p1:size()-1 do
			dist=dist+math.pow(p1(i):distance(p2(i)),2)*weights(i)
		end
		return dist
	end
	CompareChain2Dori=LUAclass()

	function CompareChain2Dori:__init(vrmlLoader, boneForwardKinematics1, boneForwardKinematics2, markerFile)
		self.skel=vrmlLoader
		self.chain1=boneForwardKinematics1
		self.chain2=boneForwardKinematics2
		self.points1=quaterN()
		self.points2=quaterN()
		self.weights=vectorn()
		self.frames=vectorn()
		self.metric=QuaterMetric()
		self.weight=vectorn()
		self.weight:setSize(self.skel:numBone()-1)
		CompareChainHead.__init(self, vrmlLoader)
		local function calcArticulatedMass(bone)
			local mass=0
			if bone==nil then return mass end
			mass=mass+bone:mass()
			local b=bone:childHead()
			while b do
				mass=mass+calcArticulatedMass(MainLib.VRMLloader.upcast(b))
				b=b:sibling()
			end
			return mass
		end
		for i=1,self.skel:numBone()-1 do
			self.weight:set(i-1, calcArticulatedMass(self.skel:VRMLbone(i)))
		end
	end

	function CompareChain2Dori:drawMatching(objectList)
	end

	function CompareChain2Dori:compareQueue(objectList, seg, objCost,flush)
		RE.output("points1 size", tostring(self.points1:size())..","
		..tostring(self.points2:size())..tostring(self.points1:range(0,math.min(10,self.points1:size()))))

		if self.points1:size()<2  then
			dbg.console()
			return 0,0
		end

		-- dist<= sum of the square distances between all corresponding points

		local penalty=0
		local skipCalcDist=false
		if objCost then
			local cost,_skip=objCost(self,seg,dist)
			--print('cs',cost,_skip)
			penalty=penalty+cost

			if _skip~=nil and not flush then
				skipCalcDist=_skip
			end
		end
		local cost=penalty
		--	local cost=0
		local numFrames=0
		if not skipCalcDist then
			if true then
				-- align
				local qs1=self.points1(0)
				local qs2=self.points2(0)
				local nb=self.skel:numBone()-1
				local qe1=self.points1(self.points1:size()-nb)
				local qe2=self.points1(self.points1:size()-nb)
				local d1=quater() local d2=quater()
				d1:difference(qs1,qs2)
				d2:difference(d1:rotationY()*qe1,qe2)
				local adjust_half=quater()
				adjust_half:interpolate(0.5, quater(1,0,0,0), d2:rotationY())
				for i=0, self.points1:size()-1 do
					self.points1(i):leftMult(adjust_half*d1:rotationY())
				end
			end

			local dist=self.metric:calcDistance(self.points1, self.points2, self.weights)
			-- if g_cdbg then
			-- 	local cdbg=g_cdbg
			-- 	assert(cdbg)
			-- 	array.pushBack(cdbg, {"points", cost, dist,self.points1:copy(), self.points2:copy()})
			-- end

			if self.metric.errorOccurred==true then
				--      debug.debug()
			end

			numFrames=self.points1:size()/3/(self.skel:numBone()-1)
			--   print(penalty*0.01, dist)
			cost=cost+dist

			local cost_head=CompareChainHead.compareQueue(self)
			print(dist, cost, cost_head)
			cost=cost+cost_head

			if objectList~=nil then
				self:drawMatching(objectList)
			end
			assert(self.skel:numBone()-1==self.weight:size())
			--   dist=dist*numFrames
			self.points1:setSize(0)
			self.points2:setSize(0)
			self.weights:setSize(0)
		end
		self.frames:setSize(0)
		return cost*1e-7, numFrames -- 1e-7 is for matching the output magnitude to the CompareChain2D class.
	end

	function CompareChain2Dori.updatePoints(skel,chain, points)
		points:setSize(skel:numBone()-1)
		for i=1,skel:numBone()-1 do
			--points(i-1):assign( chain:localFrame(i).rotation)
			points(i-1):assign( chain:globalFrame(i).rotation)
		end
	end
	function CompareChain2Dori:compare()

		local skel=self.skel

		local updatePoints=self.updatePoints
		updatePoints(skel,self.chain1, self.points1, self.markers1.markers)
		updatePoints(skel,self.chain2, self.points2, self.markers2.markers)

		local dist=self.metric:calcDistance(self.points1, self.points2, self.weight)
		if self.metric.errorOccurred==true then
			--      debug.debug()
		end
		return dist
	end


	function CompareChain2Dori:addQueue(mrdMotion, synthesis)
		local simulator=synthesis.simulator
		local a=quaterN()
		local b=quaterN()
		local skel=self.skel

		local updatePoints=self.updatePoints
		updatePoints(skel,self.chain1, a)
		updatePoints(skel,self.chain2, b)

		self.points1:concat(a)
		self.points2:concat(b)

		CompareChainHead.addQueue(self,mrdMotion,synthesis)
		self.weights:concat(self.weight)
		assert(a:size()==self.weight:size())
		self.mrdMotion=mrdMotion
		if mrdMotion then
			self.frames:pushBack(mrdMotion:numFrames())
		end
	end
end
