
local DynamicsSimulator_gmbs_penalty=Physics.DynamicsSimulator_gmbs_penalty
local DynamicsSimulator=Physics.DynamicsSimulator

function DynamicsSimulator_gmbs_penalty:_convertJacobian(DpDq, jacobian, ibone, bFreeRootJoint)
	if self.coordIndex==nil then
		local coordIndex_=matrixn()
		self:test("coordIndex", coordIndex_)
		self.coordIndex=coordIndex_:row(0):copy()
		self.dofIndex=coordIndex_:row(1):copy()
	end

	local coordIndex=self.coordIndex
	local dofIndex=self.dofIndex

	jacobian:setSize(DpDq:rows(), self:skeleton(0).dofInfo:numDOF())
	assert(self:skeleton(0).dofInfo:numDOF()==coordIndex:size())
	
	jacobian:setAllValue(0)
	--[[
	for i=0, coordIndex:size()-1 do
		local idx=coordIndex(i) -- i: dofIndex, idx: columnIndex
		if idx~=-1 then
			jacobian:column(i):assign(DpDq:column(idx))
		end
	end
	]]--

	ibone=ibone or 0
	local function assumeFreeRootJoint()

		-- now root joint has to be specially handled
		-- In gmbs world velocity dq = RS * dtheta
		-- where dtheta[0:3] is euler angle vel in body coord
		--       dtheta[3:6] is linear vel in body coord
		-- in our case dq[0:3] : linear displacement in world coord
		--             dq[4:7] : angular displacement in world coord

		-- given x = J* dtheta, we want to find J_new such that x=J_new* dq
		-- -> J*dtheta=J*invS*invR*dq
		-- -> J_new=J*invS*invR

		local invS=matrixn()
		local invR=matrixn()
		self:test("getRoot_invS", invS)
		self:test("getRoot_invR", invR)
		local invSinvR=matrixn(6,6)
		-- invSinvR= invS*(invR 0)
		--                (0    invR)
		invSinvR:sub(0,3,0,3):mult(invS:sub(0,3,0,3), invR)
		invSinvR:sub(0,3,3,6):mult(invS:sub(0,3,3,6), invR)
		invSinvR:sub(3,6,0,3):mult(invS:sub(3,6,0,3), invR)
		invSinvR:sub(3,6,3,6):mult(invS:sub(3,6,3,6), invR)

		local J_new=DpDq:sub(0,0,0,6)*invSinvR

		jacobian:sub(0,0,0,3):assign(J_new:sub(0,0,3,6))
		jacobian:sub(0,0,4,7):assign(J_new:sub(0,0,0,3))
	end
	if ibone==0 then -- in case of COM jacobian or momentum jacobian, all DOFs are associated with the jacobian matrix.
		assert(DpDq:cols()==coordIndex:size()-1)
		for i=0, dofIndex:size()-1 do
			local idx=dofIndex(i) -- i: columnIndex, idx: dofIndex
			if idx~=-1 then
				jacobian:column(idx):assign(DpDq:column(i))
			end
		end
		if bFreeJoint~=false then 
			assumeFreeRootJoint() -- by default, COM jacobian include root joint terms. You can always discard them later.
		end
	else
		-- otherwise, jacobian is actually is not a full-matrix. e.g. upperbody columns are all 0 when adjusting the foot joint. 
		-- gmbs doesn't output those zeros, but this function always output full-matrix even when it is sparse.
		assert(ibone~=nil)
		local coordIndex_=CT.mat(1,1, ibone)
		self:test("getJacobian_coordIndex", coordIndex_)

		local coordIndex=coordIndex_:row(0)
		for i=0, coordIndex:size()-1 do
			local idx=dofIndex(coordIndex(i)) -- i: columnIndex, idx: dofIndex
			if idx~=-1 then
				jacobian:column(idx):assign(DpDq:column(i))
			end
		end

		if bFreeJoint then
			assumeFreeRootJoint() -- by default(bFreeJoint==nil), assumes Fixed Root Joint. 
		end
	end

end

-- prerequisties for the following two functions
function DynamicsSimulator_gmbs_penalty:updateJacobians(chara)
--	dbg.console()
	local DpDq=matrixn()
	self:test("UpdateJacobian", DpDq)
end

function DynamicsSimulator_gmbs_penalty:calcCOMjacobian(chara, jacobian)
	local DpDq=matrixn()
	self:test("PositionCOM2", DpDq)
	self:_convertJacobian(DpDq, jacobian)
end

function DynamicsSimulator_gmbs_penalty:calcMomentumGlobalJacobian(chara, jacobian)
	local DpDq=matrixn()
	self:test("MomentumGlobal", DpDq)
	self:_convertJacobian(DpDq, jacobian)
end
function DynamicsSimulator_gmbs_penalty:calcMomentumCOMjacobian(chara, jacobian)
	local DpDq=matrixn()
	self:test("MomentumCOM", DpDq)
	self:_convertJacobian(DpDq, jacobian)
end
function DynamicsSimulator_gmbs_penalty:calcJacobian(chara, ibone, jacobian, localpos, bFreeRootJoint)
	local DpDq=matrixn(1,1)
	DpDq:set(0,0, ibone)
	if localpos==nil then
	--if true then
		self:test("getJacobian", DpDq)
	else
		DpDq:resize(1,4)
		DpDq:set(0,1,localpos.x)
		DpDq:set(0,2,localpos.y)
		DpDq:set(0,3,localpos.z)
		self:test("getJacobianLpos", DpDq)
	end
	self:_convertJacobian(DpDq, jacobian, ibone, bFreeRootJoint)
end

function DynamicsSimulator_gmbs_penalty:calcDotJacobian(chara, ibone, jacobian)
	local DpDq=matrixn(1,1)
	DpDq:set(0,0, ibone)
	self:test("getDotJacobian", DpDq)
	self:_convertJacobian(DpDq, jacobian, ibone)
	assert(false) -- this implementation is completely wrong. read mywiki
end

function DynamicsSimulator:convertStateFromSource(srcSim, tgtSkel, state, dstate, state2, dstate2)
	tgtSkel:convertSourceDOFexceptRoot(state, state2)
	tgtSkel:convertSourceDOFexceptRoot(dstate, dstate2)
	MotionDOF.setRootTransformation(state2, srcSim:getWorldState(0):globalFrame(tgtSkel.newRootBone))
	dstate2:setVec3(0, srcSim:getWorldVelocity(0, tgtSkel.newRootBone, vector3(0,0,0)))
	dstate2:set(3,1)
	dstate2:setVec3(4, srcSim:getWorldAngVel(0, tgtSkel.newRootBone))
end

function DynamicsSimulator:integrateDState(state, dstate)
	state:range(7,state:size()):assign(state:range(7, state:size())+dstate:range(7, dstate:size()))
	-- root vel
	state:range(0,3):assign(state:range(0,3)+dstate:range(0,3))
	-- root ori
	local w=dstate:toVector3(4)
	q=state:toQuater(3)
	local dq=quater()
	dq:setRotation(w)
	q:leftMult(dq)-- note that infinisimal rotations are commutative so rightMult should work too in general
	state:setQuater(3, q)
end

defineDerived(DynamicsSimulator, {DynamicsSimulator_gmbs_penalty, DynamicsSimulator_AIST_penalty}, {"convertStateFromSource","integrateDState" })

function MainLib.VRMLloaderView:convertJacobianExceptRoot(jacobian, srcJacobian)
	srcJacobian:setSize(jacobian:rows(), self:getSourceSkel().dofInfo:numDOF())
	srcJacobian:sub(0,0,0,7):zero()
	for i=0, jacobian:rows()-1 do
		self:convertDOFexceptRoot(jacobian:row(i), srcJacobian:row(i))
	end
end
