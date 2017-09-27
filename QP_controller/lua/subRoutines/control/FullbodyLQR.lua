require('RigidBodyWin/subRoutines/control/LQR')
function DOFindexMap(skel,unactuatedBoneIndex)
	local dofInfo=skel.dofInfo
	local numDOF=0
	for i=1, dofInfo:skeleton():numBone()-1 do
		if i~=unactuatedBoneIndex then
			if dofInfo:hasQuaternion(i) then
				numDOF=numDOF+dofInfo:numDOF(i)-1
			else
				numDOF=numDOF+dofInfo:numDOF(i)
			end
		end
	end

	local dofIndex=vectorn(numDOF)
	local cdof=0
	for i=1, dofInfo:skeleton():numBone()-1 do
		if i~=unactuatedBoneIndex then
			if dofInfo:hasQuaternion(i) then
				local sR=dofInfo:startR(i)
				for j=dofInfo:startT(i), sR-1 do
					dofIndex:set(cdof,j)
					cdof=cdof+1
				end
				dofIndex:set(cdof,sR+1)
				dofIndex:set(cdof+1,sR+2)
				dofIndex:set(cdof+2,sR+3)
				cdof=cdof+3
			else
				for j=dofInfo:startT(i), dofInfo:endR(i)-1 do
					dofIndex:set(cdof,j)
					cdof=cdof+1
				end
			end
		end
	end
	assert (cdof==numDOF)
	return dofIndex
end

FullbodyLQR=LUAclass()

function FullbodyLQR:__init(filename, g, dt, q, qd)
	
	self.dt=dt
	self.q=q
	self.qd=qd
	if type(filename)=="string" then
		self.loader=VRMLloader(filename)
	else
		self.loader=filename
	end
	self.loader:checkMass()


	self.theta=vectorn()

	self.loader:getPoseDOF(self.theta)

	self.theta_d=self.theta:copy()

	self.unactuatedBoneIndex=-1
	if self.theta:size()==10 then
		self.theta:setQuater(3, quater(math.rad(10),vector3(0,0,1)) )
	elseif self.theta:size()==4 then
		self.theta:set(3, math.rad(50))
		self.theta:set(0, 1)
		self.unactuatedBoneIndex=2
	elseif self.theta:size()==6 then
		self.theta:set(2, math.rad(50))
		self.theta:set(3, math.rad(50))
		self.theta:set(0, 1)
	elseif self.theta:size()==8 then
		self.theta:set(2, math.rad(50))
		self.theta:set(3, math.rad(50))
		self.theta:set(4, math.rad(50))
		self.theta:set(0, 1)
	else
		self.unactuatedBoneIndex=1
	end


	self.dtheta=vectorn(self.theta:size())
	self.dtheta:setAllValue(0)
	self.dtheta_d=self.dtheta:copy()
	self.testVelControl=false
	if self.testVelControl then
		self.dtheta_d:set(1, 1)
	end

	self.simulator=DynamicsSimulator_gmbs_penalty() 
	self.simulator:registerCharacter(self.loader)
	self.simulator:init(self.dt, DynamicsSimulator.EULER) 
	self.simulator:setLinkData(0,DynamicsSimulator.JOINT_VALUE, self.theta) 
	self.simulator:setLinkData(0, DynamicsSimulator.JOINT_VELOCITY, self.dtheta)
	self.simulator:setGVector(vector3(0,g,0)) 
	self.simulator:initSimulation()

	self.IDsolver=DynamicsSimulator_gmbs_penalty() 
	--self.IDsolver=DynamicsSimulator_VP_penalty() 
	self.IDsolver:registerCharacter(self.loader)
	self.IDsolver:init(self.dt, DynamicsSimulator.EULER) 
	self.IDsolver:setLinkData(0,DynamicsSimulator.JOINT_VALUE, self.theta) 
	self.IDsolver:setLinkData(0, DynamicsSimulator.JOINT_VELOCITY, self.dtheta)
	self.IDsolver:setGVector(vector3(0,g,0)) 
	self.IDsolver:initSimulation()
	self.controlforce=vectorn(self.theta:size())
	self.Y=matrixn()
	self.numFrame=0
	self.desiredPos=vector3()
	self.actuatedJoints=boolN(self.loader:numBone())
	self.actuatedJoints:setAllValue(true)
	if self.unactuatedBoneIndex~=-1 then
		self.actuatedJoints:set(self.unactuatedBoneIndex, false)
	end
	self.dofIndexMap=DOFindexMap(self.loader, unactuatedBoneIndex)

	local ndof=self.dofIndexMap:size()
	local mzero=CT.zeros(ndof,ndof)
	local mone=CT.eye(ndof, ndof)
	local A=CT.block(2,2, mzero, mone, mzero, mzero)
	local B=CT.block(2,1, mzero, mone)
	local Q=CT.eye(ndof*2, ndof*2)
	Q:diag():assign((CT.ones(ndof)*self.q)..(CT.ones(ndof)*self.qd))
	local R=CT.eye(ndof)
	if self.testVelControl then
		R:set(1,1,0)
	end
	local K=matrixn()
	LQR(K, A,B,Q,R) -- cpp implementation
	self.K=K
	self:calcMassMatrix()
end

function FullbodyLQR:extract(v1,v2)
	local idof=self.dofIndexMap
	for i=0,idof:size()-1 do
		v1:set(i, v2(idof(i)))
	end
end
function FullbodyLQR:insert(v1, v2)
	local idof=self.dofIndexMap
	for i=0,idof:size()-1 do
		v2:set(idof(i), v1(i))
	end
end
function FullbodyLQR:Extract(v2)
	local idof=self.dofIndexMap
	local v1=vectorn(idof:size())
	for i=0,idof:size()-1 do
		v1:set(i, v2(idof(i)))
	end
	return v1
end

function FullbodyLQR:calcMassMatrix()
	local ddq=vectorn(self.theta:size())
	local cf=vectorn(self.theta:size())
	-- calculate C
	local ndof=self.dofIndexMap:size()

	local idof=self.dofIndexMap
	self.C=matrixn(ndof,1)
	ddq:setAllValue(0)
	local function ID()
		--self.IDsolver:inverseDynamics(self.theta, self.dtheta, ddq, cf)
		self.IDsolver:hybridDynamics2(self.actuatedJoints:bit(), self.theta, self.dtheta, ddq, self.simulator:queryContactAll(), cf)

	end
	ID()
	self:extract(self.C:column(0),cf)

	self.M=matrixn(ndof, ndof)

	for i=0, ndof-1 do
		ddq:setAllValue(0)
		ddq:set(idof(i),1)
		ID()
		self:extract(self.M:column(i), cf)
		self.M:column(i):rsub(self.C:column(0))
	end
end

function FullbodyLQR:LQRplanning()
	-- input: theta, theta_d, dtheta, dtheta_d
	-- output: controlforce
	-- lqr planning
	local x=vectorn()

	local K=self.K
	MotionDOF.diffPose(self.loader.dofInfo, 1, x,self.theta, self.theta_d )
	local xd=self.dtheta_d-self.dtheta
	local theta=self:Extract(x)..self:Extract(xd)
	local u=K*theta:column()

	-- u=inv(M)(tau-C)
	local tau=self.M*u+self.C

	self.controlforce:zero()
	self:insert(tau:column(0), self.controlforce)
	--self.controlforce:set(2,0)
	--self.controlforce:set(3,0)
	--self.controlforce:set(4,0)
	--print(self.theta)
end
function FullbodyLQR:oneStep()
	print('onestep')
	self.controlforce:zero()
	if true then
		self:calcMassMatrix()
		self:LQRplanning()
	end
	self.simulator:setLinkData(0, DynamicsSimulator.JOINT_TORQUE, self.controlforce)
	self.simulator:stepSimulation()
	self.simulator:getLinkData(0, DynamicsSimulator.JOINT_VALUE, self.theta)
	self.simulator:getLinkData(0, DynamicsSimulator.JOINT_VELOCITY, self.dtheta)
end

function FullbodyLQR:setDesiredVelocity()
end
function FullbodyLQR:draw()
	if self.skin==nil then
		self.skin=RE.createVRMLskin(self.loader, false)
		self.skin:scale(100,100,100)
	end
	self.skin:setPose(self.simulator,0)
end
