require("subRoutines/gmbs_addon")
COPcontroller=LUAclass()
--class 'COPcontroller'


function COPcontroller:__init(vrml_loader)
	self.skel=vrml_loader

end

function COPcontroller._calcDesiredComAcc(XG,dotXG,dampingCoef,pgainCoef, desiredCOP, ref_yg)
	local ref_ddot_yg=dampingCoef*(-1*dotXG.y)
	if ref_yg then
		ref_ddot_yg=ref_ddot_yg+pgainCoef*(ref_yg-XG.y)
	end
	local g=9.8

	local w=math.sqrt((math.max(ref_ddot_yg+g,0))/(XG.y-desiredCOP.y))

	local ddotXG=vector3()
	ddotXG.x=w*w*(XG.x-desiredCOP.x)
	ddotXG.z=w*w*(XG.z-desiredCOP.z)
	ddotXG.y=ref_ddot_yg

	return ddotXG
end

function COPcontroller:calcDesiredComAcc(simulator, desiredCOP, ref_yg)
	--	-- Sugihara02 : z axis is the vertical axis
	--Sugihara02
	--ddot(xg)=w^2(xg-xz)
	--ddot(yg)=w^2(yg-yz)
	--Where
	--
	--w=sqrt((ref_ddot(zg)+g)/(zg-zz)),
	--Ref_ddot(zg)=Kz(ref_vz-dot(zg))         <- heuristic to determine vertical acc
	--
	--Sugihara03
	--Ref_ddot(zg)=fz/m-g,
	--Fz=m(K(ref_zg-zg)+Kd(ref_dot(zg)-dot(zg))+g)
	--
	--So basically equivalent except the k term.
	--	-- 

	-- y axis is the vertical axis in my case.
	local XG=simulator:calculateCOM(0)
	local dotXG=simulator:calculateCOMvel(0)
	local dampingCoef=useCase._COPcontroller_dampingCoef 
	local pgainCoef=useCase.COPcontroller_kCoef or 100
	--print(dampingCoef,pgainCoef,'coef')
	local ref_ddot_yg=dampingCoef*(-1*dotXG.y)
	if ref_yg then
		ref_ddot_yg=ref_ddot_yg+pgainCoef*(ref_yg-XG.y)
	end
	local g=9.8

	local w=math.sqrt((math.max(ref_ddot_yg+g,0))/(XG.y-desiredCOP.y))

	local ddotXG=vector3()
	ddotXG.x=w*w*(XG.x-desiredCOP.x)
	ddotXG.z=w*w*(XG.z-desiredCOP.z)
	ddotXG.y=ref_ddot_yg

	return ddotXG
end


-- COP controller. Taesoo's implementation. A novel hybrid between Yamane's IROS 09 paper and Macchieto's SIGGRAPH 09 paper.
--class 'HDcontroller'(COPcontroller)
HDcontroller=LUAclass(COPcontroller)

function HDcontroller:__init(vrml_loader, timestep )
	COPcontroller.__init(self, vrml_loader)
	self.skel_lfoot=VRMLloaderView(vrml_loader, vrml_loader:getBoneByVoca(MotionLoader.LEFTANKLE), vector3(0,0,0))
	MotionLoader.setVoca(self.skel_lfoot, model.bones)
	self.skel_lfoot_rfootIndex=self.skel_lfoot:getBoneByVoca(MotionLoader.RIGHTANKLE):treeIndex()
	self.simulator_lfoot=DynamicsSimulator_gmbs_penalty()
	self.simulator_lfoot:registerCharacter(self.skel_lfoot)
	self.simulator_lfoot:init(timestep, integrator)
	self.simulator_lfoot:setGVector(vector3(0,9.8,0))
	self.simulator_lfoot:initSimulation()
	self.controlforce=vectorn()
	self.idservo=IDservo(vrml_loader.dofInfo)
	self.idservo:initIDservo(nil,nil, nil, nil)
end

function HDcontroller:_generateTorque(simulator, frame, desiredCOP, locosyn)

	local ddotXG
	
	if false then
		ddotXG=self:calcDesiredComAcc(simulator, desiredCOP)
	else
		-- differential dynamics idea: 
		local currentCOP=locosyn.pendulum:calcCartPos()
		local acc1=self:calcDesiredComAcc(simulator, desiredCOP)
		local acc2=self:calcDesiredComAcc(simulator, currentCOP)
		ddotXG=acc1-acc2
--		ddotXG.y=acc1.y-correction.y
	end
	local state=vectorn()
	local dstate=vectorn()
	local statel=vectorn()
	local dstatel=vectorn()

	simulator:getLinkData(0, DynamicsSimulator.JOINT_VALUE, state)
	simulator:getLinkData(0, DynamicsSimulator.JOINT_VELOCITY, dstate)
	self.simulator_lfoot:convertStateFromSource(simulator, self.skel_lfoot,state, dstate, statel, dstatel)

	local ljacobian=matrixn()
	local ldotjacobian=matrixn()
	local lconJacobian=matrixn() -- jacobian of the right foot
	local lconDotJacobian=matrixn() -- dot jacobian of the right foot

	self.simulator_lfoot:updateJacobians(0)
	self.simulator_lfoot:calcCOMjacobian(0, ljacobian)
--	self.simulator_lfoot:calcCOMdotJacobian(0, ldotjacobian)
	self.simulator_lfoot:calcJacobian(0, self.skel_lfoot_rfootIndex, lconJacobian)
	--self.simulator_lfoot:calcDotJacobian(0, self.skel_lfoot_rfootIndex, lconDotJacobian)

	self.idservo:calcDesiredAcceleration(simulator, frame, state, dstate)
	local desiredAcc=self.idservo.desiredacceleration
	local sp=vector3(0,0.9,0)*100
	dbg.draw('Line', sp, sp+ddotXG*80*5, "COM vf")

	local desiredAccL=vectorn() -- desired acceleration represented in the left foot rooted skeleton.
	self.skel_lfoot:convertSourceDOFexceptRoot(desiredAcc, desiredAccL)
	
	if false then -- use QP 
		-- solve for the new desired acceleration
		local qpSolver=CT.QP(desiredAccL:size()-7, 0) -- COM and foot acceleration constraints

		local eVar=desiredAccL:size()
		for i=7, eVar-1 do
			qpSolver:add(1, i-7, desiredAccL(i)*-1)
		end


		if false then
			qpSolver:conSystem(ljacobian:range(0,3, 7, eVar), CT.vec(ddotXG.x, ddotXG.y, ddotXG.z))
			qpSolver:conSystem(lconJacobian:range(0,6, 7, eVar), -(lconDotJacobian*dstatel:column()):column(0))
		elseif true then
			qpSolver:addSystem(100,ljacobian:range(0,3, 7, eVar), CT.vec(ddotXG.x, ddotXG.y, ddotXG.z))
			qpSolver:addSystem(100,lconJacobian:range(0,6, 7, eVar), -(lconDotJacobian*dstatel:column()):column(0))
		else
			local nullProjL=math.nullspaceProjector(lconJacobian)
			ddotXG=ddotXG*0.01
			qpSolver:addSystem(100,(ljacobian*nullProjL):range(0,3, 7, eVar), CT.vec(ddotXG.x, ddotXG.y, ddotXG.z))
			--qpSolver:conSystem((ljacobian*nullProjL):range(0,3, 7, eVar), CT.vec(ddotXG.x, ddotXG.y, ddotXG.z))
		end

		local newDesiredAccL=CT.vec(0,0,0,0,0,0,0)..qpSolver:solve()
		--	print(desiredAccL-newDesiredAccL)

		-- desired acceleration represented in the pelvis rooted skeleton.
		self.skel_lfoot:convertDOFexceptRoot(newDesiredAccL, desiredAcc)
		-- solve ID servo
		self.idservo:_computeHDtorque(simulator, state,dstate)
		self.controlforce=self.idservo.controlforce
	else
		local nullProjL=math.nullspaceProjector(lconJacobian)
		local desiredAccL=(nullProjL*ljacobian:Transpose()*CT.mat(3,1,ddotXG.x, ddotXG.y, ddotXG.z)):column(0)
		local newDesiredAcc=vectorn()
		self.skel_lfoot:convertDOFexceptRoot(desiredAccL, newDesiredAcc)
		desiredAcc:radd(newDesiredAcc)
		-- solve ID servo
		self.idservo:_computeHDtorque(simulator, state,dstate, 9.8*80)
		self.controlforce=self.idservo.controlforce:copy()
--		desiredAcc:assign(newDesiredAcc) -- add COM acceleration
--		self.idservo:_computeHDtorque(simulator, state,dstate, 9.8*800)
--		self.controlforce:radd(self.idservo.controlforce)
--		self.controlforce:rmult(0.5)
	end

end
