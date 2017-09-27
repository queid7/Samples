require('subRoutines/ConvexHull2D')
require("module")
HessianQuadratic.add=QuadraticFunctionHardCon.add
drawContactForces =true

function checkLCP(x, CE, ce0, CI, ci0, LCP, lcp0,lcp_s,lcp_e)
	
	local eq
	if CE:rows()>0 then
		eq=CE*x:column()+ce0:column()
	else
		eq=CT.vec(0)
	end
	local iq=CI*x:column()+ci0:column()
	print('eq:', eq:minimum(), eq:maximum())
	print('iq:', iq:minimum(), iq:maximum())
	local lcp_v=vectorn(lcp_e-lcp_s)
	local lambda=x:range(lcp_s, lcp_e)
	local acc=LCP*lambda:column()+lcp0
	print('acc:', acc:minimum(), acc:maximum())
	print('lambda:', lambda:minimum(), lambda:maximum())
	for i=0, lcp_v:size()-1 do
		lcp_v:set(i, acc(i,0)*lambda(i))
	end
	local iMax=lcp_v:argMax()
	print('acc*lambda:', lcp_v:minimum(), lcp_v:maximum(), iMax, acc(iMax,0), lambda(iMax))
end
-- add objective function term weight*( x_i -value)^2
function HessianQuadratic:addD(weight, index, value)
	local sqw=math.sqrt(weight)
	local i=CT.ivec(index)
	local v=CT.vec(sqw,sqw*-1*value)
	self:addSquared(i,v)
end
function HessianQuadratic:addSquaredW(weight, i, v)
	self:addSquared(i, math.sqrt(weight)*v)
end
-- minimize weight*(M * x[si:ei) +b)^2
function HessianQuadratic:addV(weight, M, si, ei,b)
	local i=CT.colon(si,ei,1)
	assert(M:cols()==ei-si)
	local v=vectorn(M:cols()+1)
	local sqw=math.sqrt(weight)
	for j=0,M:rows()-1 do
		v:range(0,v:size()-1):assign(M:row(j))
		v:set(v:size()-1, b(j,0))
		v:rmult(sqw)
		self:addSquared(i,v)
	end
	--##dos if g_debugOneStep then --##dos g_debugOneStep:pushBack({"addv",self.R:copy()}) --##dos end 
end 

function HessianQuadratic:addVselective(weight, M, si, ei,b, index)
	local i=CT.colon(si,ei,1)
	assert(M:cols()==ei-si)
	local v=vectorn(M:cols()+1)
	local sqw=math.sqrt(weight)
	for j=0,index:size()-1 do
		v:range(0,v:size()-1):assign(M:row(index(j)))
		v:set(v:size()-1, b(index(j),0))
		v:rmult(sqw)
		self:addSquared(i,v)
	end
	--##dos if g_debugOneStep then --##dos g_debugOneStep:pushBack({"vsel",self.R:copy()}) --##dos end 
end 
function HessianQuadratic:addComplementory(weight,si,ei, M,q)
	-- seems to be buggy
	-- zT(Mz + q)
	-- where z is x[si:ei)
	
	-- xT(Nx + q)

	-- 0.5H=M -> H=2M
	self.H:range(si,ei,si,ei):radd(M*(2*weight))
	self.R:range(si,ei):rsub(q*weight)
end

function HessianQuadratic:addComplementory3(weight, n, si1,ei1, M_S2, S2q)
	-- seems to be buggy
	-- minimize (S1*x)'*(M*S2*x+S2*q) where S1, S2 is selection matrices
	-- = x'*S1'*M*S2*x+S1'S2*q 

	-- e.g. x=(x1, x2, x3)' -> S1= (0 1 0;0 0 1)
	local function selectionMatrix(n, si, ei)
		local S=matrixn(ei-si, n)
		S:setAllValue(0)
		S:sub(0,0,si,ei):identity()
		return S
	end

	-- M*   S2   *      x           = M* ddq
	-- M*[I 0 0] * [ddq;tau;lambda] = ddq
	 
	local S1=selectionMatrix(n, si1, ei1)
	local S1tM_S2=matrixn()
	local S1tS2q=matrixn()
	S1tM_S2:multAtB(S1,M_S2)
	S1tS2q:multAtB(S1, S2q)
	self.H:radd(S1tM_S2*(2*weight))
	self.R:radd(S1tS2q:column(0)*weight)
end
function Physics.ContactBasis:__tostring()
	return string.format("%d %d (%s) n:(%s) f:%d %d %d\n", self.ibody, self.ibone,tostring( self.globalpos), tostring(self.normal), self.frictionNormal:size(), self.globalIndex, self.globalFrictionIndex)
end
function Physics.Vec_ContactBasis:__tostring()
	local out=""
	for i=0, self:size()-1 do
		out=out..i..": "..tostring(self(i))
	end
	return out
end

-- PDServo class
--class 'QPservo'
QPservo=LUAclass()

function QPservo:setCoef(dofInfo,kp, kd, tgtVelScale, k_scale)
	kp:setSize(dofInfo:numDOF())
	kp:setAllValue(k_p)
	kd:setSize(dofInfo:numDOF())
	kd:setAllValue(k_d)
	tgtVelScale:setSize(dofInfo:numDOF())
	tgtVelScale:setAllValue(1)
	
	if self.excludeRoot then
		-- exclude root joint
		kp:range(0,7):setAllValue(0)
		kd:range(0,7):setAllValue(0)
	end
	
	--print("initQPservo:"..dofInfo:skeleton():bone(1):name())
	for i=2,dofInfo:skeleton():numBone()-1 do
		local bone=dofInfo:skeleton():bone(i)
		local vbone=bone:treeIndex()
		local nJoint=dofInfo:numDOF(vbone)
		--      print("initQPservo:"..bone:name())
		for j=0, nJoint-1 do
			
			local dofIndex=dofInfo:DOFindex(vbone,j)
			
			kp:set(dofIndex, k_p*k_scale.default[1])
			kd:set(dofIndex, k_d*k_scale.default[2])
			tgtVelScale:set(dofIndex, k_scale.default[3])

			if bone:voca()==MotionLoader.LEFTANKLE or bone:voca()==MotionLoader.RIGHTANKLE then
				if k_scale.ankle~=nil then
					kp:set(dofIndex, k_p*k_scale.ankle[1])
					kd:set(dofIndex, k_d*k_scale.ankle[2])
					tgtVelScale:set(dofIndex, k_scale.ankle[3])
				end
			elseif bone:voca()==MotionLoader.LEFTWRIST or bone:voca()==MotionLoader.RIGHTWRIST then
				if k_scale.wrist~=nil then
					kp:set(dofIndex, k_p*k_scale.wrist[1])
					kd:set(dofIndex, k_d*k_scale.wrist[2])
					tgtVelScale:set(dofIndex, k_scale.wrist[3])
				end
			elseif bone:voca()==MotionLoader.LEFTCOLLAR or bone:voca()==MotionLoader.RIGHTCOLLAR then
				if k_scale.collar~=nil then
					kp:set(dofIndex, k_p*k_scale.collar[1])
					kd:set(dofIndex, k_d*k_scale.collar[2])
					tgtVelScale:set(dofIndex, k_scale.collar[3])
				end
			elseif bone:voca()==MotionLoader.LEFTSHOULDER or bone:voca()==MotionLoader.RIGHTSHOULDER then
				if k_scale.shoulder~=nil then
					kp:set(dofIndex, k_p*k_scale.shoulder[1])
					kd:set(dofIndex, k_d*k_scale.shoulder[2])
					tgtVelScale:set(dofIndex, k_scale.shoulder[3])
				end
			elseif bone:voca()==MotionLoader.LEFTELBOW or bone:voca()==MotionLoader.RIGHTELBOW then
				if k_scale.elbow~=nil then
					kp:set(dofIndex, k_p*k_scale.elbow[1])
					kd:set(dofIndex, k_d*k_scale.elbow[2])
					tgtVelScale:set(dofIndex, k_scale.elbow[3])
				end
			elseif bone:voca()==MotionLoader.LEFTKNEE or bone:voca()==MotionLoader.RIGHTKNEE then
				if k_scale.knee~=nil then
					kp:set(dofIndex, k_p*k_scale.knee[1])
					kd:set(dofIndex, k_d*k_scale.knee[2])
					tgtVelScale:set(dofIndex, k_scale.knee[3])
				end
			elseif bone:voca()==MotionLoader.LEFTHIP or bone:voca()==MotionLoader.RIGHTHIP then
				if k_scale.hip~=nil then
					kp:set(dofIndex, k_p*k_scale.hip[1])
					kd:set(dofIndex, k_d*k_scale.hip[2])
					tgtVelScale:set(dofIndex, k_scale.hip[3])
				end
			elseif bone:voca()==MotionLoader.CHEST then
				if k_scale.chest~=nil then
					kp:set(dofIndex, k_p*k_scale.chest[1])
					kd:set(dofIndex, k_d*k_scale.chest[2])
					tgtVelScale:set(dofIndex, k_scale.chest[3])
				end
			elseif bone:voca()==MotionLoader.CHEST2 then
				if k_scale.chest2~=nil then
					kp:set(dofIndex, k_p*k_scale.chest2[1])
					kd:set(dofIndex, k_d*k_scale.chest2[2])
					tgtVelScale:set(dofIndex, k_scale.chest2[3])
				end
			elseif bone:voca()==MotionLoader.NECK then
				if k_scale.neck~=nil then
					kp:set(dofIndex, k_p*k_scale.neck[1])
					kd:set(dofIndex, k_d*k_scale.neck[2])
					tgtVelScale:set(dofIndex, k_scale.neck[3])
				end
			elseif bone:voca()==MotionLoader.HEAD then
				if k_scale.head~=nil then
					kp:set(dofIndex, k_p*k_scale.head[1])
					kd:set(dofIndex, k_d*k_scale.head[2])
					tgtVelScale:set(dofIndex, k_scale.head[3])
				end
			end
			if str_include(bone:name(), "toes") then
				local dofIndex=dofInfo:DOFindex(vbone,j)
				if k_scale.toes~=nil then
					kp:set(dofIndex, k_p*k_scale.toes[1])
					kd:set(dofIndex, k_d*k_scale.toes[2])
					tgtVelScale:set(dofIndex, k_scale.toes[3])
				end

			end

			if dofInfo:DOFtype(vbone, j)==MotionDOFinfo.SLIDE then
				local dofIndex=dofInfo:DOFindex(vbone,j)
				kp:set(dofIndex, k_p_slide)
				kd:set(dofIndex, k_d_slide)
				tgtVelScale:set(dofIndex, 0)
			end
		end
	end
end

function QPservo:updateCoef()
	local dofInfo=self.dofInfo

	k_p=1
	k_d=1
	k_p_slide=5
	k_d_slide=5

	-- self:setIDGain(dofInfo:skeleton(), self.kp_id, self.kd_id, k_p, k_d, k_p_slide or k_p*5, k_d_slide or k_d*5)
	self.weight=vectorn()
	local bigJointsAccCoef=useCase.bigJointsAccCoef or 1
	model.k_scale_id.hip={bigJointsAccCoef,bigJointsAccCoef,1}
	model.k_scale_id.chest={bigJointsAccCoef,bigJointsAccCoef,1}
	--model.k_scale_id.ankle={0.5,0.5,1}
	self:setCoef(dofInfo, self.kp_id, self.kd_id, self.weight, model.k_scale_id)

	self.weight2=vectorn()

	local temp=vectorn()

	local bigJointsTorqueCoef=useCase.bigJointsTorqueCoef or 1
	local k_torque_bound={
		default={1,1,1}, 
		hip={bigJointsTorqueCoef,1,1},
		chest={bigJointsTorqueCoef,1,1},
		--ankle={1/bigJointsTorqueCoef,1,1},
		--ankle={0.5,1,1},
	}

	self:setCoef(dofInfo, self.weight2,temp,temp, k_torque_bound)

	self.dofToLimb=vectorn()
	self.dofToLimbMap={L=0,R=1, LH=2, RH=3, O=4}
	self.dofToLimbInvMap={[0]='L',[1]='R',[2]='LH',[3]='RH',[4]='O'}
	do
		local dtl=self.dofToLimb
		local map=self.dofToLimbMap
		dtl:setSize(dofInfo:numDOF())
		dtl:setAllValue(map.O)
		for i=2,dofInfo:skeleton():numBone()-1 do
			local bone=dofInfo:skeleton():bone(i)
			local vbone=bone:treeIndex()
			local nJoint=dofInfo:numDOF(vbone)
			--      print("initQPservo:"..bone:name())

			local voca=bone:voca()
			if voca==MotionLoader.LEFTANKLE or 
				voca==MotionLoader.LEFTKNEE or
				voca==MotionLoader.LEFTHIP or
				bone:name()=='ltoes' then
				for j=0, nJoint-1 do
					dtl:set( dofInfo:DOFindex(vbone,j), map.L)
				end
			elseif voca==MotionLoader.RIGHTANKLE or 
				voca==MotionLoader.RIGHTKNEE or
				voca==MotionLoader.RIGHTHIP or
				bone:name()=='rtoes' then
				for j=0, nJoint-1 do
					dtl:set( dofInfo:DOFindex(vbone,j), map.R)
				end
			elseif voca==MotionLoader.LEFTSHOULDER or 
				voca==MotionLoader.LEFTELBOW or
				voca==MotionLoader.LEFTWRIST then
				for j=0, nJoint-1 do
					dtl:set( dofInfo:DOFindex(vbone,j), map.LH)
				end
			elseif voca==MotionLoader.RIGHTSHOULDER or 
				voca==MotionLoader.RIGHTELBOW or
				voca==MotionLoader.RIGHTWRIST then
				for j=0, nJoint-1 do
					dtl:set( dofInfo:DOFindex(vbone,j), map.RH)
				end
			end
		end
	end
end

function QPservo:computeContact(sim, swingFoot)
	local simulator=self.simulator
	self.contactHull=nil
	local cf=Physics.Vec_CFinfo()
	cf:assign(simulator:queryContactAll())

	local cstate=self.state
	self.contactHull=ConvexHull2D()
	local skel=self.dofInfo:skeleton()
	local lfoot=skel:getTreeIndexByVoca(MotionLoader.LEFTANKLE)
	local ltoes=skel:getTreeIndexByVoca(MotionLoader.LEFTTOES)
	local rfoot=skel:getTreeIndexByVoca(MotionLoader.RIGHTANKLE)
	local rtoes=skel:getTreeIndexByVoca(MotionLoader.RIGHTTOES)

	local lhand=skel:getTreeIndexByVoca(MotionLoader.LEFTWRIST)
	local rhand=skel:getTreeIndexByVoca(MotionLoader.RIGHTWRIST)
	local actualContact={L=false, R=false,LH=false, RH=false}

	for i=0, cf:size()-1 do
		if cf(i).chara==0 then
			local ci=cf(i)

			local ti=ci.bone:treeIndex()
			local isLfoot=false
			local isRfoot=false
			if (ti==lfoot or ti==ltoes) then 
				isLfoot=true 
				actualContact.L=true 
			end
			if (ti==rfoot or ti==rtoes) then 
				isRfoot=true 
				actualContact.R=true 
			end
			if ti==lhand then actualContact.LH=true end
			if ti==rhand then actualContact.RH=true end
			
			local frame=simulator:getWorldState(0):globalFrame(ci.bone)
			local globalPos=frame:toGlobalPos(ci.p)

			if not (swingFoot=="L" and actualContact.L ) and not (swingFoot=="R" and actualContact.R) then
				self.contactHull:addVector3(globalPos)
			end
		end
	end
	--print(simulator:getWorldState(0):globalFrame(lfoot).translation)
	--print(simulator:getNumContactLinkPairs())
	--printTable(actualContact)

	cstate.actualContact=actualContact
	if actualContact.L==false and actualContact.R==false 
	and actualContact.LH==false and actualContact.RH==false then
		cstate.flightPhase=true
	else
		cstate.flightPhase=false
	end

	-- keeps track of contact changes
	if not cstate.previousFlightPhase and cstate.flightPhase then
		cstate.supportPhaseElapsed=0
	elseif cstate.previousFlightPhase and not cstate.flightPhase then
		cstate.flightPhaseElapsed=0
	end

	if cstate.flightPhase then
		cstate.flightPhaseElapsed=cstate.flightPhaseElapsed+1
	else
		cstate.supportPhaseElapsed=cstate.supportPhaseElapsed+1
	end

	cstate.previousFlightPhase=cstate.flightPhase
end

function QPservo:__init(dofInfo, timestep,integrator, simulator)
	self.simulator=simulator
	self.dtinv=1/timestep
	self.invfricCoef=useCase.invFricCoef or 0.5
	-- settings
	self.useMLCPmatFromAIST=false
	self.excludeRoot=useCase.excludeRoot
	self.excludeRootFlight=useCase.excludeRootFlight or false
	self.excludeNormalBasis=false
	self.giOffset=0
	-- defalt
	self.state={previousFlightPhase=false, flightPhase=false, supportPhaseElapsed=100, flightPhaseElapsed=0, prevContact={}, contact={}, contactDuration={}, contactClass={}}
	self.numCLinks=simulator:getNumAllLinkPairs()
	local index=intvectorn()
	simulator:getContactLinkBoneIndex(0,index)

	--_checkpoints:pushBack(deepCopyTable({'clinks', self.numCLinks, index}))
	self.bi={}
	local bi=self.bi
	do 
		bi.L=simulator:skeleton(0):getTreeIndexByVoca(MotionLoader.LEFTANKLE)
		bi.R=simulator:skeleton(0):getTreeIndexByVoca(MotionLoader.RIGHTANKLE)
		bi.LH=simulator:skeleton(0):getTreeIndexByVoca(MotionLoader.LEFTWRIST)
		bi.RH=simulator:skeleton(0):getTreeIndexByVoca(MotionLoader.RIGHTWRIST)
		assert(bi.L~=-1)
	end

	do
		-- set footLoop jacobian tools
		local footLoop={}
		local skel=simulator:skeleton(0)
		footLoop.skel= MainLib.VRMLloaderView(skel,skel:getBoneByVoca(MotionLoader.LEFTANKLE), vector3(0,0,0))
		MotionLoader.setVoca(footLoop.skel, model.bones)
		footLoop.sim= Physics.DynamicsSimulator_gmbs(true)
		footLoop.sim:registerCharacter(footLoop.skel)
		footLoop.sim:init(timestep, Physics.DynamicsSimulator.EULER)
		footLoop.Rindex=footLoop.skel:getTreeIndexByVoca(MotionLoader.RIGHTANKLE)
		self.footLoop=footLoop
	end

	for i=1,self.numCLinks do
		self.state.prevContact[i]=false
		self.state.contact[i]=false
		self.state.contactDuration[i]=0

		for k,isL in ipairs({'L','R','LH','RH'}) do
			if index(i-1)==bi[isL] or index(i-1)==bi[isL]+1 then
				self.state.contactClass[i]=isL
			end
		end
		if self.state.contactClass[i]==nil then
			self.state.contactClass[i]='O'
		end
	end
	self.state.aggContactForce=vector3(0,0,0)

	--_checkpoints:pushBack(deepCopyTable({'state', state}))
	self.theta=vectorn()
	self.dtheta=vectorn()
	-- HD servo
	self.theta_d=vectorn() -- desired q
	self.dtheta_d=vectorn() -- desired dq
	self.ddtheta_d=vectorn() -- desired ddq

	-- PD servo
	--self.theta_d_pd=vectorn()

	self.desiredacceleration=vectorn()
	self.controlforce=vectorn()
	self.kp=vectorn()
	self.kd=vectorn()
	self.kp_id=vectorn()
	self.kd_id=vectorn()

	self.tgtVelScale=vectorn()
	self.mask_slide=vectorn()
	

	-- lleg+rleg+upperbody=all
	self.mask_lleg=vectorn() -- excluding sliding joints
	self.mask_rleg=vectorn() -- excluding sliding joints
	self.mask_upperbody=vectorn()
	self.scale_lleg=1
	self.scale_rleg=1
	self.scale_upperbody=1
	
	self.muscleActiveness=0.3
	self.kp_weight=1.0 -- use kp_active(1) or kp_passive(0)
	self.kd_weight=1.0 -- use kd_active(1) or kd_passive(0)
	self.mask_slide:setSize(dofInfo:numDOF())
	self.mask_slide:setAllValue(0)
	self.mask_lleg:setSize(dofInfo:numDOF())
	self.mask_rleg:setSize(dofInfo:numDOF())
	self.mask_upperbody:setSize(dofInfo:numDOF())
	self.mask_lleg:setAllValue(0)
	self.mask_rleg:setAllValue(0)
	self.mask_upperbody:setAllValue(1)

	self.dofInfo=dofInfo
	self:updateCoef()
	print ("kp=",self.kp)
	print ("kd=",self.kd)

	local skel=dofInfo:skeleton()

	local lhip=skel:getBoneByVoca(MotionLoader.LEFTHIP)
	local rhip=skel:getBoneByVoca(MotionLoader.RIGHTHIP)

	self.lkneeDOF=dofInfo:DOFindex(skel:getBoneByVoca(MotionLoader.LEFTKNEE):treeIndex(),0)
	self.rkneeDOF=dofInfo:DOFindex(skel:getBoneByVoca(MotionLoader.RIGHTKNEE):treeIndex(),0)
	self.lelbowDOF=dofInfo:DOFindex(skel:getBoneByVoca(MotionLoader.LEFTELBOW):treeIndex(),0)
	self.relbowDOF=dofInfo:DOFindex(skel:getBoneByVoca(MotionLoader.RIGHTELBOW):treeIndex(),0)
	local function setClampMax(clampForce, clampTorque)
		local clampMax=vectorn(dofInfo:numDOF())
		clampMax:setAllValue(0)
		for i=2,skel:numBone()-1 do
			local bone=skel:bone(i)
			local vbone=bone:treeIndex()
			local nJoint=dofInfo:numDOF(vbone)
			for j=0, nJoint-1 do
				local dofIndex=dofInfo:DOFindex(vbone,j)
				if dofInfo:DOFtype(vbone, j)==MotionDOFinfo.SLIDE then
					local dofIndex=dofInfo:DOFindex(vbone,j)
					self.mask_slide:set(dofIndex, 1)
					clampMax:set(dofIndex, clampForce)
				else
					clampMax:set(dofIndex, clampTorque)

					if bone:isDescendent(lhip) then
						self.mask_lleg:set(dofIndex,1)
						self.mask_upperbody:set(dofIndex,0)
					elseif bone:isDescendent(rhip) then
						self.mask_rleg:set(dofIndex,1)
						self.mask_upperbody:set(dofIndex,0)
					end


				end
			end
		end   
		return clampMax
	end

	local clampTorque=model.clampTorqueID or 400
	local clampForce=model.clampForceID or 4000

	self.clampMaxID=setClampMax(clampForce, clampTorque)

	clampTorque=model.clampTorque or 800
	clampForce=model.clampForce or 8000

	self.clampMax=setClampMax(clampForce, clampTorque)
	
	self.clampMin=self.clampMax*-1
	self.clampMinID=self.clampMaxID*-1

	self.numActualDOF=dofInfo:numActualDOF()
	self.workspace={}
	local w=self.workspace
	w.M=matrixn()
	w.b=vectorn(self.numActualDOF)
	w.JtV=matrixn()
	-- for friction cones
	w.J=matrixn()
	w.dotJ=matrixn()
	w.V=matrixn()
	w.dotV=matrixn()

	-- for acceleration cones
	if useCase.useRotationalFriction then
		w.J2=matrixn()
		w.dotJ2=matrixn()
	end
	w.V2=matrixn()
	w.dotV2=matrixn()

	w.Mlcp=matrixn()
	w.Mlcp_bias=vectorn()
	w.CE=matrixn()
	w.ce0=vectorn()
	w.CI=matrixn()
	w.ci0=vectorn()
	w.x=vectorn()
	w.x_lcp=vectorn()
	w.CElcp=matrixn()
	w.ce0lcp=vectorn()
	w.CIinvM=matrixn()

	w.clearCI=function(self, max_numCon, totalDIM) -- max_numCon은 정확할 필요 없음. 대충 적당히 넉넉히 주면 메모리 재할당이 줄듯.
		self.CI:setSize(max_numCon, totalDIM)
		self.CI:resize(0,totalDIM)
		self.ci0:setSize(max_numCon)
		self.ci0:resize(0)
	end
	w.addCI=function(self, CI, startIndex)
		local nrow=self.CI:rows()
		self.CI:resize(nrow+CI:rows(), self.CI:cols())
		self.CI:sub(nrow, nrow+CI:rows(),startIndex, startIndex+CI:cols()):assign(CI)
	end
	w.addCI0=function(self, ci0)
		local nrow=self.ci0:size()
		self.ci0:resize(nrow+ci0:size())
		self.ci0:range(nrow, nrow+ci0:size()):assign(ci0)
		assert(self.ci0:size()==self.CI:rows())
	end

	return o
end

function QPservo:initQPservo(startf, endf,motionDOF, dmotionDOF, ddmotionDOF)--, motionDOF_pdtarget)

	self.startFrame=startf
	self.endFrame=endf
	self.currFrame=startf
	self.deltaTime=0
	self.motionDOF=motionDOF
	self.dmotionDOF=dmotionDOF
	self.ddmotionDOF=ddmotionDOF
	--self.motionDOF_pdtarget=motionDOF_pdtarget or motionDOF

end

-- generate FBtorque
function QPservo:generateTorque(sim, maxForce)
	local simulator=self.simulator
	self.currFrame=(simulator:currentTime()+self.deltaTime)*model.frame_rate+self.startFrame
	--print(self.currFrame) -- extremely slow.
	if self.currFrame>self.endFrame-1 then
		self:sampleCurrPose(simulator)
	end
	
	self:_generateTorque(simulator, self.currFrame, maxForce)
	return true
end
function QPservo:_calcDesiredAcceleration(syn)
	if syn then
		self.state.reftime=syn.outputGlobal:getRefTime()
	end
	local state=self.theta
	local dstate=self.dtheta
	self.desiredacceleration:setSize(self.motionDOF:numDOF())
	
	--   self.desiredacceleration:assign(self.kp*(self.theta_d-state)+
	--			 self.kd*(self.dtheta_d-dstate))

	-- dcjo
--[[	
	for i = 1,state:size() do
		if state(i-1) < -3.141592 or state(i-1) > 3.141592 then
			print("pipipi")
			return false
		end
	end
]]

	local delta=self.theta_d-state
	--[[
	do
		local dofInfo=self.dofInfo
		local vbone=dofInfo:skeleton():getTreeIndexByName('lhumerus')
		local lclvcl_dofIndex= dofInfo:DOFindex(vbone,2)
		if self.theta_d(lclvcl_dofIndex)>0 then
			self.theta_d:set(lclvcl_dofIndex, self.theta_d(lclvcl_dofIndex)*-1)
		end
		local vbone=dofInfo:skeleton():getTreeIndexByName('rhumerus')
		local lclvcl_dofIndex= dofInfo:DOFindex(vbone,2)
		if self.theta_d(lclvcl_dofIndex)>0 then
			self.theta_d:set(lclvcl_dofIndex, self.theta_d(lclvcl_dofIndex)*-1)
			dbg.console()
		end
	end
	]]--
	MainLib.VRMLloader.projectAngles(delta) -- [-pi, pi]

--	do
--		local mLoader=self.dofInfo:skeleton()
--		local rhumerusIndex=mLoader:getTreeIndexByName('rhumerus')
--		local start_rh=mLoader.dofInfo:startR(rhumerusIndex)
--		print('RH:'..tostring(self.theta_d:range(start_rh, start_rh+3))..','..tostring(state:range(start_rh, start_rh+3))..
--		tostring(delta:range(start_rh, start_rh+3))..','..
--		tostring(self.dtheta_d:range(start_rh, start_rh+3))..','..
--		tostring(dstate:range(start_rh, start_rh+3))
--		)
--	end
	local V=MotionDOF.rootTransformation(state):twist(MotionDOF.rootTransformation(self.theta_d),1)

	delta:setVec3(0, V.v)
	delta:set(3,0)
	delta:setVec3(4, V.w)

	--self.desiredacceleration:setAllValue(0)
	self.dtheta_d:range(0,7):setAllValue(0)

	local dscale_coef=1.0
	local ddscale_coef=1.0
	if useCase.QPservoDScaleCoef then
		local startt=useCase.DScaleStart or -1
		local rt= self.state.reftime 
		if startt==-1 or rt> startt then
			dscale_coef=useCase.QPservoDScaleCoef 
			ddscale_coef=useCase.QPservoDDScaleCoef or ddscale_coef
			local clampDtheta_t=useCase.clampDtheta_d 
			if clampDtheta_t then
				self.dtheta_d:smoothClamp(-clampDtheta_t,clampDtheta_t)
			end
		end
	end

	local ddelta=(self.dtheta_d*dscale_coef-dstate)

	self.desiredacceleration:assign(self.kp_id*delta*useCase.k_p_ID +  self.kd_id*ddelta*useCase.k_d_ID)

	--self.desiredacceleration:range(0,7):setAllValue(0)
	local accClamp=useCase.desiredAccThr or 400
	self.desiredacceleration:smoothClamp(-accClamp, accClamp)

	self.ddtheta_d:range(0,7):setAllValue(0)
	self.desiredacceleration:radd(self.ddtheta_d*ddscale_coef)
	--print(self.ddtheta_d)
	--self.desiredacceleration:clamp(-400, 400)

--##dos if g_debugOneStep then --##dos --##dos g_debugOneStep:pushBack({"theta",self.theta:copy()}) --##dos g_debugOneStep:pushBack({"dtheta",self.dtheta:copy()}) --##dos g_debugOneStep:pushBack({"dtheta_d",self.dtheta_d:copy()}) --##dos g_debugOneStep:pushBack({"kp_id",self.kp_id:copy()}) --##dos g_debugOneStep:pushBack({"kd_id",self.kd_id:copy()}) --##dos g_debugOneStep:pushBack(useCase.QPservoDScaleCoef ) --##dos g_debugOneStep:pushBack({"delta",delta:copy()}) --##dos g_debugOneStep:pushBack({"ddtheta_d",self.ddtheta_d:copy()}) --##dos g_debugOneStep:pushBack({"desiredAcc",self.desiredacceleration:copy()}) --##dos end
end


-- deprecated: use _calcDesiredAcceleration
function QPservo:calcDesiredAcceleration(sim, frame, state, dstate)

	local simulator=self.simulator

	--[[ continuous sampling ]]--
	--   print("theta",self.theta)

	self:sampleTargetPoses(frame)

	--   self.dtheta_d:setAllValue(0)
	self:_calcDesiredAcceleration()
end

function QPservo:sampleCurrPose(sim)
	local simulator=self.simulator
	simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, self.theta)
	simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, self.dtheta)
end
function QPservo:sampleTargetPoses( frame)
	-- desired (target) pose
	self.motionDOF:samplePose(frame, self.theta_d)
--	self.motionDOF_pdtarget:samplePose(frame, self.theta_d_pd)
	self.dmotionDOF:sampleRow(frame, self.dtheta_d)
	self.ddmotionDOF:sampleRow(frame, self.ddtheta_d)
end

function QPservo:stepSimul(sim, impulse)
	local simulator=self.simulator
	if false then
		self:stepSimul_old(sim, impulse)
	else -- use QP solution directly
		local qp=self.qp
		local w=self.workspace
		local numActualDOF=self.numActualDOF
		--##dos if g_debugOneStep then --##dos g_debugOneStep:pushBack({"qpR",qp.H:copy(), qp.R:copy()}) --##dos end
		
		if impulse then
			local J=matrixn()
			simulator:calcJacobian(0,impulse.chest:treeIndex(),J)
			local tf=simulator:getWorldState(0):globalFrame(impulse.chest)
			-- refer to DynamicsSimulator_gmbs.cpp:calcContactjacobianAll
			local dAd=matrixn(6,6)
			dAd:setSize(6,6)
			dAd:setAllValue(0)
			dAd:diag():setAllValue(1)
			dAd:sub(0,3,3,6):assign(CT.skew(tf:toGlobalPos(impulse.chest:localCOM())*-1))
			local lf=impulse.lf
			local mlf=CT.mat(6,1,0,0,0,lf.x, lf.y, lf.z)
			print(lf)
			local tau=matrixn()
			tau:multAtB(J,dAd*liegroup.dAd(tf:inverse())*mlf)

			w.ce0:range(0,numActualDOF):rsub(tau:column(0))
		end
		if w.CI_additional and w.CI:cols()> 0 then
			for i,v in ipairs(w.CI_additional) do
				w:addCI(v[1], v[2])
				w:addCI0(v[3])
			end
		end
		Eigen.solveQuadprog(qp, w.CE, w.ce0, w.CI, w.ci0, w.x)
		--Eigen.solveQuadprog(qp,w.CE, w.ce0, w.CI, w.ci0, w.x, true)
		--_checkpoints:pushBack(deepCopyTable({'quadprog', w.CE, w.ce0, w.CI, w.ci0, w.x}))

		if false then
			if _count==1 then
				if util.isFileExist('_checkpoints.tbl') then
					tbl2=util.loadTable('_checkpoints.tbl')
					util.compareTable(_checkpoints, tbl2)
				else
					util.saveTable(_checkpoints, '_checkpoints.tbl')
				end
				dbg.console()
			end
			_count=_count+1
		end

		if useCase.enforceLCPcondition and w.link_pair_count>0 then
			self:enforceLCPcondition()
		end

		assert(w.x==w.x)
		self.controlforce:range(0,7):setAllValue(0)
		self.controlforce:range(7,self.controlforce:size()):assign(w.x:range(numActualDOF+6,numActualDOF*2))

		--##dos if g_debugOneStep then --##dos g_debugOneStep:pushBack({"qpcontrolForce", w.x:copy(), w.CE:copy(), w.ce0:copy(), w.CI:copy(), w.ci0:copy()}) end 
		local w=self.workspace
		local numDOF=self.numActualDOF
		local ddq=w.x:range(0,numDOF)
		local tau = w.x:range(numDOF, numDOF*2)
		local lambda= w.x:range(numDOF*2, w.x:size())

		local aggCF=vector3(0,0,0)
		if true then -- export contact force
			local lambda=w.x:range(self.numActualDOF*2, w.x:size())
			local skel=self.dofInfo:skeleton()
			local lfoot=skel:getTreeIndexByVoca(MotionLoader.LEFTANKLE)
			local ltoes=skel:getTreeIndexByVoca(MotionLoader.LEFTTOES)
			local rfoot=skel:getTreeIndexByVoca(MotionLoader.RIGHTANKLE)
			local rtoes=skel:getTreeIndexByVoca(MotionLoader.RIGHTTOES)

			
			if lambda:size()>0 then 

				local w=self.workspace
				assert(w.bases and  w.bases:size()>0)
				local collector
				if mrd_info and mrd_info.outputContactForce then
					collector=mrd_info.outputContactForce[2]
				else
					collector={vector3(0,0,0), vector3(0,0,0), vector3(0,0,0)}
				end
				local limbs={L=1,R=2,O=3}
				for i=1,3 do collector[i]:assign(vector3(0,0,0)) end
				local giOffset=self.giOffset
				local excludeNormalBasis=self.excludeNormalBasis
				local mat=nil -- do not draw contact forces
				if drawContactForces then
					mat=vector3N(w.bases:size()*2) -- draw contact forces
				end

				for i=0, w.bases:size()-1 do
					local b=w.bases(i)
					local limb=limbs.O
					if b.ibone==lfoot or b.ibone==ltoes then
						limb=limbs.L
					elseif b.ibone==rfoot or b.ibone==rtoes then
						limb=limbs.R
					end

					local cb=collector[limb]
					local cf=vector3(0,0,0)
					if not excludeNormalBasis then
						cf:radd(lambda(b.globalIndex)*b.normal)
					end
					for j=0, b.frictionNormal:size()-1 do
						local gi=b.globalFrictionIndex+j-giOffset
						cf:radd(lambda(gi)*b.frictionNormal(j))
					end
					cb:radd(cf)
					if mat then
						mat(i*2):assign(w.bases(i).globalpos)
						mat(i*2+1):assign(w.bases(i).globalpos+cf*0.005)
						--dbg.console()
					end
				end
				if mat then dbg.namedDraw('Traj', mat:matView()*100, "contacts") end
				for i=1,3 do 
					aggCF:radd(collector[i])
					self.state.prevContactY = aggCF:getY()
				end
			else
				local mat=vector3N(2)
				mat:matView():setAllValue(0)
				dbg.draw('Traj', mat:matView()*100, "contacts") 
			end
		end
		self.state.aggContactForce:assign(aggCF)
		if false then -- verify QP solution
			self:verifyQPsolution()
		end
		--print('ddq', ddq)
		--print('before 2', self.theta)
		if true then
			simulator:stepKinematic(ddq, vectorn(), true)
		else
			local link_pair_count=w.link_pair_count

			local numDOF=self.numActualDOF
			local rootR = self.simulator:getWorldState(0):globalFrame(1).rotation
			local function packTau( tau)
				local genForce=vectorn(numDOF+1)
				local M = tau:toVector3(0)
				local F = tau:toVector3(3)
				--M:rotate(rootR)
				--F:rotate(rootR)
				genForce:setVec3(4, M)
				genForce:setVec3(0, F)
				genForce:range(7,genForce:size()):assign(tau:range(6,tau:size()))
				return genForce
			end
			local controlForce=packTau( tau)
			if link_pair_count>0 then
				local cf=w.JtV*lambda:column()
				local genContactForce=packTau( cf:column(0))

				controlForce:radd(genContactForce)
			end
			simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_TORQUE, controlForce)
			simulator:stepSimulation()
		end

		--self:sampleCurrPose()
		--print('after 2', self.theta)
	end
end
function QPservo:_generateTorque(sim, frame, maxForce, swingFoot)
	local simulator=self.simulator

	self:sampleCurrPose(simulator)
	self:calcDesiredAcceleration(simulator, frame, self.theta, self.dtheta)

	self:QPsolve(simulator, self.theta, self.dtheta, {L=1,R=1,LH=1, RH=1,O=1}, {L=1,R=1,LH=1, RH=1,O=1})
end

function QPservo:calcDQ()
	local dq=vectorn(self.dtheta:size()-1)
	dq:range(0,3):assign(self.dtheta:range(4,7)) dq:range(3,6):assign(self.dtheta:range(0,3)) dq:range(6,dq:size()):assign(self.dtheta:range(7,self.dtheta:size()))
	return dq
end
function QPservo:addCOMobjective(sim, desiredCOMacc, weight)
	local simulator=self.simulator
	local weight=weight or 5000
	if weight~=0 then
		local qp=self.qp
		local J=matrixn()
		local DJ=matrixn()
		local dq=self:calcDQ()
		simulator:calcCOMdotJacobian(0, J, DJ) -- com j and dj
		--qp:addV(weight, J, 0, dq:size(), DJ*dq:column()-CT.mat(3,1,desiredCOMacc.x, desiredCOMacc.y, desiredCOMacc.z))
		qp:addVselective(weight, J, 0, dq:size(), DJ*dq:column()-CT.mat(3,1,desiredCOMacc.x, desiredCOMacc.y, desiredCOMacc.z), CT.vec(1))
	end
end
function QPservo:addBoneObjective(sim, ibone, localpos, desiredAcc, weight, onlyHorizontal)
	local simulator=self.simulator
	local weight=weight or 50000
	local qp=self.qp
	local J=matrixn()
	local DJ=matrixn()
	local dq=self:calcDQ()
	simulator:calcBoneDotJacobian(0, ibone, localpos, J, DJ) 
	--##dos if g_debugOneStep then --##dos g_debugOneStep:pushBack({"addboneobj",ibone,localpos:copy(), J:copy(), DJ:copy(), desiredAcc:copy(),weight }) --##dos end 
	if onlyHorizontal then
		qp:addVselective(weight, J, 0, dq:size(), DJ*dq:column()-CT.mat(3,1,desiredAcc.x, desiredAcc.y, desiredAcc.z), CT.vec(0,2))
	else
		qp:addV(weight, J, 0, dq:size(), DJ*dq:column()-CT.mat(3,1,desiredAcc.x, desiredAcc.y, desiredAcc.z))
	end

end
function QPservo:addBoneObjective2(sim, ibone, localpos, desiredAccRot, desiredAcc, weight, weightLin)
	local simulator=self.simulator
	local weight=weight or 50000
	if weightLin==nil then
		weightLin=weight
	end
	local qp=self.qp
	local J=matrixn()
	local DJ=matrixn()
	local dq=self:calcDQ()
	simulator:calcBoneDotJacobian2(0, ibone, localpos, J, DJ) 
	--##dos if g_debugOneStep then --##dos g_debugOneStep:pushBack({"addboneobj2",ibone,localpos:copy(),desiredAccRot:copy(),desiredAcc:copy(), weight, J:copy(), DJ:copy()}) --##dos end 
	local desired=DJ*dq:column()-CT.mat(6,1,desiredAccRot.x, desiredAccRot.y, desiredAccRot.z, desiredAcc.x, desiredAcc.y, desiredAcc.z)
	qp:addVselective(weight*0.5, J, 0, dq:size(), desired , CT.vec(0,1,2)) -- angular acc
	qp:addVselective(weightLin, J, 0, dq:size(), desired , CT.vec(3,4,5)) -- linear acc
end
function QPservo:addBoneObjectiveAngular(sim, ibone, desiredAccRot, weight)
	local simulator=self.simulator
	local weight=weight or 50000
	local qp=self.qp
	local J=matrixn()
	local DJ=matrixn()
	local dq=self:calcDQ()
	simulator:calcBoneDotJacobian2(0, ibone, vector3(0,0,0), J, DJ) 
	local desired=DJ*dq:column()-CT.mat(6,1,desiredAccRot.x, desiredAccRot.y, desiredAccRot.z, 0,0,0)
	qp:addVselective(weight*0.5, J, 0, dq:size(), desired , CT.vec(0,1,2)) -- angular acc
end
function QPservo:addMomentumObjective(sim_unused,desiredDotAngMomentum, desiredDotLinMomentum,weight_ang, weight_lin)
	local simulator=self.simulator
	if self.workspace.link_pair_count>0 then
		local weight=weight or 50000
		local qp=self.qp
		local J=matrixn()
		local DJ=matrixn()


		local dq=self:calcDQ()
		simulator:calcMomentumDotJacobian(0, J, DJ) 

		if false then 
			self:addMomentumObjective_additional(sim_unused,desiredDotAngMomentum, desiredDotLinMomentum,weight_ang, weight_lin)
		end
		--##dos if g_debugOneStep then --##dos local ee=matrixn() simulator:test('getExactStateSpherical', ee) --##dos local theta=vectorn() local dtheta=vectorn() simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, theta) simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, dtheta) g_debugOneStep:pushBack({"addmmobj",ee,simulator._lastSimulatedPose:copy(), simulator:calculateCOM(0),simulator:calcCOM2_gmbs(0),simulator:calculateCOMvel(0), desiredDotAngMomentum:copy(), theta, dtheta, J:copy(), DJ:copy(),weight}) --##dos end 
		qp:addVselective(weight_ang, J, 0, dq:size(), DJ*dq:column()-CT.mat(6,1,desiredDotAngMomentum.x, desiredDotAngMomentum.y, desiredDotAngMomentum.z,
		desiredDotLinMomentum.x, desiredDotLinMomentum.y, desiredDotLinMomentum.z), CT.vec(0,1,2))
		if weight_lin then
			qp:addVselective(weight_lin, J, 0, dq:size(), DJ*dq:column()-CT.mat(6,1,desiredDotAngMomentum.x, desiredDotAngMomentum.y, desiredDotAngMomentum.z,
			desiredDotLinMomentum.x, desiredDotLinMomentum.y, desiredDotLinMomentum.z), CT.vec(3,4,5))
		end
		--RE.output2("momentumOBJ","true")
	else
		--RE.output2("momentumOBJ","false")
	end
end
function QPservo:QPsolve(sim, state, dstate, spprtImportance, importance)
	local simulator=self.simulator

	
	local link_pair_count=simulator:getNumContactLinkPairs()
	local numActualDOF=self.numActualDOF
	local w=self.workspace
	simulator:calcMassMatrix3(0, w.M, w.b)
	--_checkpoints:pushBack(deepCopyTable({'mass', w.M, w.b, link_pair_count}))
	--print('state', self.theta, self.dtheta)
	--print('mass', w.M, w.b, link_pair_count)
	--self.simulator:skeleton(0):printHierarchy()
	--for i=1, self.simulator:skeleton(0):numBone()-1 do print(self.simulator:skeleton(0):VRMLbone(i):mass()) end

--##dos if g_debugOneStep then --##dos print('_qpsolv') --##dos g_debugOneStep:pushBack({'state', state:copy(), dstate:copy()}) --##dos g_debugOneStep:pushBack({'desiredacc', self.desiredacceleration:copy()}) --##dos end

	if false then
		self:testMassMatrix()
	end
	self.controlforce:setSize(numActualDOF+1)
	self.controlforce:setAllValue(0)
	w.link_pair_count=link_pair_count

	local state=self.state
	for i=1, self.numCLinks do state.prevContact[i]=state.contact[i] state.contact[i]=false end
	w.CI_additional=nil
	if link_pair_count>0 then
		local useEqualityVelConstraint=useCase.useEqualityVelConstraint


		if useCase.useContactCentroid then
			--hwangpil
			--calculate centroid of each contact body
			--and change contact mode ( multi contact -> pseudo-single contact ) 
			local bases=Physics.Vec_ContactBasis()
			local accbases=Physics.Vec_ContactBasis()
			simulator:getContactBases(bases,self.invfricCoef)
			simulator:getContactBases(accbases,useCase.invAccConeCoef or self.invfricCoef)

			w.bases = vector()
			w.accbases = vector()
			local torqueNormalSize=1
			if useCase.useThreeDimTorque then
				torqueNormalSize=3
			end

			local function makeSingleFromMulti(singleBasis, multiBasis) 
				local frictionNormalSize
				local contact_info_idx = 1
				local boneToIdx={} -- contact idx
				local contact_info={}

				for i=0, multiBasis:size()-1 do
					local b=multiBasis(i)
					local ibone = b.ibone
					if boneToIdx[ibone]==nil then
						boneToIdx[ibone] = contact_info_idx

						local cinfo={
							cHull=ConvexHull2D(),
							depth=0,
							ilinkpair=-1,
							globalpos=vector3(0,0,0),
							cNum=0,
							normal=vector3(0,0,0),
							frictionNormal=vector(b.frictionNormal:size(), function() return vector3(0,0,0) end ),
							ibone=ibone,
						}
						contact_info[contact_info_idx]=cinfo
						contact_info_idx = contact_info_idx +1
					end
					local cInfo=contact_info[boneToIdx[ibone]]
					cInfo.cHull:addVector3(b.globalpos)
					cInfo.depth = cInfo.depth+b.depth
					assert(cInfo.ilinkpair ==-1 or cInfo.ilinkpair ==b.ilinkpair)
					cInfo.ilinkpair = b.ilinkpair
					cInfo.globalpos = cInfo.globalpos+b.globalpos
					cInfo.cNum = cInfo.cNum+1
					cInfo.normal = cInfo.normal+b.normal
					frictionNormalSize = b.frictionNormal:size()

					for j=0, b.frictionNormal:size()-1 do
					--	if cInfo.frictionNormal[j]==nil then cInfo.frictionNormal[j]=vector3(0,0,0) end
						--cInfo.frictionNormal[j] = cInfo.frictionNormal[j]+b.frictionNormal(j)
						cInfo.frictionNormal:set(j, cInfo.frictionNormal(j)+b.frictionNormal(j))
					end
				end
				for i,cInfo in ipairs(contact_info) do
					cInfo.cHull:buildHull()
					cInfo.cPoint, cInfo.area=cInfo.cHull:calcCentroid()
					cInfo.depth=cInfo.depth/cInfo.cNum
					cInfo.globalpos=cInfo.globalpos/cInfo.cNum
					cInfo.normal=cInfo.normal/cInfo.cNum
					cInfo.normal:normalize()
					for j=0, frictionNormalSize-1 do
						cInfo.frictionNormal:set(j, cInfo.frictionNormal(j)/cInfo.cNum)
						cInfo.frictionNormal(j):normalize()
					end
				end
				singleBasis:resize(#contact_info)
				for i,cinfo in ipairs(contact_info) do
					singleBasis:set(i-1, cinfo)
					local b = singleBasis(i-1)
					b.ibody=0
					b.globalpos = vector3(b.cPoint.x, b.globalpos.y, b.cPoint.y) 
					b.globalIndex = i-1
					b.globalFrictionIndex = #contact_info + frictionNormalSize*(i-1)
					local lpos=simulator:getWorldState(0):globalFrame(b.ibone):toLocalPos(b.globalpos)
					b.relvel=simulator:getWorldVelocity(0, simulator:skeleton(0):VRMLbone(b.ibone), lpos):copy()
					b.torqueNormal=vector(torqueNormalSize, function() return vector3(0,0,0) end )
					if useCase.useRotationalFriction then
						b.globalTorqueIndex = #contact_info*(1+frictionNormalSize) + torqueNormalSize*(i-1)
						b.torqueNormal:set(0, b.normal:copy())
						for j=1, torqueNormalSize-1 do
							b.torqueNormal:set(j, (b.frictionNormal(j)-b.frictionNormal(j):dotProduct(b.normal)*b.normal):copy())
							b.torqueNormal(j):normalize()
						end
					else
						b.torqueNormal=vector(0)
					end
				end
			end	
			local function copyOneBasis(b, maxNormalBasis)
				local cinfo={
					depth=b.depth,
					ilinkpair=b.ilinkpair,
					globalpos=b.globalpos:copy(),
					normal=b.normal:copy(),
					torqueNormal=vector(0),
					ibone=b.ibone,
					ibody=b.ibody,
					relvel=b.relvel:copy(),
					area=3.14*0.02*0.02, -- MIN_AREA
				}
				maxNormalBasis=maxNormalBasis or b.frictionNormal:size()
				cinfo.frictionNormal=vector(maxNormalBasis)
				for j=0, maxNormalBasis-1 do
					cinfo.frictionNormal:set(j, b.frictionNormal(j):copy())
				end
				return cinfo
			end
			local function copyBases(basesout, basesin, maxNormalBasis)
				basesout:resize(basesin:size())
				for i=0, basesin:size()-1 do
					local b=basesin(i)
					assert(basesout(i)==nil)
					local cinfo=copyOneBasis(b,maxNormalBasis)
					basesout:set(i, cinfo)
				end
				updateFrictionIndex(basesout)
			end
			local function mergeBases(basesout, bases1, bases2)
				basesout:resize(bases1:size()+bases2:size())
				for i=0, bases1:size()-1 do
					local b=bases1(i)
					assert(basesout(i)==nil)
					local cinfo=copyOneBasis(b)
					basesout:set(i, cinfo)
				end
				for i=0, bases2:size()-1 do
					local b=bases2(i)
					local ti=bases1:size()+i
					assert(basesout(ti)==nil)
					local cinfo=copyOneBasis(b)
					basesout:set(ti, cinfo)
				end
				updateFrictionIndex(basesout)
			end
			function updateFrictionIndex(basesin)
				local fi=basesin:size() 
				for i=0, basesin:size()-1 do
					local b=basesin(i)
					b.globalIndex=i
					b.globalFrictionIndex=fi
					fi=fi+b.frictionNormal:size()
				end
			end

			if false then
				makeSingleFromMulti(w.bases, bases)
				makeSingleFromMulti(w.accbases, accbases)
			elseif false then
				copyBases(w.bases, bases)
				copyBases(w.accbases, accbases)
			elseif true then
				copyBases(w.bases, bases)
				local accbases1 = vector()
				local accbases2 = vector()
				copyBases(accbases1, accbases, 1)
				makeSingleFromMulti(accbases2, accbases)
				mergeBases(w.accbases, accbases1, accbases2)
			else
				copyBases(w.bases, bases)
				makeSingleFromMulti(w.accbases, accbases)
			end

			w.link_pair_count = w.bases:size()

			--w.bases = bases
			--w.accbases = accbases

			--\hwangpil
			-- 
		else
			if w.bases==nil then w.bases=Physics.Vec_ContactBasis() end
			if w.accbases==nil then w.accbases=Physics.Vec_ContactBasis() end
			simulator:getContactBases(w.bases,self.invfricCoef)
			simulator:getContactBases(w.accbases,useCase.invAccConeCoef or self.invfricCoef) -- basis vectors for acceleration cone can be different from those of friction cones
		end

		local giOffset=0
		local excludeNormalBasis=self.excludeNormalBasis  -- redundant basis vector

		if excludeNormalBasis then
			for i=0, w.bases:size()-1 do
				local b=w.bases(i)
				local gi=b.globalIndex
				giOffset=math.max(giOffset, gi+1)
			end
		end
		self.giOffset=giOffset
		if useCase.useRotationalFriction then
			local ENABLETORQUE = true

			local function calcContactJacobianAll(bases, J, dotJ, V, dotV, linkPairCount, frictionCoef)
				local V_temp = matrixn()
				local dotV_temp = matrixn()
				local N = numActualDOF
				local numTorqueVector = bases(0).torqueNormal:size()
				local a = bases(bases:size()-1).globalFrictionIndex+bases(bases:size()-1).frictionNormal:size()

				if ENABLETORQUE then
					a = a+ bases:size() * numTorqueVector
				end

				J:setSize(6*linkPairCount,N)
				dotJ:setSize(6*linkPairCount,N)
				V:setSize(6*linkPairCount,a)
				dotV:setSize(6*linkPairCount,a)
				V_temp:setSize(6,a)
				dotV_temp:setSize(6,a)

				local dot_R_dAd=matrixn()
				dot_R_dAd:setSize(6,6)
				dot_R_dAd:setAllValue(0)

				for i=0, bases:size()-1 do
					local b=bases(i)
					local J_i=matrixn()
					local dotJ_i=matrixn()
					J_i:setSize(6,N)
					dotJ_i:setSize(6,N)
					simulator:calcJacobian(b.ibody, b.ibone, J_i)
					simulator:calcDotJacobian(b.ibody, b.ibone, dotJ_i)
					assert((i+1)*6<=J:rows())
					assert(N==J_i:cols())
					J:range( i*6, (i+1)*6, 0, N):assign(J_i)
					dotJ:range( i*6, (i+1)*6, 0, N):assign(dotJ_i)

					do
						V_temp:setAllValue(0)
						dotV_temp:setAllValue(0)

						local n = b.normal:copy()
						local p = b.globalpos:copy()
						local pcn = p:cross(n)

						local globalIndex = b.globalIndex
						local vk 
						local dot_vk 
						assert(globalIndex>=0 and globalIndex<V_temp:cols())
						vk = V_temp:column(globalIndex)
						dot_vk = dotV_temp:column(globalIndex)
						vk:setVec3(0, vk:toVector3(0)+pcn)
						vk:setVec3(3, vk:toVector3(3)+n)
						local dot_p = vector3(0,0,0)
						dot_p = -1*b.relvel:copy()
						pcn=dot_p:cross(n)
						dot_vk:setVec3(0, dot_vk:toVector3(0)+pcn)
						local n0=n:copy()

						for j=0, b.frictionNormal:size()-1 do
							local vk=V_temp:column(b.globalFrictionIndex+j-giOffset);
							local dot_vk=dotV_temp:column(b.globalFrictionIndex+j-giOffset)
							local n=b.frictionNormal(j)
							pcn=p:cross(n)

							-- transform the global generalized force (0, n) to spatial coordinate
							-- local f=liegroup.dse3(vector3(0,0,0), n)
							-- vk+=f:invdAd(transf(quater(1,0,0,0), p)))
							vk:setVec3(0, vk:toVector3(0)+pcn)
							vk:setVec3(3, vk:toVector3(3)+n)
							pcn=dot_p:cross(n)
							dot_vk:setVec3(0,dot_vk:toVector3(0)+pcn)
						end

						if ENABLETORQUE then
							for j=0, b.torqueNormal:size()-1 do
								--local gi = (bases:size()*(b.frictionNormal:size()+1)+(b.globalFrictionIndex-bases:size())*numTorqueVector/b.frictionNormal:size()+j)
								local gi=b.globalTorqueIndex+j-giOffset 
								local n=b.torqueNormal(j)
								-- transform the global generalized force (n, 0) to spatial coordinate
								-- local f=liegroup.dse3(n, vector3(0,0,0))
								-- vk+=f:invdAd(transf(quater(1,0,0,0), p)))
								vk=V_temp:column(gi)
								vk:setVec3(0, vk:toVector3(0)+n)
							end
						end
					end
					local GT=simulator:getWorldState(0):globalFrame(b.ibone)
					-- spatial coordinate to bone global coordinate
					local R_dAd= liegroup.invdAd(transf(GT.rotation,vector3(0,0,0))*GT:inverse())
					
					V:range(i*6, (i+1)*6, 0, a):assign(R_dAd*V_temp)

					local iboneVel=-1*simulator:getWorldVelocity(0, simulator:skeleton(0):VRMLbone(b.ibone), vector3(0,0,0)):copy()
					dot_R_dAd:range(0,3,3,6):assign(CT.skew(iboneVel):copy())
					dotV:range(i*6, (i+1)*6, 0, a):assign(dot_R_dAd*V_temp + R_dAd*dotV_temp)
				end
			end

			calcContactJacobianAll(w.bases, w.J, w.dotJ, w.V, w.dotV, w.bases:size(), self.invfricCoef)
			calcContactJacobianAll(w.accbases, w.J2, w.dotJ2, w.V2, w.dotV2, w.accbases:size(), useCase.invAccConeCoef or self.invfricCoef)

			w.JtV:multAtB(w.J, w.V)
			--\hwangpil
		elseif true then
			-- setting friction cone for contact force
			simulator:calcContactJacobianAll(w.J, w.dotJ, w.V, w.dotV, link_pair_count,self.invfricCoef)
			-- setting acceleration cone
			simulator:calcContactBasisAll(w.V2, w.dotV2, link_pair_count,useCase.invAccConeCoef or self.invfricCoef)
			if excludeNormalBasis then
				w.V:assign(w.V:sub(0,0,giOffset,0):copy())
				w.dotV:assign(w.dotV:sub(0,0,giOffset,0):copy())
				w.V2:assign(w.V2:sub(0,0,giOffset,0):copy())
				w.dotV2:assign(w.dotV2:sub(0,0,giOffset,0):copy())
			end
			w.JtV:multAtB(w.J, w.V)
			-- mJtV=matrixn()
			-- simulator:calcContactJacobian(mJtV, link_pair_count)
			-- dbg.console()
		else
			simulator:calcContactJacobian(w.JtV, link_pair_count)
		end
		local cdim=w.JtV:cols()
		local totalDIM=numActualDOF*2+cdim -- ddq, tau, lambda
		local qp=HessianQuadratic(totalDIM)
		self.qp=qp
		local ddqObjWeight=useCase.ddqObjWeight or 10000
		local ddqObjWeight2=useCase.ddqObjWeight2 or ddqObjWeight
		local weight=self.weight
		assert(ddqObjWeight)
		-- minimize desired acc error
		if self.excludeRoot then
			local w=1
			for i=0,3 do -- root
				qp:addD(w,i,0)
			end
			for i=3,6 do -- root
				qp:addD(w,i,0)
			end
		else
			for i=0,3 do -- root
				qp:addD(ddqObjWeight*self.weight(i+4),i,self.desiredacceleration(i+4))
			end
			if useCase.excludeRootPos then
				for i=3,6 do -- root
					qp:addD(1,i,self.desiredacceleration(i-3))
				end
			else
				for i=3,6 do -- root
					qp:addD(ddqObjWeight*self.weight(i-3),i,self.desiredacceleration(i-3))
				end

				--local dv=self.desiredacceleration:toVector3(0)
				--local R=simulator:getWorldState(0):globalFrame(1).rotation
				--dbg.draw('Arrow', vector3(0,100,0), vector3(0,100,0)+rotate(dv,R)*100,'R*dv')
			end
		end

		--_checkpoints:pushBack(deepCopyTable({'wi', self.weight, self.desiredacceleration}))
		do
			importance.O=1
			local limb=self.dofToLimb
			local invmap=self.dofToLimbInvMap
			for i=6,numActualDOF-1 do
				local imp=importance[invmap[limb(i+1)]]
				local w=sop.map(imp, 0,1, ddqObjWeight2, ddqObjWeight)
				qp:addD(w*self.weight(i+1),i,self.desiredacceleration(i+1))
			end
		end

		--##dos if g_debugOneStep then --##dos g_debugOneStep:pushBack({"qp1",qp.H:copy(), qp.R:copy()}) --##dos end

		-- minimize joint torque
		if useCase.tauObjWeight>1 then
			local w=useCase.tauObjWeight or 0.00001
			for i=0,5 do
				qp:addD(0.00001,i+numActualDOF,0)
			end
			for i=6,numActualDOF-1 do
				qp:addD(w*self.weight(i+1),i+numActualDOF,self.desiredacceleration(i+1)*2)
			end
		else
			local w=useCase.tauObjWeight or 0.00001
			assert(w)
			for i=0,numActualDOF-1 do
				qp:addD(w,i+numActualDOF,0)
			end
		end
		local minimizeCOMjerk=false
		if true then
			-- minimize contact force
			local lw=useCase.lambdaObjWeight or 0.00001
			local lw2=useCase.lambdaObjWeight2 or 10
			assert(lw)
			if spprtImportance then
				local boneIndex=intvectorn()
				if useCase.useRotationalFriction then
					boneIndex=intvectorn(cdim)
					for i=0, w.bases:size()-1 do
						local b=w.bases(i)
						local gi=b.globalIndex
						if not excludeNormalBasis then
							boneIndex:set(gi,b.ibone)
						end
						for j=0, b.frictionNormal:size()-1 do
							local gi=b.globalFrictionIndex+j-giOffset
							boneIndex:set(gi,b.ibone)
						end
						for j=0, b.torqueNormal:size()-1 do
							local gi=b.globalTorqueIndex+j-giOffset
							boneIndex:set(gi,b.ibone)
						end
					end
				else
					simulator:calcContactBoneIndex(link_pair_count, boneIndex);
				end
				local importance=vectorn(cdim)
				importance:setAllValue(0)
				local bi=self.bi
				for k, isL in ipairs({'L','R','LH', 'RH'}) do
					for i=0, cdim -1 do
						local bii=boneIndex(i)
						local bii2=bi[isL]
						if bii==bii2 or bii==bii2+1 then -- ankle or toe
							importance:set(i,spprtImportance[isL] or 0)
						end
					end
					--RE.output('imp_'..isL, spprtImportance[isL])
				end
				local function impMap(imp)
					return sop.map(math.pow(1-imp,4), 1,0, lw, lw2)
				end
				if useCase.spprtImpFromImp then
					impMap=function(imp)
						return sop.map(math.pow(imp,4), 0,1, lw, lw2)
					end
				end
				if useCase.minimizeMidSpprtContactForceY then
					impMap=function(imp)
						if imp==1 then
							local reftime=self.state.reftime
							local reftimeInOnePeriod=reftime-math.floor(reftime)
							local sppCenter=0.35
							local sppCenter2=0.55
							local gamma=2
							local weight
							local lw_mid=lw2*10
							if reftimeInOnePeriod<sppCenter then
								local t=sop.map(reftimeInOnePeriod, 0, sppCenter, 0, 1)
								weight=sop.map(math.pow(t, gamma), 0, 1, lw,lw_mid)
							elseif reftimeInOnePeriod > sppCenter2 then
								local t=sop.map(reftimeInOnePeriod, sppCenter2, 1, 0, 1)
								weight=sop.map(math.pow(t, gamma), 0, 1, lw_mid, lw)
							else
								weight=lw_mid
							end
							return weight
						else
							return sop.map(math.pow(imp,4), 0,1, lw, lw2)
						end
					end
				end
				for i=0, w.bases:size()-1 do
					local b=w.bases(i)
					local gi=b.globalIndex
					if not excludeNormalBasis then
						qp:addD(impMap(importance(gi)),gi+numActualDOF*2,0)
					end
					for j=0, b.frictionNormal:size()-1 do
						gi=b.globalFrictionIndex+j-giOffset
						qp:addD(impMap(importance(gi)),gi+numActualDOF*2,0)
					end
					if b.torqueNormal then
						--hwangpil
						for j=0, b.torqueNormal:size()-1 do
							local gi=b.globalTorqueIndex+j-giOffset
							qp:addD(impMap(importance(gi)),gi+numActualDOF*2,0)
						end
						--\hwangpil
					end
				end

				--for i=0,cdim-1 do
				--	qp:addD(sop.map(importance(i), 0,1, lw, 0.0001),i+numActualDOF*2,0)
				--	qp:addD(sop.map(math.pow(importance(i),2), 0,1, lw, 0.0001),i+numActualDOF*2,0)
				--	qp:addD(sop.map(math.pow(1-importance(i),4), 1,0, lw, 10),i+numActualDOF*2,0)
				--	qp:addD(sop.map(math.pow(1-importance(i),4), 1,0, lw, 0.0001),i+numActualDOF*2,0)
				--  qp:addD(sop.clampMap(importance(i),0,0.01,lw,10) ,i+numActualDOF*2,0)
				--	end
				if importance(0)>=0.01 and importance(0)<=0.99 then
					minimizeCOMjerk=true
				end
			else
				for i=0,cdim-1 do
					qp:addD(lw,i+numActualDOF*2,0)
				end
			end
		end
		
		--hwangpil
		if false and useCase.useRotationalFriction then
			--minimize y component of dot contact force
			if not self.state.prevContactY then self.state.prevContactY=0 end
			local b=vectorn()
			local M=matrixn()
			M:setSize(1, cdim-w.bases:size()*w.bases(0).torqueNormal:size())
			b:setSize(1)
			b:set(0, -1*self.state.prevContactY)
			for i=0, w.bases:size()-1 do
				local b=w.bases(i)
				local gi=b.globalIndex
				if not excludeNormalBasis then
					M:set(0, gi, 1)
				end
				for j=0, b.frictionNormal:size()-1 do
					gi=b.globalFrictionIndex+j-giOffset
					M:set(0, gi, b.frictionNormal(j):dotProduct(b.normal))
				end
			end
			if true then
				-- soft constaints 
				--qp:addV(500, M, numActualDOF*2, numActualDOF*2+M:cols(), b:column())
				qp:addV(1, M, numActualDOF*2, numActualDOF*2+M:cols(), b:column())
			else
				-- hard constaints (will be appended at the end of w.CI) 
				local bound=200
				w.CI_additional={
					-- fy >  prevContactY-bound
					-- fy -prevContactY+bound >0
					{M, numActualDOF*2, b+CT.ones(1)*bound},
					-- fy < prevContactY+bound
					-- -fy > -prevContactY-bound
					{-M, numActualDOF*2, -b+CT.ones(1)*bound},
				}
			end
		end
		--/hwangpil


		-- set inequality constraints
		do
			-- use [de Lasa et al, SIGGRAPH2010]
			-- a_c=V'J ddq + V'dotJ dq + dotV' J dq >= 0

			local link_pair_count=w.link_pair_count
			assert(w.J:rows()==link_pair_count*6)
			assert(w.J:cols()==numActualDOF)
			assert(w.V:rows()==link_pair_count*6)
			assert(w.V:cols()==cdim)

			if w.VtJ==nil then w.VtJ=matrixn() end
			if w.VtJ2==nil then w.VtJ2=matrixn() end
			if w.VtDotJ==nil then w.VtDotJ=matrixn() end
			if w.dotVtJ==nil then w.dotVtJ=matrixn() end
			if useCase.useRotationalFriction then
				-- acc cones can be different from friction cones
				w.VtJ:transpose(w.JtV)
				w.VtJ2:multAtB(w.V2, w.J2)
				w.VtDotJ:multAtB(w.V2, w.dotJ2)
				w.dotVtJ:multAtB(w.dotV2, w.J2)
			else
				w.VtJ:transpose(w.JtV)
				w.VtJ2:multAtB(w.V2, w.J)
				w.VtDotJ:multAtB(w.V2, w.dotJ)
				w.dotVtJ:multAtB(w.dotV2, w.J)
			end

			local numConTau=numActualDOF-6
			local clampTorque=true
			local maxTorque= useCase.maxTorque or 400
			if clampTorque==false then
				maxTorque=160 --  actually maxAcceleration (clampAcceleration)
			end

			if useCase.useRotationalFriction then
				w:clearCI(w.VtJ:rows()+(cdim-w.bases:size())+numConTau*2+1+w.bases:size()*2, totalDIM)
			else
				w:clearCI(w.VtJ:rows()+cdim+numConTau*2+1, totalDIM)
			end
			--if minimizeCOMjerk then
			if false then
				-- minimize COM jerk - actually increased rolling and pitching
				local weight={}
				local axes={'x','y','z'}
				for i,a in ipairs(axes) do
					weight[a]=vectorn(cdim+1)
					weight[a]:zero()
				end
				for i=0, w.bases:size()-1 do
					local b=w.bases(i)
					local gi=b.globalIndex
					if not excludeNormalBasis then
						for k,a in ipairs(axes) do weight[a]:set(gi, weight[a](gi)+b.normal[a]) end
					end
					for j=0, b.frictionNormal:size()-1 do
						gi=b.globalFrictionIndex+j-giOffset
						for k,a in ipairs(axes) do weight[a]:set(gi, weight[a](gi)+b.frictionNormal(j)[a]) end
					end
				end
				local index=CT.colon(numActualDOF*2, totalDIM,1)
				for k,a in ipairs(axes) do
					weight[a]:set(weight[a]:size()-1, self.state.aggContactForce[a]*-1)
					if minimizeCOMjerk then
						qp:addSquaredW(800, index, weight[a])
					else
						qp:addSquaredW(50, index, weight[a])
					end
				end
			end
			-- update contact state
			for i=0, w.bases:size()-1 do
				state.contact[w.bases(i).ilinkpair+1]=true
			end
			-- state.contact are
			-- Left heel, Left toe, Right heel, Right toe, Left hand, Right hand, respectively
			-- w.bases(i).ilinkpair has those values for each w.bases(i)
			-- print(w.bases(1).normal:dotProduct(vector3(1,1,1)))

			local dq=w.dq
			if dq==nil then w.dq=vectorn(self.dtheta:size()-1) dq=w.dq end
			dq:range(0,3):assign(self.dtheta:range(4,7))
			dq:range(3,6):assign(self.dtheta:range(0,3))
			dq:range(6,dq:size()):assign(self.dtheta:range(7,self.dtheta:size()))

			w:addCI(w.VtJ2, 0) -- ddq (contact acceleration constraints)
			w:addCI0((w.VtDotJ*dq:column()):column(0)+(w.dotVtJ*dq:column()):column(0), 0)
			if useCase.useRotationalFriction then
				w:addCI(CT.eye(cdim-w.bases:size()*w.bases(0).torqueNormal:size()), numActualDOF*2) --lambdas (friction cone coefs). These constrain contact forces.
				w:addCI0(CT.zeros(cdim-w.bases:size()*w.bases(0).torqueNormal:size()))
			else
				w:addCI(CT.eye(cdim), numActualDOF*2) --lambdas (friction cone coefs). These constrain contact forces.
				w:addCI0(CT.zeros(cdim))
			end

			--_checkpoints:pushBack(deepCopyTable({'w', w.VtJ,w.dotVtJ}))
			--_checkpoints:pushBack(deepCopyTable({'dq',dq, w.CI, w.ci0}))

			if numConTau>0 then
				local startc=numActualDOF
				if not clampTorque then startc=0 end
				-- -tau + maxTorque>0
				w:addCI(CT.eye(numConTau)*-1, startc+6)
				local www=self.weight2:range(7,numActualDOF+1)
				w:addCI0(maxTorque*www)
				-- tau+maxTorque>0
				w:addCI(CT.eye(numConTau), startc+6)
				w:addCI0(maxTorque*www)
			end

			local clampLambda=useCase.clampLambda -- clamping lambdas independently doesn't have any physical meaning. Instead, I will try to clamp aggregated contact force
			local clampAggLambda=not clampLambda
			if clampLambda then
				local numConLambda=w.VtJ:rows()
				assert(numConLambda==totalDIM-numActualDOF*2)
				local maxLambda=useCase.maxAggLambda or 500
				maxLambda=maxLambda*0.2

				for i=0, w.bases:size()-1 do
					local b=w.bases(i)
					local lambdaWeight=CT.zeros( numConLambda)
					if not excludeNormalBasis then
						local gi=b.globalIndex
						lambdaWeight:set(gi, lambdaWeight(gi)-b.normal.y)
					end
					for j=0, b.frictionNormal:size()-1 do
						local gi=b.globalFrictionIndex+j -- -giOffset
						lambdaWeight:set(gi, lambdaWeight(gi)-b.frictionNormal(j).y)
					end

					local cmargin=useCase.contactMargin or 0.005
					local maxLambda=sop.map(b.depth*b.depth, 0, cmargin*cmargin, 0, maxLambda)

					if useCase.useRotationalFriction then
						-- force/b.area < depthDependent_maxLambda
						local MIN_AREA=3.14*0.02*0.02
						maxLambda=maxLambda*b.area/MIN_AREA
					end
					-- -lambda +maxLambda>0
					w:addCI(lambdaWeight:row(), self.numActualDOF*2)
					w:addCI0(CT.ones(1)*maxLambda)
				end
			end

			if clampAggLambda then -- clampAggLambda (clamp aggregated contact force y)
				local maxAggLambda=4500

				local maxDepth=-1
				local avgDpth=0
				local cmargin=useCase.contactMargin or 0.005
				do 
					-- calc max Depth and avgDpth
					local argMax=-1
					for i=0, w.bases:size()-1 do
						local b=w.bases(i)
						avgDpth=avgDpth+b.depth
						if b.depth>maxDepth then
							maxDepth=b.depth
							argMax=i
						end
					end
					avgDpth=avgDpth/w.bases:size()
				end
				--print(maxDepth, cmargin, avgDpth)
				--local criteria=maxDepth
				local criteria=2*avgDpth
				maxAggLambda=sop.map(criteria, 0, cmargin, 0, useCase.maxAggLambda or 500)
				if criteria>cmargin then clampAggLambda=false end			
				local numConAggLambda=0
				local agg_weight={}
				if clampAggLambda then 
					numConAggLambda=1
					local weight=agg_weight
					local axes={'y'}
					for i,a in ipairs(axes) do
						weight[a]=vectorn(cdim)
						weight[a]:zero()
					end
					for i=0, w.bases:size()-1 do
						local b=w.bases(i)
						local gi=b.globalIndex
						if not excludeNormalBasis then
							for k,a in ipairs(axes) do weight[a]:set(gi, weight[a](gi)+b.normal[a]) end
						end
						for j=0, b.frictionNormal:size()-1 do
							gi=b.globalFrictionIndex+j-giOffset
							for k,a in ipairs(axes) do weight[a]:set(gi, weight[a](gi)+b.frictionNormal(j)[a]) end
						end
					end
				end
				if numConAggLambda>0 then
					w:addCI((agg_weight.y*-1):row(), numActualDOF*2)
					w:addCI0(CT.ones(1)*maxAggLambda)
				end
			end

			if useCase.useRotationalFriction then
				--considering rotational friction and another torque
				--hwangpil
				local contactAngVel={}
				local torqueDirConstraint = useCase.useTorqueConstraint
				local torqueMaxConstraint = useCase.useTorqueConstraint

				if cdim>0 then

					--TODO:
					--need to change contactConst to selfinvfriccoef
					local contactAngVelDotProdNormal={}
					local contactVel=(w.J*dq:column()):column(0)
					local mat=nil --vector3N(w.bases:size()*2) -- draw contact angvel
					for i=0, w.bases:size()-1 do
						local v=contactVel:range(6*i, 6*i+3):copy():toVector3(0)
						contactAngVel[i]=v:dotProduct(w.bases(i).normal)

						--local bone=self.simulator:skeleton(0):VRMLbone(w.bases(i).ibone)
						--local test=self.simulator:getWorldAngVel(0, bone)
						-- tesed test==v

						if contactAngVel[i] > 0 then
							contactAngVelDotProdNormal[i]=1
						else
							contactAngVelDotProdNormal[i]=-1
						end
						if mat then
							mat(i*2):assign(w.bases(i).globalpos+vector3(0,0.1,0))
							mat(i*2+1):assign(w.bases(i).globalpos+vector3(0,0.1,0)+v)
							--dbg.console()
						end
					end
					if mat then dbg.draw('Traj', mat:matView()*100, "contacts") end

					if w.bases:size()>0 and w.bases(0).globalTorqueIndex then
						if torqueDirConstraint then
							-- add torque direction constraint
							for i=0, w.bases:size()-1 do
								local b=w.bases(i) 
								--local gi=numActualDOF*2+b.globalIndex+w.bases:size()*(b.frictionNormal:size()+1)
								local gi=numActualDOF*2+b.globalTorqueIndex
								-- w*tau < 0 -- meaning torque and angvel has different direction
								-- -w*tau > 0
								w:addCI(CT.ones(1,1)*-1*contactAngVelDotProdNormal[i], gi)
								w:addCI0(CT.zeros(1))
							end
						end

						if torqueMaxConstraint and useCase.useContactCentroid then
							-- add torque maximum constraint
							if not self.state.prevContactY then self.state.prevContactY=0 end
							for i=0, w.bases:size()-1 do
								local b=w.bases(i)
								local contactTorqueConst = 8*math.sqrt(b.area/3.141592)/15

								local v=CT.zeros(w.CI:cols())
								--calculate angular velocities of each contact points
								--caution : this code assumes that each convex hull has only one contact point.
								--
								-- torque <= uN*8/15*sqrt(A/pi) = uNR
								-- [Matei Ciocarlie et al. 2007]
								-- R : contactTorqueConst
								-- u : contactConst
								-- N=self.state.prevContactY
								local contactConst = 0.8
								local maxTorque=contactConst*contactTorqueConst*
								(self.state.prevContactY/w.bases:size())
								--only consider rotational friction
								local gi=numActualDOF*2+b.globalTorqueIndex
								-- -w*tau < maxTorque
								-- w*tau > -maxTorque
								w:addCI(CT.ones(1,1)*contactAngVelDotProdNormal[i], gi)
								w:addCI0(CT.ones(1)*(-maxTorque))

								if not torqueDirConstraint then
									-- -w*tau > -maxTorque
									w:addCI(CT.ones(1,1)*-1*contactAngVelDotProdNormal[i], gi)
									w:addCI0(CT.ones(1)*(-maxTorque))
								end
							end
						end

						if useCase.useThreeDimTorque then
							-- add torque maximum constraint for another torque
							for i=0, w.bases:size()-1 do
								local b=w.bases(i)
								--approx contact circle to square
								local contactTorqueConst = math.sqrt(b.area/(2*3.141592))
								for j=1, b.torqueNormal:size()-1 do
									--consider torques except for rotational friction torque
									--   -Rf/sqrt(2) <= torque of one basis <= Rf/sqrt(2)
									local v1=CT.zeros(w.CI:cols())
									local v2=CT.zeros(w.CI:cols())

									local gi=numActualDOF*2+b.globalIndex
									v1:set(gi, contactTorqueConst)
									v2:set(gi, contactTorqueConst)
									for k=0, b.frictionNormal:size()-1 do
										local gi=numActualDOF*2+b.globalFrictionIndex+k-giOffset
										v1:set( gi, contactTorqueConst*b.frictionNormal(k):dotProduct(b.normal))
										v2:set( gi, contactTorqueConst*b.frictionNormal(k):dotProduct(b.normal))
									end
									gi=numActualDOF*2+b.globalTorqueIndex+j
									v1:set(gi, 1)  -- Rf+torque
									v2:set(gi, -1) -- Rf-torque

									-- Rf+torque>0
									w:addCI(v1:row(), 0)
									w:addCI0(CT.zeros(1))
									-- Rf-torque>0
									w:addCI(v2:row(), 0)
									w:addCI0(CT.zeros(1))
								end
							end
						end
					end
				end
			end
			--/hwangpil

				
			if false then
				-- add velocity-dependent margin for a single point which maximally penetrates.

				local maxDepth=-1
				local argMax=-1
				for i=0, w.bases:size()-1 do
					local b=w.bases(i)
					if b.depth>maxDepth then
						maxDepth=b.depth
						argMax=i
					end
				end
				local velMarginStrengthO=useCase.velMarginStrength or 1.0
				local b=w.bases(argMax)
				--local relvel=b.relvel:copy()+(useCase.maxPenetratingVel or 0)*b.normal
				-- add velocity-dependent margin
				local velMarginOffset=useCase.velMarginOffset or 18
				local maxPenetratingVel=useCase.maxPenetratingVel or 0
				for i=0, w.bases:size()-1 do
					local b=w.bases(i)
					local relvel=b.relvel:copy()+maxPenetratingVel *b.normal
					local velMarginStrength=velMarginStrengthO
					local offset=0
					if i~=argMax then 
						--velMarginStrength=0 
						offset=velMarginOffset-- turn off acceleration cone bound (friction cones still are turned on)
					else
						--if relvel.y>0.01 then relvel:scale(0.01/relvel.y) end
					end
					local dp=b.normal:dotProduct(relvel)
					local projectionOfRelvel =dp*self.dtinv*velMarginStrength
					local gi=b.globalIndex
					if not excludeNormalBasis then
						w.ci0:set(gi, w.ci0(gi)+projectionOfRelvel+offset)
					end
					for j=0, b.frictionNormal:size()-1 do
						projectionOfRelvel=b.frictionNormal(j):dotProduct(relvel)*self.dtinv*velMarginStrength
						gi=b.globalFrictionIndex+j-giOffset
						w.ci0:set(gi, w.ci0(gi)+projectionOfRelvel+offset)
					end
				end
			elseif (not useCase.useSoftContactModel) or useCase.useSoftContactModel_old then
				-- cmargin dependent
				local cmargin=useCase.contactMargin or 0.005
				local velMarginStrength=useCase.velMarginStrength or 1.0
				local maxPenetratingVel=useCase.maxPenetratingVel or 0
				local dtinv=self.dtinv
				-- add velocity-dependent margin
				for i=0, w.accbases:size()-1 do
					local b=w.accbases(i)
					local dp=b.normal:dotProduct(b.relvel)

					local cmargin_dep_max_penetrating_vel
					if b.depth>cmargin then
						--cmargin_dep_max_penetrating_vel =maxPenetratingVel 
						cmargin_dep_max_penetrating_vel =maxPenetratingVel +sop.clampMap(b.depth, cmargin, cmargin*1.5, cmargin*dtinv, 0)
					else
						--cmargin_dep_max_penetrating_vel =maxPenetratingVel + sop.clampMap(b.depth ,0, cmargin, cmargin*dtinv, 0) 
						cmargin_dep_max_penetrating_vel =maxPenetratingVel + cmargin*dtinv 
					end
					local relvel=b.relvel:copy()+cmargin_dep_max_penetrating_vel*b.normal
					local projectionOfRelvel=b.normal:dotProduct(relvel)*dtinv
					local gi=b.globalIndex
					if not excludeNormalBasis then
						w.ci0:set(gi, w.ci0(gi)+projectionOfRelvel)
					end
					for j=0, b.frictionNormal:size()-1 do
						--print(b.normal, b.frictionNormal(j))
						--projectionOfRelvel=(b.frictionNormal(j):dotProduct(relvel)+maxPenetratingVel)*dtinv*velMarginStrength -- allows slight foot slipping
						projectionOfRelvel=b.frictionNormal(j):dotProduct(relvel)*dtinv*velMarginStrength -- do not allow foot slipping
						local gi=b.globalFrictionIndex+j-giOffset
						w.ci0:set(gi, w.ci0(gi)+projectionOfRelvel)
					end
				end
				--hwangpil
			elseif useEqualityVelConstraint then
				-- cmargin dependent
				local cmargin=useCase.contactMargin or 0.005
				local velMarginStrength=useCase.velMarginStrength or 1.0
				local maxPenetratingVel=useCase.maxPenetratingVel or 0
				local dtinv=self.dtinv
				-- add velocity-dependent margin
				for i=0, w.accbases:size()-1 do
					local b=w.accbases(i)
					local dp=b.normal:dotProduct(b.relvel)
					local cmargin_dep_max_penetrating_vel
					if b.depth>cmargin then
						--cmargin_dep_max_penetrating_vel =maxPenetratingVel 
						cmargin_dep_max_penetrating_vel =maxPenetratingVel +sop.clampMap(b.depth, cmargin, cmargin*1.5, cmargin*dtinv, 0)
					else
						--cmargin_dep_max_penetrating_vel =maxPenetratingVel + sop.clampMap(b.depth ,0, cmargin, cmargin*dtinv, 0) 
						cmargin_dep_max_penetrating_vel =maxPenetratingVel + cmargin*dtinv 
					end
					local relvel=b.relvel:copy()+cmargin_dep_max_penetrating_vel*b.normal
					local projectionOfRelvel=b.normal:dotProduct(relvel)*dtinv
					local gi=b.globalIndex
					if not excludeNormalBasis then
						w.ci0:set(gi, w.ci0(gi)+projectionOfRelvel)
					end
					for j=0, b.frictionNormal:size()-1 do
						--projectionOfRelvel=(b.frictionNormal(j):dotProduct(relvel)+maxPenetratingVel)*dtinv*velMarginStrength -- allows slight foot slipping
						projectionOfRelvel=b.frictionNormal[j]:dotProduct(relvel)*dtinv*velMarginStrength -- do not allow foot slipping
						local gi=b.globalFrictionIndex+j-giOffset
						w.ci0:set(gi, w.ci0(gi)+projectionOfRelvel)
					end
					gi=gi+w.bases:size()*(b.frictionNormal:size()+1)
				end 
				--hwangpil
			elseif true then
				local rotFric=useCase.useRotationalFriction

				-- soft contact model
				local cmargin=useCase.contactMargin or 0.005
				local maxDepth=useCase.maxDepth or 0.016
				local dtinv=self.dtinv
				
				if not rotFric then -- taesoo's setting. please do not change
					cmargin=0
					maxDepth=useCase.contactMargin
				end

				for i=0, w.accbases:size()-1 do
					local b=w.accbases(i)
					local dp=b.normal:dotProduct(b.relvel)
					local depthMargin=b.depth-maxDepth

					local cmargin_dep_max_penetrating_vel

					if depthMargin >= 0 then
						--cmargin_dep_max_penetrating_vel = -1*depthMargin*dtinv
						cmargin_dep_max_penetrating_vel = 0
					elseif b.depth>cmargin then
						--elseif true then
						cmargin_dep_max_penetrating_vel = -1*depthMargin*dtinv
						--cmargin_dep_max_penetrating_vel = sop.clampMap(b.depth, cmargin, cmargin*1.5, cmargin*dtinv, 0)
					else
						cmargin_dep_max_penetrating_vel = cmargin*dtinv
					end

					local relvel = b.relvel:copy() + cmargin_dep_max_penetrating_vel*b.normal
					local projectionOfRelvel=b.normal:dotProduct(relvel)*dtinv
					local gi=b.globalIndex
					if not excludeNormalBasis then
						w.ci0:set(gi, w.ci0(gi)+projectionOfRelvel)
					end
					for j=0, b.frictionNormal:size()-1 do
						projectionOfRelvel=b.frictionNormal(j):dotProduct(relvel)*dtinv
						local gi=b.globalFrictionIndex+j-giOffset
						w.ci0:set(gi, w.ci0(gi)+projectionOfRelvel)
					end
				end
				--/hwangpil
			else
				local velMarginStrengthO=useCase.velMarginStrength or 1.0
				-- add velocity-dependent margin
				for i=0, w.bases:size()-1 do
					local b=w.bases(i)
					local cduration= state.contactDuration[b.ilinkpair+1]
					--local velMarginStrength=sop.clampMap(cduration, 0,15, 0,1)
					--local velMarginStrength=sop.clampMap(b.depth, 0.015, 0, 0.7,0.2)
					--b.relvel.x=b.relvel.x*0.7
					--b.relvel.y=b.relvel.y*velMarginStrength
					--b.relvel.z=b.relvel.z*0.7
					local relvel=b.relvel:copy()+maxPenetratingVel *b.normal
					local velMarginStrength=velMarginStrengthO
					--if relvel.y>0 then velMarginStrength=0 end
					local dp=b.normal:dotProduct(relvel)
					local projectionOfRelvel =dp*self.dtinv*velMarginStrength
					--local projectionOfRelvel=b.normal:dotProduct(relvel)*self.dtinv
					local gi=b.globalIndex
					if not excludeNormalBasis then
						w.ci0:set(gi, w.ci0(gi)+projectionOfRelvel)
					end
					for j=0, b.frictionNormal:size()-1 do
						projectionOfRelvel=b.frictionNormal(j):dotProduct(relvel)*self.dtinv*velMarginStrength
						gi=b.globalFrictionIndex+j-giOffset
						w.ci0:set(gi, w.ci0(gi)+projectionOfRelvel)
					end
				end
			end

			if useCase.useInequalityVelConstraint then
				-- not allow escaping ground during single support phase
				-- VtJddq + (invdtVtJ + dotVtJ+VtdotJ)dq = 0
				local maxdepthForRoll={0, 0, 0, 0,0,0}
				local maxidxForRoll={0, 0, 0, 0,0,0}
				local isContact={false, false, false, false,false, false}
				local maxdepthBone=nil

				local skel=self.dofInfo:skeleton()
				local lfoot=skel:getTreeIndexByVoca(MotionLoader.LEFTANKLE)
				local ltoes=skel:getTreeIndexByVoca(MotionLoader.LEFTTOES)
				local rfoot=skel:getTreeIndexByVoca(MotionLoader.RIGHTANKLE)
				local rtoes=skel:getTreeIndexByVoca(MotionLoader.RIGHTTOES)
				local lhand=skel:getTreeIndexByVoca(MotionLoader.LEFTWRIST)
				local rhand=skel:getTreeIndexByVoca(MotionLoader.RIGHTWRIST)

				-- Left heel ,  Right heel, left toe,right toe, Left hand, Right hand, others, respectively
				local limbs={L=1, R=2, LT=3, RT=4, LH=5, RH=6, O=7}
				local limbsinv={'L', 'R', 'LT', 'RT', 'LH', 'RH', 'O'}
				local limbsToTreeIndex={lfoot, rfoot, ltoes, rtoes, lhand, rhand}
				local contactBone={}
				local availableEqualityBone={false, false,false, false, false, false}

				if spprtImportance.L==1 then
					availableEqualityBone[limbs.L]=true
					availableEqualityBone[limbs.LT]=true
				elseif spprtImportance.R==1 then
					availableEqualityBone[limbs.R]=true
					availableEqualityBone[limbs.RT]=true
				elseif spprtImportance.LH==1 then
					availableEqualityBone[limbs.LH]=true
				elseif spprtImportance.RH==1 then
					availableEqualityBone[limbs.RH]=true
				end

				RE.output2('equalityBone', table.tostring2(availableEqualityBone))
				for i=0, w.accbases:size()-1 do
					local b=w.accbases(i)
					local limb=limbs.O
					if b.ibone==lfoot then
						limb=limbs.L
					elseif b.ibone==rfoot then
						limb=limbs.R
					elseif b.ibone==ltoes then
						limb=limbs.LT
					elseif b.ibone==rtoes then
						limb=limbs.RT
					elseif b.ibone==lhand then
						limb=limbs.LH
					elseif b.ibone==rhand then
						limb=limbs.RH
					end

					if limb~=limbs.O and availableEqualityBone[limb] and w.accbases(i).depth > maxdepthForRoll[limb] then
						maxdepthBone = limb
						maxidxForRoll[limb] = i
						maxdepthForRoll[limb] = w.accbases(i).depth
						if not isContact[limb] then
							table.insert(contactBone,limb)
							isContact[limb]=true
						end
					end
				end
				
				--local contactbone=simulator:skeleton(0):getTreeIndexByName('lfoot')
				local VtJsub=matrixn()
				local dotVtJsub = matrixn()
				local VtdotJsub = matrixn()
				local depthvec = vectorn()
				local dq=w.dq
				local dtinv=self.dtinv

				if #contactBone >0 then
					local numBasis = cdim/w.bases:size() 
					VtJsub:setSize(#contactBone*numBasis, numActualDOF)
					dotVtJsub:setSize(#contactBone*numBasis, numActualDOF)
					VtdotJsub:setSize(#contactBone*numBasis, numActualDOF)
					depthvec:setSize(#contactBone*numBasis)
					VtJsub:setAllValue(0)
					dotVtJsub:setAllValue(0)
					VtdotJsub:setAllValue(0)
					depthvec:setAllValue(0)
					local VtJsubidx=0
					for i, limb in ipairs(contactBone) do
						local idx = maxidxForRoll[limb]
						local b=w.accbases(idx)
						local gi=b.globalIndex
						VtJsub:sub(VtJsubidx, VtJsubidx+1, 0, numActualDOF):assign(w.VtJ:sub(gi, gi+1, 0, numActualDOF):copy())
						dotVtJsub:sub(VtJsubidx, VtJsubidx+1, 0, numActualDOF):assign(w.dotVtJ:sub(gi, gi+1, 0, numActualDOF):copy())
						VtdotJsub:sub(VtJsubidx, VtJsubidx+1, 0, numActualDOF):assign(w.VtDotJ:sub(gi, gi+1, 0, numActualDOF):copy())
						VtJsubidx=VtJsubidx+1
						for j=0, b.frictionNormal:size()-1 do
							local gi=b.globalFrictionIndex+j-giOffset
							VtJsub:sub(VtJsubidx, VtJsubidx+1, 0, numActualDOF):assign(w.VtJ:sub(gi, gi+1, 0, numActualDOF):copy())
							dotVtJsub:sub(VtJsubidx, VtJsubidx+1, 0, numActualDOF):assign(w.dotVtJ:sub(gi, gi+1, 0, numActualDOF):copy())
							VtdotJsub:sub(VtJsubidx, VtJsubidx+1, 0, numActualDOF):assign(w.VtDotJ:sub(gi, gi+1, 0, numActualDOF):copy())
							VtJsubidx=VtJsubidx+1
						end
						for j=0, b.torqueNormal:size()-1 do
							local gi=b.globalIndex+w.accbases:size()*(b.frictionNormal:size()+1)
							VtJsub:sub(VtJsubidx, VtJsubidx+1, 0, numActualDOF):assign(w.VtJ:sub(gi, gi+1, 0, numActualDOF):copy())
							dotVtJsub:sub(VtJsubidx, VtJsubidx+1, 0, numActualDOF):assign(w.dotVtJ:sub(gi, gi+1, 0, numActualDOF):copy())
							VtdotJsub:sub(VtJsubidx, VtJsubidx+1, 0, numActualDOF):assign(w.VtDotJ:sub(gi, gi+1, 0, numActualDOF):copy())
							VtJsubidx=VtJsubidx+1
						end
					end
					VtJsub:rmult(-1)
					dotVtJsub:rmult(-1)
					VtdotJsub:rmult(-1)
					local tempVtJsub=VtJsub:copy()
					tempVtJsub:rmult(dtinv)
					depthvec:assign(((tempVtJsub+dotVtJsub+VtdotJsub)*dq:column()):column(0))
					for i, limb in ipairs(contactBone) do
						local idx = maxidxForRoll[limb]
						local b=w.accbases(idx)
						local gi = (i-1)*numBasis
						local dp=b.normal:dotProduct(b.relvel)
						local cmargin_dep_max_penetrating_vel=-1*b.depth*dtinv

						local relvel = b.relvel:copy() + cmargin_dep_max_penetrating_vel*b.normal
						local projectionOfRelvel=b.normal:dotProduct(relvel)*dtinv
						local gi=b.globalIndex
						if not excludeNormalBasis then
							depthvec:set(gi, depthvec(gi)+projectionOfRelvel)
						end
						for j=0, b.frictionNormal:size()-1 do
							projectionOfRelvel=b.frictionNormal(j):dotProduct(relvel)*dtinv
							local gi=b.globalFrictionIndex+j-giOffset
							depthvec:set(gi, depthvec(gi)+projectionOfRelvel)
						end
					end
				end

				if #contactBone >0 then
					local oldCIrows = w.CI:rows()
					w.CI:resize(oldCIrows+VtJsub:rows(),totalDIM) 
					w.CI:sub(oldCIrows, oldCIrows+VtJsub:rows(), 0, totalDIM):setAllValue(0)
					w.CI:sub(oldCIrows, oldCIrows+VtJsub:rows(), 0, numActualDOF):assign(VtJsub)

					w.ci0:resize(oldCIrows+VtJsub:rows())
					w.ci0:range(oldCIrows, oldCIrows+VtJsub:rows()):assign(depthvec)
				end
			end


			-- minimize complementory cost (minimize (lambda *  (a*ddq+b)))
			--qp:addComplementory3(500,totalDIM, numActualDOF*2,numActualDOF*2+cdim, w.CI:sub(0, w.VtJ:rows(), 0, 0), w.ci0:range(0, w.VtJ:rows()):column())
			--self:addCOMobjective(simulator, vector3(-100,0,1))
		end
		-- set equality constraints  
		--
		-- w.M*ddq - tau - JtV* lambda = w.b
		--> (w.M -I -JtV)(ddq;tau;lambda) = w.b
		--
		local reftime=self.state.reftime
		if not useCase.useEqualityVelConstraint then
			w.CE:setSize(numActualDOF+6, totalDIM)
			w.CE:sub(0,numActualDOF,0,numActualDOF):assign(w.M)
			local minusI=w.CE:sub(0,numActualDOF,numActualDOF,numActualDOF*2)
			minusI:identity()
			minusI:rmult(-1)
			local minusJtV=w.CE:sub(0,numActualDOF, numActualDOF*2, totalDIM)
			minusJtV:assign(w.JtV)
			minusJtV:rmult(-1)
			-- constrain tau[0:6]=0
			w.CE:sub(numActualDOF, numActualDOF+6):setAllValue(0)
			w.CE:sub(numActualDOF, numActualDOF+6, numActualDOF, numActualDOF+6):identity()
			w.ce0:setSize(numActualDOF+6)
			w.ce0:range(0,numActualDOF):assign(w.b)
			w.ce0:range(numActualDOF,numActualDOF+6):setAllValue(0)
		elseif true then
			-- VtJddq + (invdtVtJ + dotVtJ+VtdotJ)dq = 0
			local maxdepthForRoll={0, 0, 0, 0,0,0}
			local maxidxForRoll={0, 0, 0, 0,0,0}
			local isContact={false, false, false, false,false, false}
			local maxdepthBone=nil

			local skel=self.dofInfo:skeleton()
			local lfoot=skel:getTreeIndexByVoca(MotionLoader.LEFTANKLE)
			local ltoes=skel:getTreeIndexByVoca(MotionLoader.LEFTTOES)
			local rfoot=skel:getTreeIndexByVoca(MotionLoader.RIGHTANKLE)
			local rtoes=skel:getTreeIndexByVoca(MotionLoader.RIGHTTOES)
			local lhand=skel:getTreeIndexByVoca(MotionLoader.LEFTWRIST)
			local rhand=skel:getTreeIndexByVoca(MotionLoader.RIGHTWRIST)

			-- Left heel ,  Right heel, left toe,right toe, Left hand, Right hand, others, respectively
			local limbs={L=1, R=2, LT=3, RT=4, LH=5, RH=6, O=7}
			local limbsinv={'L', 'R', 'LT', 'RT', 'LH', 'RH', 'O'}
			local limbsToTreeIndex={lfoot, rfoot, ltoes, rtoes, lhand, rhand}
			local contactBone={}
			local availableEqualityBone={false, false,false, false, false, false}
			
			if reftime then
				local reftimeInOnePeriod=reftime-math.floor(reftime)
				if spprtImportance.L==1 then
					if reftimeInOnePeriod <0.5 then
						availableEqualityBone[limbs.L]=true
					else
						availableEqualityBone[limbs.LT]=true
					end
				elseif spprtImportance.R==1 then
					if reftimeInOnePeriod <0.5 then
						availableEqualityBone[limbs.R]=true
					else
						availableEqualityBone[limbs.RT]=true
					end
				elseif spprtImportance.LH==1 then
					availableEqualityBone[limbs.LH]=true
				elseif spprtImportance.RH==1 then
					availableEqualityBone[limbs.RH]=true
				end
			end

			RE.output2('equalityBone', table.tostring2(availableEqualityBone))
			for i=0, w.accbases:size()-1 do
				local b=w.accbases(i)
				local limb=limbs.O
				if b.ibone==lfoot then
					limb=limbs.L
				elseif b.ibone==rfoot then
					limb=limbs.R
				elseif b.ibone==ltoes then
					limb=limbs.LT
				elseif b.ibone==rtoes then
					limb=limbs.RT
				elseif b.ibone==lhand then
					limb=limbs.LH
				elseif b.ibone==rhand then
					limb=limbs.RH
				end

				if limb~=limbs.O and availableEqualityBone[limb] and w.accbases(i).depth > maxdepthForRoll[limb] then
					maxdepthBone = limb
					maxidxForRoll[limb] = i
					maxdepthForRoll[limb] = w.accbases(i).depth
					if not isContact[limb] then
						table.insert(contactBone,limb)
						isContact[limb]=true
					end
				end
			end
			
			--local contactbone=simulator:skeleton(0):getTreeIndexByName('lfoot')
			local VtJsub=matrixn()
			local dotVtJsub = matrixn()
			local VtdotJsub = matrixn()
			local dq=w.dq
			local dtinv=self.dtinv

			----[==[
			if #contactBone >0 then
				local numBasis = cdim/w.bases:size() 
				VtJsub:setSize(#contactBone*numBasis, numActualDOF)
				dotVtJsub:setSize(#contactBone*numBasis, numActualDOF)
				VtdotJsub:setSize(#contactBone*numBasis, numActualDOF)
				VtJsub:setAllValue(0)
				if useCase.useRotationalFriction then
					dotVtJsub:setAllValue(0)
					VtdotJsub:setAllValue(0)
					local VtJsubidx = 0
					for i, limb in ipairs(contactBone) do
						local idx = maxidxForRoll[limb]
						local b=w.accbases(idx)
						local gi=b.globalIndex
						VtJsub:sub(VtJsubidx, VtJsubidx+1, 0, numActualDOF):assign(w.VtJ:sub(gi, gi+1, 0, numActualDOF):copy())
						dotVtJsub:sub(VtJsubidx, VtJsubidx+1, 0, numActualDOF):assign(w.dotVtJ:sub(gi, gi+1, 0, numActualDOF):copy())
						VtdotJsub:sub(VtJsubidx, VtJsubidx+1, 0, numActualDOF):assign(w.VtDotJ:sub(gi, gi+1, 0, numActualDOF):copy())
						VtJsubidx=VtJsubidx+1
						for j=0, b.frictionNormal:size()-1 do
							local gi=b.globalFrictionIndex+j-giOffset
							VtJsub:sub(VtJsubidx, VtJsubidx+1, 0, numActualDOF):assign(w.VtJ:sub(gi, gi+1, 0, numActualDOF):copy())
							dotVtJsub:sub(VtJsubidx, VtJsubidx+1, 0, numActualDOF):assign(w.dotVtJ:sub(gi, gi+1, 0, numActualDOF):copy())
							VtdotJsub:sub(VtJsubidx, VtJsubidx+1, 0, numActualDOF):assign(w.VtDotJ:sub(gi, gi+1, 0, numActualDOF):copy())
							VtJsubidx=VtJsubidx+1
						end
						for j=0, b.torqueNormal:size()-1 do
							gi=b.globalIndex+w.accbases:size()*(b.frictionNormal:size()+1)
							VtJsub:sub(VtJsubidx, VtJsubidx+1, 0, numActualDOF):assign(w.VtJ:sub(gi, gi+1, 0, numActualDOF):copy())
							dotVtJsub:sub(VtJsubidx, VtJsubidx+1, 0, numActualDOF):assign(w.dotVtJ:sub(gi, gi+1, 0, numActualDOF):copy())
							VtdotJsub:sub(VtJsubidx, VtJsubidx+1, 0, numActualDOF):assign(w.VtDotJ:sub(gi, gi+1, 0, numActualDOF):copy())
							VtJsubidx=VtJsubidx+1
						end
					end
				else
					for i, limb in ipairs(contactBone) do
						VtJsub:sub((i-1)*numBasis, i*numBasis, 0, numActualDOF):assign(w.VtJ:sub(numBasis*maxidxForRoll[limb], numBasis*(maxidxForRoll[limb]+1), 0, numActualDOF):copy())
						dotVtJsub:sub((i-1)*numBasis, i*numBasis, 0, numActualDOF):assign(w.VtJ:sub(numBasis*maxidxForRoll[limb], numBasis*(maxidxForRoll[limb]+1), 0, numActualDOF):copy())
						VtdotJsub:sub((i-1)*numBasis, i*numBasis, 0, numActualDOF):assign(w.VtJ:sub(numBasis*maxidxForRoll[limb], numBasis*(maxidxForRoll[limb]+1), 0, numActualDOF):copy())
					end		
				end
			else
				VtJsub:setSize(0, numActualDOF)
			end
			--]==]
			
			--[==[
			--only one bone for equality constraint
			if maxdepthBone then
				local numBasis = cdim/w.bases:size() 
				VtJsub:setSize(numBasis, numActualDOF)
				dotVtJsub:setSize(numBasis, numActualDOF)
				VtdotJsub:setSize(numBasis, numActualDOF)
				VtJsub:setAllValue(0)
				local limb = maxdepthBone
				VtJsub:sub(0, numBasis, 0, numActualDOF):assign(w.VtJ:sub(6*maxidxForRoll[limb], 6*(maxidxForRoll[limb]+1), 0, numActualDOF):copy())
				dotVtJsub:sub(0, numBasis, 0, numActualDOF):assign(w.VtJ:sub(6*maxidxForRoll[limb], 6*(maxidxForRoll[limb]+1), 0, numActualDOF):copy())
				VtdotJsub:sub(0, numBasis, 0, numActualDOF):assign(w.VtJ:sub(6*maxidxForRoll[limb], 6*(maxidxForRoll[limb]+1), 0, numActualDOF):copy())
			else
				VtJsub:setSize(0, numActualDOF)
			end
			--]==]

			w.CE:setSize(numActualDOF+6+VtJsub:rows(), totalDIM) -- VtJsub:rows() : for deepest point constraint
			w.CE:sub(0,numActualDOF,0,numActualDOF):assign(w.M)
			local minusI=w.CE:sub(0,numActualDOF,numActualDOF,numActualDOF*2)
			minusI:identity()
			minusI:rmult(-1)
			local minusJtV=w.CE:sub(0,numActualDOF, numActualDOF*2, totalDIM)
			minusJtV:assign(w.JtV)
			minusJtV:rmult(-1)
			-- constrain tau[0:6]=0
			w.CE:sub(numActualDOF, numActualDOF+6):setAllValue(0)
			w.CE:sub(numActualDOF, numActualDOF+6, numActualDOF, numActualDOF+6):identity()

			w.ce0:setSize(numActualDOF+6+VtJsub:rows())
			w.ce0:range(0,numActualDOF):assign(w.b)
			w.ce0:range(numActualDOF,numActualDOF+6):setAllValue(0)

			if #contactBone >0 then
				w.CE:sub(numActualDOF+6, numActualDOF+6+VtJsub:rows()):setAllValue(0)
				w.CE:sub(numActualDOF+6, numActualDOF+6+VtJsub:rows(), 0, numActualDOF):assign(VtJsub)

				VtJsub:rmult(dtinv)
				w.ce0:range(numActualDOF+6, numActualDOF+6+VtJsub:rows()):assign(((VtJsub+dotVtJsub+VtdotJsub)*dq:column()):column(0))
			end

			--resize inequality constraint matrix
			if #contactBone > 0 then
				local CI = matrixn()
				local ci0 = vectorn()
				local numBasis = cdim/w.bases:size()
				local cirowsize =w.CI:rows()-numBasis*#contactBone
				local oldcirowsize = w.CI:rows()
				CI:setSize(cirowsize, totalDIM) 
				CI:setAllValue(0)
				ci0:setSize(cirowsize)
				ci0:setAllValue(0)
				local idxCI=0
				for i=0, w.bases:size()-1 do
					local b=w.bases(i)
					local isequality = false
					for j,limb in ipairs(contactBone) do
						if limbsToTreeIndex[limb] == b.ibone then
							--print(limb)
							isequality = true
						end
					end
					if not isequality then
						CI:sub(idxCI, idxCI+numBasis, 0, totalDIM):assign(w.CI:sub(i*numBasis, (i+1)*numBasis, 0, totalDIM):copy())
						ci0:range(idxCI, idxCI+numBasis):assign(w.ci0:range(idxCI, idxCI+numBasis):copy())
						idxCI=idxCI+numBasis
					end
				end
				--print(idxCI, cirowsize, w.VtJ:rows(), oldcirowsize)
				CI:sub(idxCI,cirowsize,0, totalDIM):assign(w.CI:sub(w.VtJ:rows(), oldcirowsize, 0, totalDIM):copy())
				ci0:range(idxCI,cirowsize):assign(w.ci0:range(w.VtJ:rows(), oldcirowsize):copy())
				w.CI:assign(CI:copy())
				w.ci0:assign(ci0:copy())
			end
		end
		--]]

		--##dos if g_debugOneStep then --##dos g_debugOneStep:pushBack({qp.H:copy(), qp.R:copy()}) --##dos end
		-- print(self.controlforce)
		-- self:addPDtorque(simulator)
		-- dbg.console()
		
	else
		local totalDIM=numActualDOF*2 -- ddq and tau
		local qp=HessianQuadratic(totalDIM)
		self.qp=qp
		-- dbg.console()
		-- minimize desired acc error
		local ddqObjWeight_flight=useCase.ddqObjWeight_flight or 1000
		local weight=self.weight
		if self.excludeRootFlight then --self.excludeRoot then
			local w=1
			for i=0,3 do -- root
				qp:addD(w,i,self.desiredacceleration(i+4))
			end
			for i=3,6 do -- root
				qp:addD(w,i,self.desiredacceleration(i-3))
			end
		else
			for i=0,3 do -- root
				qp:addD(10000,i,self.desiredacceleration(i+4))
			end
			for i=3,6 do -- root
				qp:addD(10000,i,self.desiredacceleration(i-3))
			end
		end

		local ddqObjWeight_flight=useCase.ddqObjWeight_flight or 1000
		local weight=self.weight
		for i=6,numActualDOF-1 do
			qp:addD(ddqObjWeight_flight*weight(i+1),i,self.desiredacceleration(i+1))
		end
		for i=0,numActualDOF-1 do
			qp:addD(1,i+numActualDOF,0)
		end
		w.CE:setSize(numActualDOF+6, numActualDOF*2)
		w.CE:sub(0,numActualDOF,0,numActualDOF):assign(w.M)
		local minusI=w.CE:sub(0,numActualDOF,numActualDOF,numActualDOF*2)
		minusI:identity()
		minusI:rmult(-1)
		-- constrain tau[0:6]=0
		w.CE:sub(numActualDOF, numActualDOF+6):setAllValue(0)
		w.CE:sub(numActualDOF, numActualDOF+6, numActualDOF, numActualDOF+6):identity()
		w.ce0:setSize(numActualDOF+6)
		w.ce0:range(0,numActualDOF):assign(w.b)
		w.ce0:range(numActualDOF,numActualDOF+6):setAllValue(0)
		w.CI:setSize(0,0)
		w.ci0:setSize(0)

		--print('desiredacc', self.desiredacceleration)
		--print(w.ce0, w.b, w.ci0)
		--this("exit",1)
	end
	if true then
		-- clamp knee angle
		-- acceleration bounds
		-- acc > minAcc
		local CI_cols=w.CI:cols()
		if w.CI:rows()==0 then
			CI_cols=numActualDOF*2
		end
		local angleBound=math.rad(useCase.angleBound or -2)
		local minAcc=0
		local maxAcc=100
		--RE.output2('kneeDOF', self.theta(self.lkneeDOF), self.theta(self.rkneeDOF), self.dtheta(self.lkneeDOF), self.dtheta(self.rkneeDOF))

		local knees={ self.lkneeDOF, self.rkneeDOF, self.lelbowDOF, self.relbowDOF }
		local kneeAxes={1 ,1, -1, -1}
		for iknee,idof in ipairs(knees) do
			local axis=kneeAxes[iknee]
			if axis*self.theta(idof)<angleBound then
				local idq=self.dofInfo:DOFtoDQ(idof)
				w.CI:resize(w.CI:rows()+1, CI_cols)
				local ci0=w.ci0
				ci0:resize(ci0:size()+1)
				local lastRow=w.CI:row(w.CI:rows()-1)
				-- acc- minAcc>0
				lastRow:setAllValue(0)
				lastRow:set(idq,axis)
				-- 0.5a*t^2=angleBound-self.theta(idof) 인 a찾자. 
				--minAcc=2*(angleBound-self.theta(idof))/(0.1*0.1)
				local vel=math.min(axis*self.dtheta(idof),0)
				minAcc=-vel*self.dtinv
				ci0:set(ci0:size()-1, (minAcc)*-1)
			end
		end

		--local www=self.weight2:range(7,numActualDOF+1)
		--w.ci0:range(start, start+numConTau):assign(maxTorque*www)
		---- tau+maxTorque>0
		--w.CI:sub(start+numConTau, start+numConTau*2, startc+6, startc+numActualDOF):identity()
		----w.ci0:range(start+numConTau, start+numConTau*2):setAllValue(maxTorque)
		--w.ci0:range(start+numConTau, start+numConTau*2):assign(maxTorque*www)
	end

	--_checkpoints:pushBack(deepCopyTable({'ce',w.CE, w.ce0}))
	local state=self.state
	local perClassContactMargin=useCase.perClassContactMargin==1 
	if not perClassContactMargin then -- dynamic adjustment of collision margin
		local cmargin=useCase.contactMargin or 0.005
		for i=1, self.numCLinks do
			if state.prevContact[i] and not state.contact[i] then
				simulator:setCollisionMargin(i-1, 0) 
				--RE.output2('contactmargin'..i,'0')
				state.contactDuration[i]=0
			elseif not state.prevContact[i] and state.contact[i] then
				simulator:setCollisionMargin(i-1, cmargin) -- allow some penetration
				--RE.output2('contactmargin'..i,'1')
				--simulator:setCollisionMargin(i-1, 0.013) -- allow some penetration
				state.contactDuration[i]=0
			elseif state.contact[i] then
				state.contactDuration[i]=state.contactDuration[i]+1
			end
		end
	else
		-- per class
		local prevContact={L=false,R=false,LH=false,RH=false,O=false}
		local contact={L=false,R=false,LH=false,RH=false,O=false}
		local targets={L={},R={}, LH={}, RH={}, O={}}
		for i=1, self.numCLinks do
			local cls=state.contactClass[i]
			prevContact[cls]=prevContact[cls] or state.prevContact[i]
			contact[cls]=contact[cls] or state.contact[i]
			array.pushBack(targets[cls],i)
		end
		RE.output2('prevContact', prevContact.L, prevContact.R)
		RE.output2('Contact', contact.L, contact.R)
		RE.output2('ContactState',state.contact[1], state.contact[2], state.contact[3], state.contact[4]
		,state.contact[5], state.contact[6], state.contact[7], state.contact[8])
		local cmargin=useCase.contactMargin or 0.005
		for icls,cls in ipairs({"L","R","LH","RH"}) do
			if prevContact[cls] and not contact[cls] then
				for ii,i in ipairs(targets[cls]) do
					simulator:setCollisionMargin(i-1, 0) 
				end
			elseif not prevContact[cls] and contact[cls] then
				for ii,i in ipairs(targets[cls]) do
					simulator:setCollisionMargin(i-1, cmargin) -- allow some penetration
				end
			end
		end
	end
	RE.output2("contactDuration", table.tostring(state.contactDuration))
	--_checkpoints:pushBack(deepCopyTable({'cte', prevContact, contact, state}))
	--self:addCOMobjective(simulator, vector3(30,0,10))
		--##dos if g_debugOneStep then --##dos g_debugOneStep:pushBack({"qpE",self.qp.H:copy(), self.qp.R:copy()}) --##dos end
end

function QPservo:calcContactCentroid(sim, graph, swingFoot)
	local simulator=self.simulator
	local contactHull=self.contactHull
	assert(contactHull)
	--assert(contactHull.N>=1)
	if contactHull.N==0 then RE.output2("warning","contactHull.N==0") end

	if swingFoot~="L" then
		local frameL=simulator:getWorldState(0):globalFrame(graph.lfoot)
		-- it's safe to include the current heel and toe positions in the support polygon. 
		-- This allows faster swiching between heel and toe supports.
		contactHull:addVector3(frameL.translation)
		contactHull:addVector3(frameL:toGlobalPos(graph.lfootpos))
	end

	if swingFoot~="R" then
		local frameR=simulator:getWorldState(0):globalFrame(graph.rfoot)
		contactHull:addVector3(frameR.translation)
		contactHull:addVector3(frameR:toGlobalPos(graph.rfootpos))
	end

	contactHull:buildHull()

	local centroid, area=contactHull:calcCentroid()
	centroid=vector3(centroid.x, 0, centroid.y)
	return centroid
end

function QPservo:rewindTargetMotion(sim)
	local simulator=self.simulator
	self.deltaTime=-1*simulator:currentTime()
end


