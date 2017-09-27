require('RigidBodyWin/subRoutines/ConvexHull2D')

function __globals.createHybridDynamicsSolver()
   return Physics.DynamicsSimulator_gmbs_penalty()
--   return DynamicsSimulator_VP_penalty()
end

-- PDServo class
--class 'IDservo'
IDservo=LUAclass()

function IDservo:setCoef(dofInfo,kp, kd, tgtVelScale, k_scale)
   kp:setSize(dofInfo:numDOF())
   kp:setAllValue(k_p)
   kd:setSize(dofInfo:numDOF())
   kd:setAllValue(k_d)
   tgtVelScale:setSize(dofInfo:numDOF())
   tgtVelScale:setAllValue(k_d)
   
   -- exclude root joint
   kp:range(0,7):setAllValue(0)
   kd:range(0,7):setAllValue(0)
   tgtVelScale:range(0,7):setAllValue(0)
  
   print("initIDservo:"..dofInfo:skeleton():bone(1):name())
   for i=2,dofInfo:skeleton():numBone()-1 do
      local bone=dofInfo:skeleton():bone(i)
      local vbone=bone:treeIndex()
      local nJoint=dofInfo:numDOF(vbone)
--      print("initIDservo:"..bone:name())
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

function IDservo:updateCoef()
   local dofInfo=self.dofInfo

   k_p=model.k_p_ID or 100 -- hyunwoo 100
   k_d=model.k_d_ID or 30 -- hyunwoo 50
   k_p_slide=model.k_p_ID*5
   k_d_slide=model.k_d_ID*5

   -- self:setIDGain(dofInfo:skeleton(), self.kp_id, self.kd_id, k_p, k_d, k_p_slide or k_p*5, k_d_slide or k_d*5)
   local unused=vectorn()
   self:setCoef(dofInfo, self.kp_id, self.kd_id, unused, model.k_scale_id)

   k_p=model.k_p_PD or 100 -- hyunwoo 100
   k_d=model.k_d_PD or 10 -- hyunwoo 50
   k_p_slide=model.k_p_PD*100
   k_d_slide=model.k_d_PD*100

   local k_scale_active=model.k_scale_active_pd

   self:setCoef(dofInfo,self.kp_active, self.kd_active, self.tgtVelScale_active, k_scale_active)

   local k_scale_passive=model.k_scale_passive_pd

   self:setCoef(dofInfo,self.kp_passive, self.kd_passive, self.tgtVelScale_passive, k_scale_passive)
end

function IDservo:__init(dofInfo,timestep,integrator)
	self.state={previousFlightPhase=false, flightPhase=false, supportPhaseElapsed=100, flightPhaseElapsed=0}
   self.theta=vectorn()
   self.dtheta=vectorn()
   -- HD servo
   self.theta_d=vectorn() -- desired q
   self.dtheta_d=vectorn() -- desired dq
   self.ddtheta_d=vectorn() -- desired ddq

   -- PD servo
   self.theta_d_pd=vectorn()

   self.desiredacceleration=vectorn()
   self.controlforceSmooth=vectorn()
   self.controlforce=vectorn()
   self.kp=vectorn()
   self.kd=vectorn()
   self.kp_id=vectorn()
   self.kd_id=vectorn()

   self.tgtVelScale=vectorn()
   self.kp_active=vectorn()
   self.kd_active=vectorn()
   self.tgtVelScale_active=vectorn()
   self.kp_passive=vectorn()
   self.kd_passive=vectorn()
   self.tgtVelScale_passive=vectorn()
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

   -- todo: create DynamicsSimulator...
   self.IDsolver=__globals.createHybridDynamicsSolver()
   self.IDsolver:registerCharacter(skel)
   self.IDsolver:setGVector(vector3(0,-9.8,0))
   self.IDsolver:init(timestep, integrator)
      
   local lhip=skel:getBoneByVoca(MotionLoader.LEFTHIP)
   local rhip=skel:getBoneByVoca(MotionLoader.RIGHTHIP)

   self.lkneeDOF=dofInfo:DOFindex(skel:getBoneByVoca(MotionLoader.LEFTKNEE):treeIndex(),0)
   self.rkneeDOF=dofInfo:DOFindex(skel:getBoneByVoca(MotionLoader.RIGHTKNEE):treeIndex(),0)
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


   return o
end

function IDservo:initIDservo(startf, endf,motionDOF, dmotionDOF, ddmotionDOF, motionDOF_pdtarget)

   self.startFrame=startf
   self.endFrame=endf
   self.currFrame=startf
   self.deltaTime=0
   self.motionDOF=motionDOF
   self.dmotionDOF=dmotionDOF
   self.ddmotionDOF=ddmotionDOF
   self.motionDOF_pdtarget=motionDOF_pdtarget or motionDOF

end

-- generate FBtorque
function IDservo:generateTorque(simulator, maxForce)
   
   self.currFrame=(simulator:currentTime()+self.deltaTime)*model.frame_rate+self.startFrame
   --print(self.currFrame) -- extremely slow.
   if self.currFrame>self.endFrame-1 then
      simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, self.theta)
      simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, self.dtheta)
      return false
   end
   
   self:_generateTorque(simulator, self.currFrame, maxForce)
   return true
end
function IDservo:_calcDesiredAcceleration()
	local state=self.theta
	local dstate=self.dtheta
   self.desiredacceleration:setSize(self.motionDOF:numDOF())
   
--   self.desiredacceleration:assign(self.kp*(self.theta_d-state)+
--			 self.kd*(self.dtheta_d-dstate))

   local delta=self.theta_d-state
   MainLib.VRMLloader.projectAngles(delta) -- [-pi, pi]


   self.desiredacceleration:assign(self.kp_id*delta +  self.kd_id*(self.dtheta_d*(useCase.IDservoDScaleCoef or 1.0)-dstate))
   self.desiredacceleration:smoothClamp(-400, 400)
   self.desiredacceleration:radd(self.ddtheta_d)
   --self.desiredacceleration:clamp(-400, 400)

end


-- deprecated: use _calcDesiredAcceleration
function IDservo:calcDesiredAcceleration(simulator, frame, state, dstate)


	--[[ continuous sampling ]]--
	--   print("theta",self.theta)

	self:sampleTargetPoses(frame)

	--   self.dtheta_d:setAllValue(0)
	self:_calcDesiredAcceleration()
end

   function IDservo:__computeHDtorque(state, dstate,cf)
	   self.IDsolver:hybridDynamics(state, dstate, self.desiredacceleration, cf, self.controlforce)
	   --   self.idSolver:inverseDynamics(state, dstate, self.desiredacceleration, self.controlforce)

	   self.controlforce:clamp(self.clampMinID, self.clampMaxID)
	   if false then -- smooth hybrid dynamics output.
		   smoothAmt=0.5
		   if self.controlforceSmooth:size()~=0 then
			   self.controlforce:assign(self.controlforceSmooth*smoothAmt+self.controlforce*(1-smoothAmt))
		   end
		   self.controlforceSmooth:assign(self.controlforce)
	   end

	   if false then -- discard sliding joint actuation (need to use high-gain : slows down simulation)
		   self.controlforce:rsub(self.controlforce*(self.mask_slide*1))
	   end

	   if true then -- selective compliance. (disable some of the joint torques)

		   if true then -- 66.025ms
			   self.controlforce:assign(self.controlforce*self.mask_lleg*self.scale_lleg+
			   self.controlforce*self.mask_rleg*self.scale_rleg+
			   self.controlforce*self.mask_upperbody*self.scale_upperbody)
		   else -- 47.009 ms
			   -- code optimized. not a big deal, just for fun benchmarking.
			   local cf=self.controlforce:copy()
			   local t=vectorn()
			   t:assign(cf)
			   t:rmult(self.scale_lleg)
			   t:rmult(self.mask_lleg)
			   self.controlforce:assign(t)
			   t:assign(cf)
			   t:rmult(self.scale_rleg)
			   t:rmult(self.mask_rleg)
			   self.controlforce:radd(t)
			   t:assign(cf)
			   t:rmult(self.scale_upperbody)
			   t:rmult(self.mask_upperbody)
			   self.controlforce:radd(t)
		   end
	   end
	   if model.k_ID_torque_scale then
		   self.controlforce:rmult(model.k_ID_torque_scale)
	   end

	   self.controlforce:range(0,7):setAllValue(0)
   end
   
 function IDservo:_computeHDtorque(simulator, state, dstate, maxForce, swingFoot)
	 local cf=self:computeClampedContactForce(simulator, maxForce, swingFoot)
	 self:__computeHDtorque(state, dstate,cf)
 end

 function IDservo:sampleCurrPose(simulator)
	 simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, self.theta)
	 simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, self.dtheta)
 end
 function IDservo:sampleTargetPoses( frame)
	-- desired (target) pose
	self.motionDOF:samplePose(frame, self.theta_d)
	 self.motionDOF_pdtarget:samplePose(frame, self.theta_d_pd)
	self.dmotionDOF:sampleRow(frame, self.dtheta_d)
	self.ddmotionDOF:sampleRow(frame, self.ddtheta_d)

 end

 function IDservo:computeClampedContactForce( simulator, maxForce, swingFoot)
	 self.contactHull=nil
	 if usePenaltyMethod then
		 --if false then
		 if false then -- smooth hybrid dynamics input : contact force
			 self.IDsolver:queryContactAll():interpolate(
			 0.5, self.IDsolver:queryContactAll(),simulator:queryContactAll(), simulator)
			 return self.IDsolver:queryContactAll()
		 elseif false then -- scale 
			 local cf=Vec_CFinfo()
			 cf:assign(simulator:queryContactAll())
			 cf:scale(1)
			 return cf
		 elseif true then -- clamp

			 local cf=Physics.Vec_CFinfo()
			 cf:assign(simulator:queryContactAll())
			 local forceY=0
			 local cop=vector3(0,0,0) -- center of pressure: unused

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
					  
					 if true then -- ignore unexpected contact forces
						 if swingFoot=="L" and isLfoot then
							 ci.f:setValue(0,0,0)
							 ci.tau:setValue(0,0,0)
						 end
						 if swingFoot=="R" and isRfoot then
							 ci.f:setValue(0,0,0)
							 ci.tau:setValue(0,0,0)
						 end
						 if false then -- TODO: make continuously varying
						 --if swingFoot=="N" then
							 ci.f:setValue(0,0,0)
							 ci.tau:setValue(0,0,0)
						 end
					 end
					 local frame=simulator:getWorldState(0):globalFrame(ci.bone)
					 local globalForce=frame:toGlobalDir(ci.f)
					 forceY=forceY+globalForce.y
					 local globalPos=frame:toGlobalPos(ci.p)


					 if not (swingFoot=="L" and actualContact.L ) and not (swingFoot=="R" and actualContact.R) then
						 self.contactHull:addVector3(globalPos)
					 end
					 cop=cop+globalForce.y*globalPos
				 end
			 end

			 cstate.actualContact=actualContact
			 cop=cop/forceY

			 cop.y=0
			 self.COP=cop

			 if cop.x==cop.x and cop.z==cop.z then
				 dbg.namedDraw('Sphere', cop*100, "cop",'green', 1) 
			 end


			 -- detect flight phase
			 local maxForceY=maxForce or 9.8*80*4 -- smoothness clamping? 
			 local swingFoot=swingFoot or "N"

			 cstate.flightPhase=false
			 if forceY>maxForceY then
				 cf:scale(maxForceY/forceY)
			 elseif forceY<=0 then
				 cstate.flightPhase=true
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

			 return cf

		 elseif false then -- selective scale
			 local cf=Vec_CFinfo()
			 cf:assign(simulator:queryContactAll())

			 for i=0, cf:size()-1 do
				 if cf(i).chara==0 then
					 local ci=cf(i)
					 local frame=simulator:getWorldState(0):globalFrame(ci.bone)
					 local globalForce=frame:toGlobalDir(ci.f)
					 globalForce.y=globalForce.y*0.2
					 ci.f:assign(frame:toLocalDir(globalForce))
				 end
			 end
			 return cf
		 else
			 return simulator:queryContactAll()
		 end
	 else
		 return Vec_CFinfo()
	 end
 end
 function IDservo:addPDtorque(simulator)
   -- pdservo

   
   if mrd_info and mrd_info.outputContactForce and usePenaltyMethod then
      local cqInfo=simulator:queryContactAll()      
      local collector=mrd_info.outputContactForce[2]
      for i=0, cqInfo:size()-1 do
	 if cqInfo(i).chara==0 then
	    local bone=cqInfo(i).bone
	    if bone:name()=="lfoot" or bone:name()=="ltoes" then
	       collector[1]:radd(simulator:getWorldState(0):globalFrame(bone):toGlobalDir(cqInfo(i).f))
	    else
	       collector[2]:radd(simulator:getWorldState(0):globalFrame(bone):toGlobalDir(cqInfo(i).f))
	    end
	 end
      end
   end
   self.kp:interpolate(self.kp_weight, self.kp_passive, self.kp_active)
   self.kd:interpolate(self.kd_weight, self.kd_passive, self.kd_active)
   self.tgtVelScale:interpolate(self.kd_weight, self.tgtVelScale_passive, self.tgtVelScale_active)
   local delta_pd=self.theta_d_pd-self.theta
   MainLib.VRMLloader.projectAngles(delta_pd) -- [-pi, pi]

   local pdforce=self.kp*delta_pd + self.kd*(self.dtheta_d*self.muscleActiveness*self.tgtVelScale-self.dtheta)
   pdforce:clamp(self.clampMin, self.clampMax)
   ---- taesoo debug.. 
   --pdforce:range(0,7):setAllValue(0)
   --self.controlforce:range(0,7):setAllValue(0)
   
   self.controlforce:radd(pdforce)
   do return end

   if self.kneeTorqueL then
      self.controlforce:set(self.lkneeDOF, self.controlforce(self.lkneeDOF)+self.kneeTorqueL)
      self.controlforce:set(self.rkneeDOF, self.controlforce(self.rkneeDOF)+self.kneeTorqueR)
   end

   if g_debugOneStep and g_debugOneStepFlag then

	   for i=0, 6 do 
		   assert(pdforce(i)==0)
	   end
      g_debugOneStep:pushBack(saveDebugInfo(simulator))
      if simulator._debugInfo:length()~=0 then 
	 g_debugOneStep:pushBack(tostring(simulator._debugInfo)) 
	 simulator._debugInfo:assign("")
      end
      g_debugOneStep:pushBack({"theta",self.theta:copy()})
      g_debugOneStep:pushBack({"dtheta",self.dtheta:copy()})
      g_debugOneStep:pushBack({"dtheta_d",self.dtheta_d:copy()})
      g_debugOneStep:pushBack({"theta_d_pd",self.theta_d_pd:copy(),self.kp, self.kd, self.tgtVelScale})
      g_debugOneStep:pushBack({"desiredAcc",self.desiredacceleration:copy()})
      g_debugOneStep:pushBack({"controlforce",self.controlforce:copy()})
      g_debugOneStep:pushBack({"pdforce",pdforce:copy()})
      local cqInfo=simulator:queryContactAll()
      g_debugOneStep:pushBack({"cqInfo", cqInfo:size()})
      for i=0, cqInfo:size()-1 do
	 if cqInfo(i).chara==0 then
	    g_debugOneStep:pushBack({cqInfo(i).bone:name(), cqInfo(i).p:copy(), cqInfo(i).tau:copy(), cqInfo(i).f:copy(), })
	 end
      end
      
      --g_debugOneStepFlag=false -- store all frames or just the first simulation frames
   end

 end
function IDservo:_generateTorque(simulator, frame, maxForce, swingFoot)

   simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, self.theta)
   simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, self.dtheta)
	self:calcDesiredAcceleration(simulator, frame, self.theta, self.dtheta)
	self:_computeHDtorque(simulator, self.theta, self.dtheta, maxForce, swingFoot)
	self:addPDtorque(simulator)

end

function IDservo:calcContactCentroid(simulator, graph, swingFoot)
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

function IDservo:rewindTargetMotion(simulator)
   self.deltaTime=-1*simulator:currentTime()
end



--class 'PoseMaintainerID'
PoseMaintainerID=LUAclass()

function PoseMaintainerID:__init(skel)
   self.theta=vectorn()
   self.dtheta=vectorn()
   self.theta_d=vectorn() -- desired q
   self.dtheta_d=vectorn() -- desired dq

   self.IDsolver=__globals.createHybridDynamicsSolver()
   --self.IDsolver=

   self.IDsolver:registerCharacter(skel)
   self.IDsolver:setGVector(vector3(0,-9.8,0))
   self.IDsolver:init(timestep, integrator)
   self.desiredAcceleration=vectorn()

   self.controlforce=vectorn()

   -- followings are temporaries
   self.kp=vectorn()
   self.kd=vectorn()
   self.kp_pd=vectorn()
   self.kd_pd=vectorn()
   self.mask_slide=vectorn()
   self.mask_rotate=vectorn()
end

function PoseMaintainerID:setGain(skel, kp, kd, k_p, k_d, k_p_slide, k_d_slide)
   local dofInfo=skel.dofInfo
   kp:setSize(dofInfo:numDOF())
   kp:setAllValue(k_p)
   kd:setSize(dofInfo:numDOF())
   kd:setAllValue(k_d)
   
   -- exclude root joint
   kp:range(0,7):setAllValue(0)
   kd:range(0,7):setAllValue(0)
   
   for i=1,skel:numBone()-1 do
      vbone=skel:VRMLbone(i)
      nJoint=vbone:numHRPjoints()
      for j=0, nJoint-1 do
	 if vbone:HRPjointType(j)==VRMLTransform.SLIDE then
	    kp:set(vbone:DOFindex(j), k_p_slide)
	    kd:set(vbone:DOFindex(j), k_d_slide)
	 end
      end
   end
end
   
function PoseMaintainerID:init(skel, simulator, k_p, k_d, k_p_slide, k_d_slide)
   simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, self.theta_d)
   simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, self.dtheta_d)

   k_p=200
   k_d=30

   self:setGain(skel, self.kp, self.kd, k_p, k_d, k_p_slide or k_p*10, k_d_slide or k_d*10)

   k_p=100
   k_d=10

   self:setGain(skel, self.kp_pd, self.kd_pd, k_p, k_d, k_p_slide or k_p*50, k_d_slide or k_d*50)

   local clampTorque=400
   local clampForce=4000

   if model.clampTorque~=nil then
      clampTorque=model.clampTorque
   end

   if model.clampForce~=nil then
      clampForce=model.clampForce
   end

   local dofInfo=skel.dofInfo

   self.mask_slide:setSize(dofInfo:numDOF())
   self.mask_rotate:setSize(dofInfo:numDOF())
   self.mask_slide:setAllValue(0)
   self.mask_rotate:setAllValue(1)


   self.clampMax=vectorn(dofInfo:numDOF())
   for i=2,skel:numBone()-1 do
      local vbone=skel:VRMLbone(i)
      local nJoint=vbone:numHRPjoints()
      for j=0, nJoint-1 do
	 local dofIndex=vbone:DOFindex(j)
	 if vbone:HRPjointType(j)==VRMLTransform.SLIDE then
	    self.clampMax:set(dofIndex, clampForce)
	    self.mask_slide:set(dofIndex,1)
	    self.mask_rotate:set(dofIndex,0)
	 else
	    self.clampMax:set(dofIndex, clampTorque)
	 end
      end
   end   

   self.clampMin=self.clampMax*-1
end

function PoseMaintainerID:generateTorque(simulator)

   simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, self.theta)
   simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, self.dtheta)
   self.desiredAcceleration:setSize(simulator:skeleton(0).dofInfo:numDOF())
   self.desiredAcceleration:setAllValue(0)

   self.desiredAcceleration:assign(self.kp*(self.theta_d-self.theta)+
				self.kd*(self.dtheta_d-self.dtheta))

   self.IDsolver:hybridDynamics(self.theta, self.dtheta, self.desiredAcceleration, simulator:queryContactAll(), self.controlforce)
--   self.IDsolver:inverseDynamics(self.theta, self.dtheta, self.desiredAcceleration, self.controlforce)

   -- execlude root joint
   self.controlforce:range(0,7):setAllValue(0)
   self.controlforce:assign(self.controlforce*self.mask_rotate*0.8+self.controlforce*self.mask_slide)
--   self.controlforce:setAllValue(0)

   -- pd-servo
   self.controlforce:radd(self.kp_pd*(self.theta_d-self.theta)+self.kd_pd*(self.dtheta_d-self.dtheta))
   self.controlforce:clamp(self.clampMin, self.clampMax)


end

