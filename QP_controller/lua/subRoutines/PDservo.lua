-- PDServo class
--class 'PDservo'
PDservo=LUAclass()


function PDservo:setCoef(dofInfo,kp, kd, tgtVelScale, k_scale)
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
  
   print("initPDservo:"..dofInfo:skeleton():bone(1):name())
   for i=2,dofInfo:skeleton():numBone()-1 do
      local bone=dofInfo:skeleton():bone(i)
      local vbone=bone:treeIndex()
      local nJoint=dofInfo:numDOF(vbone)
--      print("initPDservo:"..bone:name())
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
	    kp:set(dofIndex, model.k_p_slide)
	    kd:set(dofIndex, model.k_d_slide)
	    tgtVelScale:set(dofIndex, 0)
	 end
      end
   end
end

function PDservo:updateCoef()
   local dofInfo=self.dofInfo
   local k_scale_active=model.k_scale_active_pd

   self:setCoef(dofInfo,self.kp_active, self.kd_active, self.tgtVelScale_active, k_scale_active)

   local k_scale_passive=model.k_scale_passive_pd

   self:setCoef(dofInfo,self.kp_passive, self.kd_passive, self.tgtVelScale_passive, k_scale_passive)
end

function PDservo:__init(dofInfo)
   self.theta=vectorn()
   self.dtheta=vectorn()
   self.theta_d=vectorn() -- desired q
   self.dtheta_d=vectorn() -- desired dq
   self.controlforce=vectorn()
   self.kp=vectorn()
   self.kd=vectorn()
   self.tgtVelScale=vectorn()
   self.kp_active=vectorn()
   self.kd_active=vectorn()
   self.tgtVelScale_active=vectorn()
   self.kp_passive=vectorn()
   self.kd_passive=vectorn()
   self.tgtVelScale_passive=vectorn()
   self.mask_slide=vectorn()
   self.muscleActiveness=0.3
   self.kp_weight=1.0 -- use kp_active(1) or kp_passive(0)
   self.kd_weight=1.0 -- use kd_active(1) or kd_passive(0)
   self.mask_slide:setSize(dofInfo:numDOF())
   self.mask_slide:setAllValue(0)
   self.dofInfo=dofInfo
   self:updateCoef()
   print ("kp=",self.kp)
   print ("kd=",self.kd)

   local clampTorque=800
   local clampForce=8000

   if model.clampTorque~=nil then
      clampTorque=model.clampTorque
   end

   if model.clampForce~=nil then
      clampForce=model.clampForce
   end

   self.clampMax=vectorn(dofInfo:numDOF())
   for i=2,dofInfo:skeleton():numBone()-1 do
      local bone=dofInfo:skeleton():bone(i)
      local vbone=bone:treeIndex()
      local nJoint=dofInfo:numDOF(vbone)
      for j=0, nJoint-1 do
	 local dofIndex=dofInfo:DOFindex(vbone,j)
	 if dofInfo:DOFtype(vbone, j)==MotionDOFinfo.SLIDE then
	    local dofIndex=dofInfo:DOFindex(vbone,j)
	    self.mask_slide:set(dofIndex, 1)
	    self.clampMax:set(dofIndex, clampForce)
	 else
	    self.clampMax:set(dofIndex, clampTorque)
	 end
      end
   end   

   self.clampMin=self.clampMax*-1
   return o
end

function PDservo:initPDservo(startf, endf,motionDOF, dmotionDOF)
   self.startFrame=startf
   self.endFrame=endf
   self.currFrame=startf
   self.deltaTime=0
   self.motionDOF=motionDOF
   self.dmotionDOF=dmotionDOF
end

-- generate FBtorque
function PDservo:generateTorque(simulator)
   
   self.currFrame=(simulator:currentTime()+self.deltaTime)*model.frame_rate+self.startFrame
   --print(self.currFrame) -- extremely slow.
   if self.currFrame>self.endFrame-1 then
      simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, self.theta)
      simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, self.dtheta)
      return false
   end
   
   self:_generateTorque(simulator, self.currFrame)
   return true
end

function PDservo:_generateTorque(simulator, frame)
   
   simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, self.theta)
   simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, self.dtheta)
   
   --[[ continuous sampling ]]--
--   print("theta",self.theta)
   
   -- desired (target) pose
   self.motionDOF:samplePose(frame, self.theta_d)
   self.dmotionDOF:sampleRow(frame, self.dtheta_d)
   
--   self.dtheta_d:setAllValue(0)

   self.dtheta_d:rmult(self.muscleActiveness) -- this corresponds to muscle activeness

   self.controlforce:setSize(self.motionDOF:numDOF())
   
--   self.controlforce:assign(self.kp*(self.theta_d-self.theta)+
--			 self.kd*(self.dtheta_d-self.dtheta))

   local delta=self.theta_d-self.theta
   MainLib.VRMLloader.projectAngles(delta) -- [-pi, pi]

   self.kp:interpolate(self.kp_weight, self.kp_passive, self.kp_active)
   self.kd:interpolate(self.kd_weight, self.kd_passive, self.kd_active)
   self.tgtVelScale:interpolate(self.kd_weight, self.tgtVelScale_passive, self.tgtVelScale_active)

   self.controlforce:assign(self.kp*delta +
			    self.kd*(self.dtheta_d*self.tgtVelScale-self.dtheta))

   self.controlforce:clamp(self.clampMin, self.clampMax)

end

function PDservo:rewindTargetMotion(simulator)
   self.deltaTime=-1*simulator:currentTime()
end

-- following functions are for feedback error learning.

-- segment={ mot, dmot, offsetTable}
function updateFFtorque(segment, debugInfo)
   
   
   dtor_loop()
   
   
   
   mPDservo=PDservo:new(segment.mot.dofInfo)
   mPDservo:initPDservo(0, segment.mot:numFrames(), segment.mot, segment.dmot)
   
   
   if segment.offsetTable:rows()==0 then
      segment.offsetTable:resize(segment.mot:numFrames(), segment.mot.dofInfo:numDOF())
      segment.offsetTable:setAllValue(0)
   end
   
   local tempOffsetTable=matrixn()
   local tempOffsetCount=vectorn()
   
   
   tempOffsetCount:setSize(segment.mot:numFrames())
   tempOffsetCount:setAllValue(0)
   tempOffsetTable:setSize(segment.mot:numFrames(), segment.mot.dofInfo:numDOF())
   tempOffsetTable:setAllValue(0)
	
   --mFloor=VRMLloader("../Resource/mesh/floor.wrl")
   mFloor=MainLib.VRMLloader("../Resource/mesh/floor_y.wrl")
	
   drawSkeleton=false
	
   assert(mSkel~=nil, "skel nil")
	
	
	mSkin=RE.createVRMLskin(mSkel, drawSkeleton)
	mSkin:setThickness(0.03)
	
	if showDesiredPose then
		mSkin2=RE.createVRMLskin(mSkel, drawSkeleton)
		mSkin2:setThickness(0.03)
		mSkin2:scale(100,100,100)
		mSkin2:setVisible(false)
	end
	
	mSkinFloor=RE.createVRMLskin(mFloor, false)

	mSkin:scale(100,100,100)
	mSkinFloor:scale(100,100,100)
	mSimulator=createSimulator(simulator)
	mSimulator:registerCharacter(mSkel)
	mSimulator:registerCharacter(mFloor)

	registerContactPair(model, mSkel, mFloor, mSimulator)
	mSimulator:init(timestep, integrator)

	mSimulator:setSimulatorParam("debugContact", debugContactParam) 
	mSimulator:setSimulatorParam("penaltyDepthMax", model.penaltyDepthMax )
	
	initialState=vectorn()
	initialState:assign(mPDservo.motionDOF:row(0))
	
	-- set global position
	initialState:set(1,initialState:get(1)+initialHeight)	
	
	mSimulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, initialState)
	mSimulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, mPDservo.dmotionDOF:row(0))
	mSkin:setPoseDOF(initialState)
	
	showDesiredPose(initialState)
	--mSimulator.setGVector(vector3(0,0,9.8))
	mSimulator:setGVector(vector3(0,9.8,0))
	mSimulator:initSimulation()
	
	local iter=0
	local offset=vectorn()
	local error=0
	
	while true do
		assert(mSkel~=nill)
	
		mSimulator:stepSimulation()
		
	
		if mPDservo:generateTorque(mSimulator) ==false then
		
			for ii=0, tempOffsetTable:rows()-1 do
				tempOffsetTable:row(ii):rdiv(tempOffsetCount:get(ii))	-- average FBtorque per bin.
			end
			
			if debugInfo~=nil then
				mm=tempOffsetTable:minimum()
				MM=tempOffsetTable:maximum()
				tempOffsetTable:draw(debugInfo.."before.bmp")
			end
			
			-- about half second kernel.
			math.gaussFilter(mPDservo.motionDOF.dofInfo:frameRate()*0.5, tempOffsetTable)
			
			if debugInfo~=nil then
				tempOffsetTable:draw(debugInfo.."after.bmp", mm, MM)
			end
			
			-- update Feed-forward torques
			segment.offsetTable:assign(segment.offsetTable*0.9+tempOffsetTable*0.1)
			break
		end

		local controlforce=vectorn()
		local FFforce=vectorn()
		-- FF torque
		segment.offsetTable:sampleRow(mPDservo.currFrame, FFforce)
		-- FB torque + FF torque
		controlforce:add(mPDservo.controlforce, FFforce)
		
		mSimulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_TORQUE, controlforce)
		--mSimulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_TORQUE, mPDservo.controlforce)

		local binIndex=math.floor(mPDservo.currFrame+0.5)
		
		-- gather Feed-back torques.
		
		mPDservo.controlforce:rsub(mPDservo.controlforce*mPDservo.mask_slide)	-- discard slide joints.
		tempOffsetTable:row(binIndex):radd(mPDservo.controlforce)
		tempOffsetCount:set(binIndex, tempOffsetCount:get(binIndex)+1)
		error=error+mPDservo.controlforce:length()
		
		if math.mod(iter, niter)==0 then		
			-- rendering

			--subtitlebar:setCaption("pos="..pos:__tostring().." dist="..dist)
			
			if g_signal~=nill then
				local controlforce=mPDservo.controlforce
				g_signal:resize(g_signal:rows()+1, 4)
				g_signal:row(g_signal:rows()-1):assign({controlforce:get(7), controlforce:get(8), controlforce:get(9), controlforce:get(10)})
			end
			
			
			showDesiredPose(mPDservo.theta_d)
			
			
			mSimulator:drawDebugInformation()
			mObjectList:clear()

			if drawCOM then
				local com=mSimulator:calculateCOM(0)
				local comDir=mSimulator:calculateCOMvel(0)
				com:assign(com*100)
				comDir:assign(comDir*100)
				comEntity=mObjectList:registerEntity("COM", "sphere1010.mesh")
				comEntity:setScale(20,20,20)
				comEntity:setPosition(com.x, com.y, com.z)
				lines=vector3N()
				lines:setSize(2)
				lines:at(0):assign(com)
				lines:at(1):assign(com+comDir*10)
				--mObjectList:registerObject("Comdir", "LineList", "solidred", lines,0)
			end
			if drawZMP then
				local zmp=mSimulator:calculateZMP(0)
				zmp:assign(zmp*100)
				print("ZMP=",zmp.x, zmp.y, zmp.z)
				zmpEntity=mObjectList:registerEntity("ZMP", "sphere1010.mesh")
				zmpEntity:setScale(10,10,10)
				zmpEntity:setPosition(zmp.x, zmp.y, zmp.z)
			end
							
			mSkin:setPose(mSimulator,0)

	
			RE.renderOneFrame(true)
			

		end
		
		iter=iter+1
	end
	
	
	
	return error
end

function dtor_loop()
	-- remove objects that are owned by C++
	if mSkin~=nill then
		RE.remove(mSkin)
		mSkin=nil
	end
	-- remove objects that are owned by C++
	if mSkin2~=nill then
		RE.remove(mSkin2)
		mSkin2=nil
	end
	if mSkinFloor~=nill then
		RE.remove(mSkinFloor)
		mSkinFloor=nil
	end
	mObjectList:clear()
	-- remove objects that are owned by LUA
	mPDservo=nil
	collectgarbage()
end



--class 'PoseMaintainer'
PoseMaintainer=LUAclass()

function PoseMaintainer:__init()
	self.theta=vectorn()
	self.dtheta=vectorn()
	self.theta_d=vectorn() -- desired q
	self.dtheta_d=vectorn() -- desired dq
	self.controlforce=vectorn()

	-- followings are temporaries
	self.kp=vectorn()
	self.kd=vectorn()
end

function PoseMaintainer:init(skel, simulator, k_p, k_d, k_p_slide, k_d_slide)
	simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, self.theta_d)
	simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, self.dtheta_d)

	local dofInfo=skel.dofInfo
	self.kp:setSize(dofInfo:numDOF())
	self.kp:setAllValue(k_p)
	self.kd:setSize(dofInfo:numDOF())
	self.kd:setAllValue(k_d)
	
	if skel:VRMLbone(1):HRPjointType(0)==MainLib.VRMLTransform.FREE then
		-- exclude free root joint
		self.kp:range(0,7):setAllValue(0)
		self.kd:range(0,7):setAllValue(0)
	end
	
	if k_p_slide==nil then
	   k_p_slide=k_p*10
	end

	if k_d_slide==nil then
	   k_d_slide=k_d*500
	end

	for i=1,skel:numBone()-1 do
		vbone=skel:VRMLbone(i)
		nJoint=vbone:numHRPjoints()
		for j=0, nJoint-1 do
			if vbone:HRPjointType(j)==MainLib.VRMLTransform.SLIDE then
				self.kp:set(vbone:DOFindex(j), k_p_slide)
				self.kd:set(vbone:DOFindex(j), k_d_slide)
			end
		end
	end
end

function PoseMaintainer:generateTorque(simulator)
	simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, self.theta)
	simulator:getLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, self.dtheta)
	self.controlforce:setSize(simulator:skeleton(0).dofInfo:numDOF())
	self.controlforce:setAllValue(0)

	self.controlforce:assign(self.kp*(self.theta_d-self.theta)+
		self.kd*(self.dtheta_d-self.dtheta))

end
function PoseMaintainer:resetParam(kp, kd, theta_d)
	self.kp:setAllValue(kp)
	self.kd:setAllValue(kd)
	self.theta_d:assign(theta_d)
end
