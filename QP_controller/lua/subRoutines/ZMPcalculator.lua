require("RigidBodyWin/subRoutines/PointTracker")

--class 'ZMPcalculatorOnline'
ZMPcalculatorOnline=LUAclass()
function ZMPcalculatorOnline:__init(simulator, frameRate)
   self.simulator=simulator
   local skel=self.simulator:skeleton(0)
   self.tracker=vector(skel:numBone())
   self.frameRate=frameRate

   self.zmp=vector3()
end

function ZMPcalculatorOnline:step()

   local sim=self.simulator
   local skel=sim:skeleton(0)
   local chain=sim:getWorldState(0)


   local g=-9.8

   local A=0
   local B=0
   local C=0
   local D=0
   local E=0

   for ibone=1,skel:numBone()-1 do
--      print(ibone, skel:name())

      local bone=skel:VRMLbone(ibone)
      local ri=chain:globalFrame(bone):toGlobalPos(bone:localCOM())
      local ri_o=sim:getWorldVelocity(0,bone, bone:localCOM())
      local ri__o=sim:getWorldAcceleration(0,bone, bone:localCOM())
      
      if self.tracker(ibone)==nil then
	 local tracker=PointTracker(self.frameRate,
				    0.00001,0.5,
				    0.1,100)
	 tracker:init(CT.vec({ri_o.x, ri_o.y, ri_o.z, ri__o.x, ri__o.y, ri__o.z}),
			   CT.vec({0.01,0.01,0.01,0.01,0.01,0.01}))
	 self.tracker:set(ibone, {z=vectorn(6), pt=tracker})
      end

      local tr=self.tracker(ibone)
      tr.z:range(0,3):assignv(ri_o)
      tr.z:range(3,6):assignv(ri__o)

      tr.pt:step(tr.z)
      local xx=tr.pt:getState()
      local ri_=xx:toVector3(0)
      local ri__=xx:toVector3(3)
      local mi=bone:mass()
      A=A+mi*(ri__.y-g  )*ri.x
      B=B+mi*(ri__.x-0)*ri.y
      C=C+mi*(ri__.y-g  )
      D=D+mi*(ri__.y-g  )*ri.z
      E=E+mi*(ri__.z-0)*ri.y
   end
   self.zmp.x=(A-B)/C
   self.zmp.y=0
   self.zmp.z=(D-E)/C

end
       

ZMPcalculator=LUAclass()
function ZMPcalculator:pack(binaryFile)
   binaryFile:pack(self.ddmotionDOF)
   binaryFile:pack(self.com)
   binaryFile:pack(self.comvel)
   binaryFile:pack(self.zmp)
end

function ZMPcalculator:unpack(binaryFile)
   self.ddmotionDOF=matrixn()
   binaryFile:unpack(self.ddmotionDOF)
   binaryFile:unpack(self.com)
   binaryFile:unpack(self.comvel)
   binaryFile:unpack(self.zmp)
end

function ZMPcalculator:__finalize()
   self.motionDOF=nil
   self.dmotionDOF=nil
   self.simulator=nil
   self.skel=nil
end

function ZMPcalculator:__init(skel, motionDOF, dmotionDOF, binaryFile)
   self.motionDOF=motionDOF
   self.dmotionDOF=dmotionDOF

   if binaryFile==nil then
      self.simulator=Physics.DynamicsSimulator_AIST_penalty()
      -- self.simulator=Physics.DynamicsSimulator_gmbs_penalty()
         --self.simulator=DynamicsSimulator_SDFAST()
      --simulator=DynamicsSimulator_UT_penalty() , --언제부턴가 죽음.

      self.simulator:registerCharacter(skel)
      self.simulator:initSimulation()
   end

   self.skel=skel
   self.com=vector3N(motionDOF:numFrames())
   self.comvel=vector3N(motionDOF:numFrames())
   self.zmp=vector3N(motionDOF:numFrames())
   
   
   if binaryFile~=nil then
      self:unpack(binaryFile)
      return
   end
   
   self.ddmotionDOF=calcAcceleration(motionDOF)
   
   for i=0, motionDOF:numFrames()-1 do
      local com, vel, zmp=ZMPcalculator._calcAll(self, i)
      self.com:at(i):assign(com)
      self.comvel:at(i):assign(vel)
      self.zmp:at(i):assign(zmp)
  end	

   --assert(motionDOF:numFrames()>100)
 
   if(motionDOF:numFrames()<100 ) then
	   self.zmp:matView(0, motionDOF:numFrames()):setAllValue(0)
   else
	   zzmp=matrixn()
	   skel:calcZMP(motionDOF, zzmp, 1)
	   self.zmp:matView(0, motionDOF:numFrames()):assign(zzmp)
   end
end

function ZMPcalculator:_calcCOM(iframe)
   self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, self.motionDOF:row(iframe))
   self.simulator:initSimulation()
   return self.simulator:calculateCOM(0)
end

function ZMPcalculator:_calcCOMvel(iframe)
   self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, self.motionDOF:row(iframe))
   self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, self.dmotionDOF:row(iframe))
   self.simulator:initSimulation()
   return self.simulator:calculateCOM(0), self.simulator:calculateCOMvel(0)
end

function ZMPcalculator:_calcAll(iframe)
   self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE, self.motionDOF:row(iframe))
   self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_VELOCITY, self.dmotionDOF:row(iframe))
   self.simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_ACCELERATION, self.ddmotionDOF:row(iframe))
   self.simulator:initSimulation()
   return self.simulator:calculateCOM(0), self.simulator:calculateCOMvel(0),
   self.simulator:calculateZMP(0)
end

function ZMPcalculator:calcCOMacc()
   local com=self.com
   local timing=vectorn()
   local frameRate=self.skel.dofInfo:frameRate()
   timing:linspace(0,(com:size()-1)/frameRate, com:size())
--   com:matView():column(1):setAllValue(0)
   local curveFit=math.NonuniformSpline(timing, com:matView())
   
   self.COMvel=vector3N(com:size())
   self.COMacc=vector3N(com:size())
   curveFit:getFirstDeriv(timing, self.COMvel:matView())
   curveFit:getSecondDeriv(timing, self.COMacc:matView())
   
end


ZMPcalculator2=LUAclass()

function ZMPcalculator2:__init(skel, motionDOF, dmotionDOF, discontinuity)
   self.motionDOF=motionDOF
   self.dmotionDOF=dmotionDOF

   self.skel=skel
   self.com=vector3N(motionDOF:numFrames())
   self.comvel=vector3N(motionDOF:numFrames())
   self.zmp=vector3N(motionDOF:numFrames())

   local segFinder=SegmentFinder(discontinuity)

   
   for i=0, segFinder:numSegment()-1 do
      local s=segFinder:startFrame(i)
      local e=segFinder:endFrame(i)

      local zc=ZMPcalculator:new(skel, motionDOF:range(s,e), dmotionDOF:range(s,e, 0, dmotionDOF:cols()))

      self.com:range(s,e):assign(zc.com)
      self.comvel:range(s,e):assign(zc.comvel)
      self.zmp:range(s,e):assign(zc.zmp)
   end
end

