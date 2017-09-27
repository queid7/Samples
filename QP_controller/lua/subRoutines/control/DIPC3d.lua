require("RigidBodyWin/subRoutines/control/SDRE")

class 'DIPC3d'

function DIPC3d:__deinit()
   if self.skin~=nil then
      RE.remove(self.skin)
      self.skin=nil
   end
end

function DIPC3d:__finalize()
   self.skin=nil
   self.simulator=nil
   self.loader=nil

   if self.SDFASTsimulator == true then
      __globals.SDFASTcartPole_Used=nil
   end
end


function DIPC3d:__init(filename, b,g,dt,q)
   self.loader=VRMLloader(filename)
   self.loader:printHierarchy()
   
   self.skin=RE.createVRMLskin(self.loader, false)
   
   self.skin:scale(100,100,100)
   
   
--   if __globals.SDFASTcartPole_Used==nil then
   if false  then
      self.simulator=DynamicsSimulator_SDFAST()
      __globals.SDFASTcartPole_Used=true
      self.SDFASTsimulator=true
   else
      self.simulator=DynamicsSimulator_UT_penalty()
--      util.msgBox("Warning! SDFAST cannot be used for CartPole class")
   end

   self.simulator:registerCharacter(self.loader)
   
   self.simulator:init(dt, DynamicsSimulator.EULER)
   
   self.skin:setPose(self.simulator,0)
   
   self.simulator:setGVector(vector3(0,g,0))
   
   self.theta=vectorn(10)
   self.theta:setAllValue(0)
   self.theta:set(2,1) -- q_identity= 1,0,0,0
   self.theta:set(6,1) -- q_identity= 1,0,0,0


--   self.theta:setQuater(2,quater(math.rad(170), vector3(0,0,1))) 
--   self.theta:setQuater(6,quater(math.deg(120), vector3(0,0,1))) 


   self.dtheta=vectorn(10)
   self.dtheta:setAllValue(0)
   -- self.dtheta:set(0,0.01)
   -- self.dtheta:set(5,0.01)
   self.dtheta:set(1,0.01)
   self.dtheta:set(7,0.01)
   self.simulator:setLinkData(0, DynamicsSimulator.JOINT_VALUE, self.theta)
   self.simulator:setLinkData(0, DynamicsSimulator.JOINT_VELOCITY, self.dtheta)
   self.simulator:initSimulation()
   
   local cart=self.loader:VRMLbone(1)
   local pole1=self.loader:VRMLbone(2)
   local pole2=self.loader:VRMLbone(3)
   self.pole1=pole1
   self.pole2=pole2
   self.cart=cart
   print(cart:name(), pole1:name())
   local M=cart:mass()
   local m1=pole1:mass()
   local m2=pole2:mass()
   local l1=pole1:localCOM().y
   local l2=pole2:localCOM().y
   local i1=pole1:inertia().x
   local i2=pole2:inertia().x
   self.friction=b
   
   print(M,m1,m2,l1,l2,i1,i2)
   self.pendX=DIPCview(M, m1, m2, l1, l2, i1,i2,dt,q)
   self.pendZ=DIPCview(M, m1, m2, l1, l2, i1,i2,dt,q)
   self.dt=dt
   self.ori=quater()	-- cart orientation
   self.ori:setValue(1,0,0,0)
   self.desiredVel=vector3(0,0,0)
   self.desiredVelGlobal=vector3(0,0,0)
   self.controlforce=vectorn(10)
   self.Y=matrixn()
   self.numFrame=0

end



function DIPC3d:setOrientation2(q)
--   print(q)
   self.ori:assign(q)
    self:setDesiredVelocity(self.desiredVelGlobal)
--self:setDesiredVelocity(self.desiredVel)
   
 end


function DIPC3d:setOrientation(angle)

   self.ori:setRotation(vector3(0,1,0), math.rad(angle))
   self:setDesiredVelocity(self.desiredVelGlobal)
--self:setDesiredVelocity(self.desiredVel)
   
end

function projectQuaternion(q)
   local qoffset=quater()
   local qaxis=quater()

   q:decomposeNoTwistTimesTwist(vector3(0,1,0), qoffset, qaxis)
   q:assign(qoffset)
end


function DIPC3d:setState()
   self:projectState()
end

function DIPC3d:setDesiredVelocity(vel) -- vector3

   self.desiredVelGlobal:assign(vel)
   self.desiredVel:assign(vel)
   --   print(self.ori)
   self.desiredVel:rotate(self.ori:inverse())
   self.pendX:setParam(self.desiredVel.x)
   self.pendZ:setParam(self.desiredVel.z)
end


function DIPC3d:numFrames()
   --		return self.Y:rows()
   return self.numFrame
end

function DIPC3d:getPos(iframe)
   
   return vector3(self.Y(iframe,1), 0, self.Y(iframe,0))
end

function DIPC3d:setDesiredSpeedX(speed)
   self.pendX:setParam(speed)
end

function DIPC3d:setDesiredSpeedZ(speed)
   self.pendZ:setParam(speed)
end

function DIPC3d:calcState2D(xx, zx, controlForce)
   local pos=vector3()
   local d=vector3()
   
   pos.x=self.theta(1)
   pos.y=0
   pos.z=self.theta(0)
   d.x=self.dtheta(1)
   d.y=0
   d.z=self.dtheta(0)

   local q=self.theta:toQuater(2)
   local q2=q*self.theta:toQuater(6)
   local w=self.dtheta:toVector3(3)

   w:rotate(q)
   local w2=self.dtheta:toVector3(7)
   w2:rotate(q2)
   w2:radd(w)

   projectQuaternion(q)
   projectQuaternion(q2)

   local theta=q:rotationVector()
   local theta2=q2:rotationVector()

   local toLocal=self.ori:inverse()

   pos:rotate(toLocal)
   w:rotate(toLocal)
   w2:rotate(toLocal)
   theta:rotate(toLocal)
   theta2:rotate(toLocal)
   d:rotate(toLocal)

   if pos.x~=pos.x then debug.debug() end

   xx:set(0, 0, pos.x)
   xx:set(1, 0, -theta.z)
   xx:set(2, 0, -theta2.z)
   xx:set(3, 0, d.x)
   xx:set(4, 0, -w.z)
   xx:set(5, 0, -w2.z)

   w.y=0
   w2.y=0
   
--   RE.output("x_v", tostring(posx).." "..tostring(posz) .." "..tostring(dx).." "..tostring(dz))
   RE.output("theta_w", tostring(theta).." "..tostring(theta2))
   zx:set(0, 0, pos.z)
   zx:set(1, 0, theta.x)
   zx:set(2, 0, theta2.x)
   zx:set(3, 0, d.z)
   zx:set(4, 0, w.x)
   zx:set(5, 0, w2.x)

   -- calc control force
   local controlForce_x=self.pendX.DIPC:calcControlForce(xx,self.pendX.U, self.pendX.Q, self.pendX.R, self.pendX.K)
   local controlForce_z=self.pendZ.DIPC:calcControlForce(zx,self.pendZ.U, self.pendZ.Q, self.pendZ.R, self.pendZ.K)

   if ctrlForceScale then
      controlForce_x=controlForce_x*ctrlForceScale
      controlForce_z=controlForce_z*ctrlForceScale
   end
   
   if false then -- debug draw
      self.pendX.y:assign(xx:column(0):range(0,3)) 
      self.pendX:draw()
      -- self.pendZ.y:assign(zx:column(0):range(0,3)) 
      -- self.pendZ:draw()
   end

   local cf=vector3()
   cf.x=controlForce_x(0,0)
   cf.z=controlForce_z(0,0)
   cf.y=0
   

   local ct=vector3()
   ct.z=controlForce_x(1,0)*-1
   ct.x=controlForce_z(1,0)
   ct.y=0

--print(self.ori)
   cf:rotate(self.ori)
   ct:rotate(self.ori)
   

   controlForce:setAllValue(0)
   controlForce:set(0, cf.z)
   controlForce:set(1, cf.x)

   controlForce:set(7, ct.x)
   controlForce:set(9, ct.z)

   -- save projected 3D state to self.theta
   q:setRotation(theta)
   q2:setRotation(theta2)

   self.theta:set(1, pos.x)
   self.theta:set(0, pos.z)
   self.theta:setQuater(2, q)
   self.theta:setQuater(6, q:inverse()*q2)

   w2:rsub(w)
   w2:rotate(q2:inverse())
   w:rotate(q:inverse())

   self.dtheta:set(1, d.x)
   self.dtheta:set(0, d.z)
   self.dtheta:setVec3(3, w)
   self.dtheta:setVec3(7, w2)
end

function DIPC3d:oneStep()
   if debug_print then
      print('oneSteps')
   end

   
   local xx=self.pendX.x
   local zx=self.pendZ.x

   self:calcState2D(xx,zx, self.controlforce)


   -- provide projected state back to the simulator
   self.simulator:getLinkData(0, DynamicsSimulator.JOINT_VALUE, self.theta)
   self.simulator:getLinkData(0, DynamicsSimulator.JOINT_VELOCITY, self.dtheta)

   self.simulator:setLinkData(0, DynamicsSimulator.JOINT_TORQUE, self.controlforce)

   if self.simulator:stepSimulation()== false then
      self.errorOccurred=true
      print("error detected")
      fineLog("error detected")
--      debug.debug()
      return
   end


--		self.Y:resize(self.Y:rows()+1, 2)
   self.Y:resize(math.max(self.Y:rows(), self.numFrame+1), 10) -- do not delete predicted result
   self.numFrame=self.numFrame+1
   local lastRow=self.numFrame-1
   
   self.Y:row(lastRow):assign(self.theta)
end



function DIPC3d:fourSteps()

   local xx=self.pendX.x
   local zx=self.pendZ.x

   local controlForce=self:calcState2D(xx,zx)

   self.controlforce:setAllValue(0)
   self.controlforce:set(0, controlForce.z)
   self.controlforce:set(1, controlForce.x)

   self.simulator:setTimestep(self.dt*4)

   self.simulator:setLinkData(0, DynamicsSimulator.JOINT_TORQUE, self.controlforce)

   if self.simulator:stepSimulation()== false then
      self.errorOccurred=true
      print("error detected")
--      debug.debug()
      return
   end

   self.simulator:getLinkData(0, DynamicsSimulator.JOINT_VALUE, self.theta)
   self.simulator:getLinkData(0, DynamicsSimulator.JOINT_VELOCITY, self.dtheta)


   self:projectState()

   self.simulator:setTimestep(self.dt)

   --		self.Y:resize(self.Y:rows()+1, 2)

   assert(self.numFrame~=0)

   self.Y:resize(math.max(self.Y:rows(), self.numFrame+4), 6) -- do not delete predicted result
   self.numFrame=self.numFrame+4
   local prevRow=self.numFrame-5
   local lastRow=self.numFrame-1

   self.Y:row(lastRow):assign(self.theta)
   self.Y:row(lastRow-1):interpolate(0.75, self.Y:row(prevRow), self.theta)
   self.Y:row(lastRow-2):interpolate(0.5, self.Y:row(prevRow), self.theta)
   self.Y:row(lastRow-3):interpolate(0.25, self.Y:row(prevRow), self.theta)

end

function DIPC3d:predictTrajectory(numSteps)
   local theta=vectorn()
   local dtheta=vectorn()
   local numFrames=self:numFrames()
   
   self:_saveStates(theta, dtheta)
   
   for i=1,numSteps do
      self:oneStep()
   end
   
   return self:_restoreStates(theta, dtheta, numFrames)
end



function DIPC3d:_saveStates(theta, dtheta)
   self.simulator:getLinkData(0, DynamicsSimulator.JOINT_VALUE, theta)
   self.simulator:getLinkData(0, DynamicsSimulator.JOINT_VELOCITY, dtheta)
   assert(not self.errorOccurred)
end

function DIPC3d:_restoreStates(theta, dtheta, numFrames)
   --		local res=matrixn()
   --		local output=self.Y:range(numFrames, self.Y:rows(), 0, self.Y:cols())
   --
   --		res:setSize(output:rows(), output:cols())
   --		res:assign(output)
   
   -- restore states
   --		self.Y:resize(numFrames, self.Y:cols())
   self.errorOccurred=false
   self.numFrame=numFrames
   self.simulator:setLinkData(0, DynamicsSimulator.JOINT_VALUE, theta)
   self.simulator:setLinkData(0, DynamicsSimulator.JOINT_VELOCITY, dtheta)
   self.simulator:initSimulation()
   
   self.theta:assign(theta)
   self.dtheta:assign(dtheta)
   return res
end

function DIPC3d._getPosition(cartpole,frame)
   local v1=vector3()
   local q=quater()

   if frame==nil then
      frame=-1
   end
   cartpole:_getPosition2(frame, v1,q)
   return v1, q
end

function DIPC3d.__getPosition(theta)
   local v1=vector3()
   local q=quater()

   local posz=theta(0)
   local posx=theta(1)

   local vt=vector3(0,1,0)
   local vtt=theta:toQuater(2)*vt

   q:axisToAxis(vt, vtt)
   
   v1:setValue(posx,0,posz);
   return v1,q
end

function DIPC3d:_getPosition2(frame, v1,q)

   
   
   local theta
   if frame>=0 then
      theta=self.Y:row(frame)
   else
      theta=self.theta
   end


   local posz=theta(0)
   local posx=theta(1)

   local vt=vector3(0,1,0)
   local vtt=theta:toQuater(2)*vt

   q:axisToAxis(vt, vtt)
   
   v1:setValue(posx,0,posz);
   
   
end

function DIPC3d.__getPosition(theta)
   
   local v1=vector3()
   local q=quater()
   local posz=theta(0)
   local posx=theta(1)

   local vt=vector3(0,1,0)
   local vtt=theta:toQuater(2)*vt

   q:axisToAxis(vt, vtt)
   
   v1:setValue(posx,0,posz);
   
         
   return v1, q
end

function DIPC3d.__getCOMpos(pend, theta)
   local v1,q=DIPC3d.__getPosition(theta)
   local frame=transf()
   frame.rotation:assign(q)
   frame.translation:assign(v1)

   return frame:toGlobalPos(pend.pole:localCOM())   
end
function DIPC3d:calcLeaning()
   
   if self.theta:size()<6 then
      return 0
   end
   
   local q=quater()

   local vt=vector3(0,1,0)
   local vtt=self.theta:toQuater(2)*vt

   q:axisToAxis(vt, vtt)

   return q:rotationAngle()
end

function DIPC3d:calcCOMvel()
   
   
   -- local w=self:calcPoleAngVel()
   
   -- local v=self:calcCartVel()+w:cross(self.pole:localCOM())
   
   -- local v2=self.simulator:getWorldVelocity(0,self.pole, self.pole:localCOM())
   -- print("V==v2", v, v2)
   -- return v2


   return 0.5*( self.simulator:getWorldVelocity(0,self.pole1, self.pole1:localCOM())+
	     self.simulator:getWorldVelocity(0,self.pole2, self.pole2:localCOM()))
end

function DIPC3d:calcPoleAngVel()


   return self.dtheta:toVector3(3)

end

function DIPC3d:calcCartVel()
   return self.simulator:getWorldVelocity(0, self.cart, self.cart:localCOM())
end

function DIPC3d:calcCOMpos()
   return self.simulator:getWorldState(0):globalFrame(self.pole):toGlobalPos(self.pole:localCOM())
end
function DIPC3d:calcCartPos()
   return self.simulator:getWorldState(0):globalFrame(self.cart):toGlobalPos(self.cart:localCOM())
end

function DIPC3d:addExtForce(f)
   self.simulator:addForceToBone(0,self.pole, self.pole:localCOM(), f)
end

function DIPC3d:_calcTip(q)
   local qq=vector3()
   qq:rotate(q,vector3(0,self.pendX.l*2,0));
   return qq
end


function DIPC3d:draw()   
   self.skin:setPose(self.simulator,0)
end

function DIPC3d:drawFrames(startFrame, objectList)
   local lines=vector3N()
   for i=startFrame, self.numFrame-1 do
      local cart, poleOri=self:_getPosition(i)
      local pole=cart+self:_calcTip(poleOri)

      lines:pushBack(cart)
      lines:pushBack(pole)
   end

   if lines:rows()~=0 then
      objectList:registerObject("zmpPredict2", "LineList", "solidred", 
				lines:matView()*100,0)
   end
end

function DIPC3d:calcZMPCOM(iframe)
   local cart, poleOri=DIPC3d._getPosition(self, iframe)
   local pole=cart+self:_calcTip(poleOri)

   return cart, (cart+pole)/2
end

function DIPC3d:drawFrame(iframe)
   -- todo
end










