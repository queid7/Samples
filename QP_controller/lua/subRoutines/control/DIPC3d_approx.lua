require("RigidBodyWin/subRoutines/control/SDRE")

function DIPC3d_cpp:getPosition(frame)
   local theta=self:getState(frame)
   local v=vector3()
   local q1=quater()
   local q2=quater()
   self:__getPosition(theta, v, q1, q2)
   return v, q1, q2
end

function DIPC3d_cpp:getState(frame)
   local theta
   if frame then
      theta=vectorn()
      self:_getState(frame, theta)
   else
      theta=self.theta
   end
   return theta
end

function DIPC3d_cpp:calcZMPCOM(iframe)
   
   local theta=self:getState(iframe)

   local COM=self:__calcCOMpos(theta)   
   local cart=self.__calcCartPos(theta)
   return cart,COM
end

function DIPC3d_cpp:drawFrames()
end

class 'DIPC3d'

function DIPC3d:__deinit()
   if self.skin~=nil then
      RE.remove(self.skin)
      self.skin=nil
   end
end

function DIPC3d:__finalize()
   self.skin=nil
   self.loader=nil
end

function DIPC3d:__init(filename, b,g,dt,q)
   self.dt=dt
   self.loader=VRMLloader(filename)
   self.loader:printHierarchy()
   
   self.skin=RE.createVRMLskin(self.loader, false)
   
   self.skin:scale(100,100,100)
    
   self.theta=vectorn()
   self.loader:getPoseDOF(self.theta)

   if ragdollTest then
      self.theta:setQuater(2,quater(math.rad(170), vector3(0,0,1))) 
   end
   self.skin:setPoseDOF(self.theta)
   

   self.dtheta=vectorn(9) -- (v, w1, w2)
   self.dtheta:setAllValue(0)
      
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

   if true then -- lua implementation
      self.pendX=DIPCview(M, m1, m2, l1, l2, i1,i2,dt,q)
      self.pendZ=DIPCview(M, m1, m2, l1, l2, i1,i2,dt,q)
   else -- cpp porting. (faster)
      self.pendX=DIPCview_cpp(M, m1, m2, l1, l2, i1,i2,dt,q)
      self.pendZ=DIPCview_cpp(M, m1, m2, l1, l2, i1,i2,dt,q)
   end
   self.dt=dt
   self.ori=quater()	-- cart orientation
   self.ori:setValue(1,0,0,0)
   self.desiredVel=vector3(0,0,0)
   self.desiredVelGlobal=vector3(0,0,0)
   self.desiredPos=vector3(0,0,0)
   self.useDesiredPos=false
   self.desiredHip=vector3(0,0,0)
   self.desiredHipGlobal=vector3(0,0,0)
   self.controlforce=vectorn(10)
   self.Y=matrixn()
   self.numFrame=0
end

function DIPC3d:setOrientation2(q)
--   print(q)
   self.ori:assign(q)
   self:setDesiredVelocity(self.desiredVelGlobal, self.desiredHipGlobal)
--self:setDesiredVelocity(self.desiredVel)
end

function DIPC3d:setOrientation(angle)
   self:setOrientation2(quater(math.rad(angle), vector3(0,1,0)))
end

function projectQuaternion(q)
   local qoffset=quater()
   local qaxis=quater()

   q:decomposeNoTwistTimesTwist(vector3(0,1,0), qoffset, qaxis)
   q:assign(qoffset)
end


function DIPC3d:setDesiredVelocity(vel, hip) -- vector3

   self.desiredVelGlobal:assign(vel)
   self.desiredVel:assign(vel)

   if not hip then
      hip=vector3(0,0,0)
   end
   self.desiredHipGlobal:assign(hip)
   self.desiredHip:assign(hip)
   --   print(self.ori)
   self.desiredVel:rotate(self.ori:inverse())
   self.desiredHip:rotate(self.ori:inverse())
   self.pendX:setParam(self.desiredVel.x, -self.desiredHip.z)
   self.pendZ:setParam(self.desiredVel.z, self.desiredHip.x)
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

function DIPC3d.__calcCartPos(theta)
   local pos=vector3()
   pos.x=theta(1)
   pos.y=0
   pos.z=theta(0)

   return pos
end

function DIPC3d:calcCartPos()
   local pos=vector3()
   pos.x=self.theta(1)
   pos.y=0
   pos.z=self.theta(0)

   return pos
end

function DIPC3d:__step(ntimes)
   local xx=self.pendX.x
   local zx=self.pendZ.x

   local pos=self:calcCartPos()
   local d=self.dtheta:toVector3(0)


   local q=self.theta:toQuater(2)
   local q2=q*self.theta:toQuater(6)

   -- w=q*W
   -- w2=q*W+q2*w2 where W and W2 are local angular velocities.
   local w=self.dtheta:toVector3(3)

--   dbg.console()
   w:rotate(q)
   local w2=self.dtheta:toVector3(6)
   w2:rotate(q2)
   w2:radd(w)

   projectQuaternion(q)
   projectQuaternion(q2)

   local theta=q:rotationVector()
   local theta2=q2:rotationVector()

   local toLocal=self.ori:inverse()
   RE.output2("qtheta", theta)
   RE.output2("qtheta2", theta2)

   pos:rotate(toLocal)
   local ldesiredPos=self.desiredPos:copy()
   ldesiredPos:rotate(toLocal)
   w:rotate(toLocal)
   w2:rotate(toLocal)
   theta:rotate(toLocal)
   theta2:rotate(toLocal)
   d:rotate(toLocal)

   if pos.x~=pos.x then debug.debug() end

--   if self.pendX.useBogdanov04 then
   if true  then
      xx:set(0, 0, pos.x)
      xx:set(1, 0, -theta.z)
      xx:set(2, 0, -theta2.z)
      xx:set(3, 0, d.x)
      xx:set(4, 0, -w.z)
      xx:set(5, 0, -w2.z)
      
      --   RE.output("x_v", tostring(posx).." "..tostring(posz) .." "..tostring(dx).." "..tostring(dz))
      RE.output("theta_w", tostring(theta).." "..tostring(theta2))
      zx:set(0, 0, pos.z)
      zx:set(1, 0, theta.x)
      zx:set(2, 0, theta2.x)
      zx:set(3, 0, d.z)
      zx:set(4, 0, w.x)
      zx:set(5, 0, w2.x)

	  if self.useDesiredPos then
		  self.pendX:setParamPos(ldesiredPos.x, -self.desiredHip.z)
		  self.pendZ:setParamPos(ldesiredPos.z, self.desiredHip.x)
		  self.pendX.useDesiredPos=true
		  self.pendZ.useDesiredPos=true
	  elseif self.pendX.useDesiredPos==true then

		  self.pendX.useDesiredPos=false
		  self.pendZ.useDesiredPos=false
		  -- restore settings
		  self.pendX:setQ(self.pendX.q)
		  self.pendZ:setQ(self.pendZ.q)
		  self.pendX:setParam(self.desiredVel.x, -self.desiredHip.z)
		  self.pendZ:setParam(self.desiredVel.z, self.desiredHip.x)
	  end
   else
      
      xx:set(0, 0, pos.x)
      xx:set(1, 0, -theta.z)
      xx:set(2, 0, -theta2.z+theta.z)
      xx:set(3, 0, d.x)
      xx:set(4, 0, -w.z)
      xx:set(5, 0, -w2.z+w.z)
      
      --   RE.output("x_v", tostring(posx).." "..tostring(posz) .." "..tostring(dx).." "..tostring(dz))
      RE.output("theta_w", tostring(theta).." "..tostring(theta2))
      zx:set(0, 0, pos.z)
      zx:set(1, 0, theta.x)
      zx:set(2, 0, theta2.x-theta.x)
      zx:set(3, 0, d.z)
      zx:set(4, 0, w.x)
      zx:set(5, 0, w2.x-w.x)
   end

   local prevDt=   self.dt
   self.pendX.dt=prevDt*ntimes
   self.pendZ.dt=prevDt*ntimes

   --##testmw   if g_debugOneStep then
   --##testmw      g_debugOneStep:pushBack({"oneStep", xx:copy(), zx:copy()})
   --##testmw   end
   self.pendX:oneStep()
   self.pendZ:oneStep()
   --##testmw   if g_debugOneStep then
   --##testmw      g_debugOneStep:pushBack({"oneStep2", xx:copy(), zx:copy()})
   --##testmw   end
   self.pendX.dt=prevDt
   self.pendZ.dt=prevDt

--   if self.pendX.useBogdanov04 then
   if true then
      -- convert back to 3d states
      pos.x= xx(0, 0 )
      theta.z= -xx(1, 0 )
      theta2.z= -xx(2, 0 )
      d.x= xx(3, 0 )
      w.z= -xx(4, 0 )
      w2.z= -xx(5, 0 )

      pos.z= zx(0, 0 )
      theta.x= zx(1, 0 )
      theta2.x= zx(2, 0 )
      d.z= zx(3, 0 )
      w.x= zx(4, 0 )
      w2.x= zx(5, 0 )
   else
      pos.x= xx(0, 0 )
      theta.z= -xx(1, 0 )
      theta2.z= -xx(2, 0 )+theta.z
      d.x= xx(3, 0 )
      w.z= -xx(4, 0 )
      w2.z= -xx(5, 0 )+w.z

      pos.z= zx(0, 0 )
      theta.x= zx(1, 0 )
      theta2.x= zx(2, 0 )+theta.x
      d.z= zx(3, 0 )
      w.x= zx(4, 0 )
      w2.x= zx(5, 0 )+w.x

   end

   local toGlobal=self.ori
   pos:rotate(toGlobal)
   w:rotate(toGlobal)
   w2:rotate(toGlobal)
   theta:rotate(toGlobal)
   theta2:rotate(toGlobal)
   d:rotate(toGlobal)

   -- calc control force
   -- local cf=vector3()
   -- cf.x=controlForce_x(0,0)
   -- cf.z=controlForce_z(0,0)
   -- cf.y=0

   -- local ct=vector3()
   -- ct.z=controlForce_x(1,0)*-1
   -- ct.x=controlForce_z(1,0)
   -- ct.y=0

   if false then -- debug draw
      self.pendX.y:assign(xx:column(0):range(0,3)) 
      self.pendX:draw()
      -- self.pendZ.y:assign(zx:column(0):range(0,3)) 
      -- self.pendZ:draw()
   end
   
   q:setRotation(theta)
   q2:setRotation(theta2)

   self.theta:set(1, pos.x)
   self.theta:set(0, pos.z)
   self.theta:setQuater(2, q)
   self.theta:setQuater(6, q:inverse()*q2)

   -- w=q*W
   -- w2=q*W+q2*w2 where W and W2 are local angular velocities.
   --> W2=q2:inverse()*(w2-w)
   --> W=q:inverse()*w

   w2:rsub(w)
   w2:rotate(q2:inverse())
   w:rotate(q:inverse())

   self.dtheta:setVec3(0, d)
   self.dtheta:setVec3(3, w)
   self.dtheta:setVec3(6, w2)

   RE.output2("theta", self.theta)
   RE.output2("dtheta", self.dtheta)
   RE.output2("cf", self.controlforce)
   RE.output2("theta2", self.theta)
   --##testmw   if g_debugOneStep then
   --##testmw      g_debugOneStep:pushBack({"oneStep3", self.theta:copy(), self.dtheta:copy()})
   --##testmw   end

end
function DIPC3d:oneStep()
   
   self:__step(1)

--		self.Y:resize(self.Y:rows()+1, 2)
   self.Y:resize(math.max(self.Y:rows(), self.numFrame+1), 10) -- do not delete predicted result
   self.numFrame=self.numFrame+1
   local lastRow=self.numFrame-1
   
   self.Y:row(lastRow):assign(self.theta)

   --##testmw   if g_debugOneStep then
   --##testmw      g_debugOneStep:pushBack({"oneStep4", self.theta:copy(), self.dtheta:copy()})
   --##testmw   end


end



function DIPC3d:fourSteps()

   if debug_print then
      print('fourSteps')
   end

   self:__step(4)

--		self.Y:resize(self.Y:rows()+1, 2)
   self.Y:resize(math.max(self.Y:rows(), self.numFrame+4), 10) -- do not delete predicted result
   self.numFrame=self.numFrame+4
   local prevRow=math.max(self.numFrame-5,0)
   local lastRow=self.numFrame-1
   
   self.Y:row(lastRow):assign(self.theta)
   self.Y:row(lastRow-1):interpolate(0.75, self.Y:row(prevRow), self.theta)
   self.Y:row(lastRow-2):interpolate(0.5, self.Y:row(prevRow), self.theta)
   self.Y:row(lastRow-3):interpolate(0.25, self.Y:row(prevRow), self.theta)

--for i=1,4 do self:oneStep() end

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
   theta:assign(self.theta)
   dtheta:assign(self.dtheta)
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
   self.theta:assign(theta)
   self.dtheta:assign(dtheta)
   return res
end


function DIPC3d:calcLeaning()
   local com=self:calcCOMpos()   
   local zmp=self:calcCartPos()
   
   local q=quater()

   local vt=vector3(0,1,0)
   local vtt=com-zmp

   q:axisToAxis(vt, vtt)

   return q:rotationAngle()
end

function DIPC3d:calcPoleAngVel()
   local q=self.theta:toQuater(2)
   local q2=q*self.theta:toQuater(6)
   local w=self.dtheta:toVector3(3)

   w:rotate(q)
   local w2=self.dtheta:toVector3(6)
   w2:rotate(q2)
   w2:radd(w)

   return q, q2, w, w2
end

function DIPC3d:calcCOMvel()
   local q1,q2, w1, w2=self:calcPoleAngVel()
   local v1=w1:cross(self.pole1:localCOM())
   local v2=v1*2+w2:cross(self.pole2:localCOM())

   local m1=self.pole1:mass()
   local m2=self.pole2:mass()

   local comV=self:calcCartVel()+(v1*m1+v2*m2)/(m1+m2)

   return comV
end

function DIPC3d:calcCartVel()
   return self.dtheta:toVector3(0)
end

function DIPC3d:__calcCOMpos(theta)
   local q1=theta:toQuater(2)
   local q2=q1*theta:toQuater(6)
   local v0=vector3(theta(1), 0, theta(0)) -- self.__calcCartPos(theta)
   local v1=vector3()
   local v2=vector3()
   v1:rotate(q1, self.pole1:localCOM())
   v2:rotate(q2,self.pole2:localCOM())

   local m1=self.pole1:mass()
   local m2=self.pole2:mass()

   return ((v0+v1)*m1+ (v0+v1*2+v2)*m2)/(m1+m2)

end

function DIPC3d:__calcHipsPos(theta)
   local q1=theta:toQuater(2)
   local v0=vector3(theta(1), 0, theta(0)) -- self.__calcCartPos(theta)
   local v1=vector3()
   v1:rotate(q1, self.pole2:getOffset())

   return v0+v1
end

function DIPC3d:calcCOMpos()
   return self:__calcCOMpos(self.theta)
end

function DIPC3d:calcCOMposFromSkel()
   local m1=self.pole1:mass()
   local m2=self.pole2:mass()
   return (self.pole1:getFrame():toGlobalPos(self.pole1:localCOM())*m1+self.pole2:getFrame():toGlobalPos(self.pole2:localCOM())*m2)/(m1+m2)

end

function DIPC3d:addExtForce(f)
   self.simulator:addForceToBone(0,self.pole, self.pole:localCOM(), f)
end

function DIPC3d.__getPosition(theta)
   local v1=vector3()

   v1.x=theta(1)
   v1.y=0
   v1.z=theta(0)
   
   local q=theta:toQuater(2)
   local q2=theta:toQuater(6)
   return v1, q, q2
end


function DIPC3d.__getState(v1, q, q2)
   local theta=vectorn(10)
   theta:set(0, v1.z)
   theta:set(1, v1.x)

   theta:setQuater(2, q)
   theta:setQuater(6, q2)
   return theta
end

function DIPC3d:setState(v1,q,q2, dv1, dq, dq2)
   self.theta:set(0, v1.z)
   self.theta:set(1, v1.x)
   self.theta:setQuater(2, q)
   self.theta:setQuater(6, q2)
   self.dtheta:setVec3(0, dv1)
   self.dtheta:setVec3(3, dq)
   self.dtheta:setVec3(6, dq2)
end

function DIPC3d:getState(frame)
   local theta
   if frame then
      theta=self.Y:row(frame)
   else
      theta=self.theta
   end
   return theta
end

function DIPC3d:getPosition(frame)
   return self.__getPosition(self:getState(frame))
end

function DIPC3d:setSkeleton(frame)
   self.loader:setPoseDOF(self:getState(frame))
end

function DIPC3d:draw()   
   self.skin:setPoseDOF(self.theta)
end

function DIPC3d:drawFrames(startFrame, objectList)
end

function DIPC3d:calcZMPCOM(iframe)
   
   local theta=self:getState(iframe)

   local COM=self:__calcCOMpos(theta)   
   local cart=self.__calcCartPos(theta)
   return cart,COM
end
