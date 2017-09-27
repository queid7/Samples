function QPservo:verifyCalcJacobian()
			mat1=matrixn()
			mat2=matrixn()
			local dq=vectorn(self.dtheta:size()-1)
			dq:range(0,3):assign(self.dtheta:range(4,7)) dq:range(3,6):assign(self.dtheta:range(0,3)) dq:range(6,dq:size()):assign(self.dtheta:range(7,self.dtheta:size()))

			simulator:calcMomentumDotJacobian(0, mat1, mat2);
			local mmm=simulator:calcMomentumCOM(0)
			if false then -- verify calcMomentumCOMfromPose
				local pose1=simulator._lastSimulatedPose
				local pose2=pose1:copy()
				local tf2=MotionDOF.rootTransformation(pose1)
				print('tf2',tf2)
				local dt=1/120
				tf2:integrate(liegroup.se3(dq:toVector3(0), dq:toVector3(3)), dt)
				MotionDOF.setRootTransformation(pose2, tf2)
				pose2:range(7,pose2:size()):radd(dq:range(6,dq:size())*dt)

				local mmm2=simulator:calcMomentumCOMfromPose(0,dt,pose1,pose2)
				print(mmm)
				print(mmm2)
				-- these two should be similar. tested o.k.
			end
			local dotL=(mat1*ddq:column()+mat2*dq:column()):column(0):copy()

			local momentum=(mat1*dq:column()):column(0):copy()
			if prevMomentum then
				print('dotL', dotL/self.dtinv, momentum-prevMomentum)
				print('dotL2', dotL/self.dtinv, prevDotL)
				print('dotL3', (mat1*ddq:column()):column(0):copy()/self.dtinv, momentum-prevMomentum)
				-- test failed.
				--dbg.console()
			end

			-- simulator:calcCOMjacobian(0, mat1)
			local mm1=mat1:copy()
			local mm2=mat2:copy()
			simulator:calcCOMdotJacobian(0, mat1, mat2) -- com j and dj
			if true then
				local totalMass=simulator:calcTotalMass(0)
				-- verify linear momentum jacobians equals to comdotjacobian*mass
				print(mm1:sub(3,6),mat1*totalMass) -- tested o.k.
				print(mm2:sub(3,6),mat2*totalMass) -- tested o.k.

				local dotL=(mat1*ddq:column()+mat2*dq:column()):column(0):copy()
				if prevMomentum then
					print('dotL', dotL*(totalMass/self.dtinv), momentum-prevMomentum)
				end
			end

			if prevMomentum then
				prevDotL=momentum-prevMomentum
			end
			prevMomentum=momentum
			print('momentum:',prevMomentum , mmm)

			local v=simulator:calculateCOMvel(0)
			print('comvel:',v.x,v.y,v.z,'err:', CT.vec(v.x,v.y,v.z)-(mat1*dq:column()):column(0))
			local cdv2=(mat1*ddq:column()+mat2*dq:column()):column(0):copy()

			local ibone=simulator:skeleton(0):getTreeIndexByName('lfoot')
			simulator:calcJacobian(0,ibone,mat1)
			simulator:calcDotJacobian(0,ibone,mat2)
			local v=simulator:getWorldVelocity(0, simulator:skeleton(0):VRMLbone(ibone), vector3(0,0,0))
			local w=simulator:getWorldAngVel(0, simulator:skeleton(0):VRMLbone(ibone))
			print("v,w=",v,w)
			print(CT.vec(w.x,w.y,w.z,v.x,v.y,v.z)-(mat1*dq:column()):column(0))
			
			local mat3=matrixn()
			local mat4=matrixn()
			local lpos=vector3(0,0,0.2)
			simulator:calcBoneDotJacobian(0,ibone,lpos, mat3, mat4)
			local v3= simulator:getWorldVelocity(0, simulator:skeleton(0):VRMLbone(ibone), lpos)
			print("v3=", v3)
			print(CT.vec(v3.x, v3.y, v3.z)-(mat3*dq:column()):column(0))
			print("ddq=", ddq)
			dbg.console()
			simulator:stepKinematic(ddq, vectorn(), false)
			do
				local w=self.workspace
				if w.bases and  w.bases:size()>0 then
					-- local tau=w.x:range(self.numActualDOF, self.numActualDOF*2)
					-- local lambda=w.x:range(self.numActualDOF*2, w.x:size())
					-- simulator:getLCPmatrix(w.Mlcp, w.Mlcp_bias)
					-- print('contactacc1=', (w.Mlcp*lambda:column()+w.Mlcp_bias:column()):column(0)) -- should be larger than 0
					local dq=w.dq
					local ca=(w.VtJ2*ddq:column()+w.VtDotJ*dq:column()+w.dotVtJ*dq:column()):column(0)
					-- local ca2=(w.VtJ2*ddq:column()+w.VtDotJ*dq:column()):column(0)
					-- local ca3=(w.J*ddq:column()+w.dotJ*dq:column()):column(0)
					print('contactacc=', ca)
					-- print('contactacc2=', ca2)
					-- print('contactacc3=', ca3:extract(CT.colon(4, ca3:size(), 6)))
					for i=0, w.bases:size()-1 do
						local b=w.bases(i)
						local bone=simulator:skeleton(0):VRMLbone(b.ibone)
						io.write(bone:name() .." :")
						local  lpos=simulator:getWorldState(0):globalFrame(b.ibone):toLocalPos(b.globalpos)
						-- io.write('lpos=', tostring(lpos))
						io.write(tostring(simulator:getWorldAcceleration(0,simulator:skeleton(0):VRMLbone(b.ibone),lpos).y-9.8))
						-- io.write(' '..tostring(simulator:getWorldAcceleration(0,simulator:skeleton(0):VRMLbone(b.ibone),vector3(0,0,0)).y-9.8))
						-- io.write('\n')
					end
					print('contactacc2 min=', ca:minimum()) -- should be larger than 0
				end
			end
			local cdv=simulator:calculateCOMacc(0)
			print('comacc= ',CT.vec(cdv.x, cdv.y, cdv.z)-cdv2)
			local dv=simulator:getWorldAcceleration( 0, simulator:skeleton(0):VRMLbone(ibone), vector3(0,0,0))
			local dw=simulator:getWorldAngAcc( 0, simulator:skeleton(0):VRMLbone(ibone))
			print("dv,dw=",dv,dw)
			local dwv=CT.vec(dw.x,dw.y,dw.z,dv.x,dv.y-9.8,dv.z)
			-- print((mat1*ddq:column()):column(0))
			local acc=(mat1*ddq:column()):column(0)+(mat2*dq:column()):column(0)
			print(acc-dwv)
			local dv3= simulator:getWorldAcceleration(0, simulator:skeleton(0):VRMLbone(ibone), lpos)
			local acc=(mat3*ddq:column()):column(0)+(mat4*dq:column()):column(0)
			print("v3=",CT.vec(v3.x, v3.y, v3.z)-(mat3*dq:column()):column(0))
			print("dv3=", CT.vec(dv3.x, dv3.y-9.8, dv3.z)-acc)
end
function QPservo:addMomentumObjective_additional(sim_unused,desiredDotAngMomentum, desiredDotLinMomentum,weight_ang, weight_lin)
		if false then
			local footJ=matrixn()
			simulator:calcJacobian(0, simulator:skeleton(0):getTreeIndexByVoca(MotionLoader.RIGHTANKLE), footJ)
			footJ:sub(0,6,0,6):setAllValue(0)
			J:sub(0,6,0,6):setAllValue(0)
			DJ:sub(0,6,0,6):setAllValue(0)
			J=J*math.nullspaceProjector(footJ)
		end

		if false then
			local pose=vectorn()
			local footLoop=self.footLoop
			local skel=footLoop.skel
			local sim=footLoop.sim
			skel:convertSourcePose(self.theta, pose)
			sim:setLinkData(0, Physics.DynamicsSimulator.JOINT_VALUE,pose)
			sim:initSimulation()
			local footJ=matrixn(6, skel.dofInfo:numActualDOF())
			local footJ2=matrixn()
			sim:calcJacobian(0, footLoop.Rindex, footJ2)
			--footJ2:sub(0,6,0,6):setAllValue(0)
			for i=0,5 do
				skel:convertDQexceptRoot(footJ2:row(i), footJ:row(i))
			end
			footJ:sub(0,6,0,6):setAllValue(0)
			J=J*math.nullspaceProjector(footJ)
		end

		if false then
			-- exclude neck joint  for momentum control
			local skel=simulator:skeleton(0)

			local headIndex=skel:getTreeIndexByVoca(MotionLoader.HEAD)
			local function crop(headIndex)
				local sR=skel.dofInfo:startR(headIndex)
				local eR=skel.dofInfo:endR(headIndex)
				--headBone=skel:VRMLbone(headIndex)
				for i=sR, eR-1 do
					J:column(i-1):setAllValue(0)
					--DJ:column(i-1):setAllValue(0)
				end
			end

			crop(headIndex)
			crop(skel:VRMLbone(headIndex):parent():treeIndex())
		end
end
function QPservo:testMassMatrix()
		local M=matrixn()
		local b=vectorn(w.b:size())
		simulator:calcMassMatrix2(0, M, b)
		if M:isSimilar(w.M) then
			print('M==w.M')
		else
			print('M!=w.M')
			assert(false)
		end

		if b:row():isSimilar(w.b:row()) then
			print('b==w.b')
		else
			print('b!=w.b')
		end
end
function QPservo:stepSimul_old(sim, impulse)
	local simulator=self.simulator
	if false then
		simulator:setLinkData(0, Physics.DynamicsSimulator.JOINT_TORQUE, self.controlforce)
		simulator:stepSimulation()
	elseif false then
		-- forward dynamics
		local w=self.workspace
		local tau=vectorn(w.b:size())
		tau:setAllValue(0)
		if w.link_pair_count >0 then
			local solution=vectorn()
			simulator:getLCPsolution(solution)
			tau:assign((w.JtV*solution:column()):column(0))
		end
		tau:radd(self.controlforce:range(1,self.controlforce:size()))
		local nb=w.b*-1+tau
		local ddq=w.M:LeftDiv(nb:column()):column(0)
		simulator:stepKinematic(ddq, tau, true)
	elseif false then
		-- forward dynamics using QP solution
		local w=self.workspace
		local tau=vectorn(w.b:size())
		tau:setAllValue(0)
		if w.link_pair_count >0 then
			local solution=w.x:range(self.numActualDOF*2, w.x:size())
			tau:assign((w.JtV*solution:column()):column(0))
		end
		tau:radd(self.controlforce:range(1,self.controlforce:size()))
		local nb=w.b*-1+tau
		local ddq=w.M:LeftDiv(nb:column()):column(0)
		
		--local lambda=w.x:range(self.numActualDOF*2, w.x:size())
		--if lambda:size()>0 then
		--	local tau2=w.x:range(self.numActualDOF, self.numActualDOF*2)
		--	print('tau err:', tau, tau2, (w.JtV*lambda:column()):column(0))
		--	local ddq2=w.x:range(0, self.numActualDOF)
		--	print('ddq err:', ddq-ddq2)
		--	dbg.console()
		--end

		simulator:stepKinematic(ddq, tau, true)
	end
end


function QPservo:verifyQPsolution()
	if false then
		local tau=w.x:range(self.numActualDOF, self.numActualDOF*2)
		local lambda=w.x:range(self.numActualDOF*2, w.x:size())
		if lambda:size()>0 then
			local tau1= w.M*ddq:column()+w.b:column()-w.JtV*lambda:column()
			print('tau=',tau)
			print('lambda=',lambda) -- should be larger than 0
			print('lambda min=', lambda:minimum())
			if self.useMLCPmatFromAIST then
				local ca=(w.Mlcp*lambda:column()+w.Mlcp_bias:column()):column(0)
				print('contactacc=', ca) -- should be larger than 0
				print('contactacc min=', ca:minimum()) -- should be larger than 0
			end
			print('tauerr=', tau1:column(0)-tau) -- should be 0
			print('ddq, desiredacc=', ddq, self.desiredacceleration:range(7,self.desiredacceleration:size())) -- should be similar
			print(ddq:range(6,ddq:size())- self.desiredacceleration:range(7,self.desiredacceleration:size())) -- should be similar

			-- all tested o.k.
			--dbg.console()
		else
		end
	end
	if false then -- verify calcJacobian
		self:verifyCalcJacobian()
	end
end
