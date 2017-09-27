
require("RigidBodyWin/common")
require("RigidBodyWin/subRoutines/PDservo")
require("RigidBodyWin/subRoutines/IDservo")
require("RigidBodyWin/subRoutines/RagdollSim")

CalcFFtorque=LUAclass()

function CalcFFtorque:__init(loader, motdof, simulatorParam)

	self.loader=loader
	self.mot=motdof
	simulatorParam.simulationMode=SimulationMode.TrackMotion
	self.ragdollSim=RagdollSim:new(loader, true, self.mot, simulatorParam)
	self.ragdollSim.drawDebugInformation=true
	
end

function CalcFFtorque:calculate(first, last, initialHeight)
	local FFtorque=matrixn(last-first+1, self.mot:numDOF())
	local outputMotion=MotionDOF(self.loader.dofInfo)
	self.ragdollSim:setFrame(first-1, initialHeight)
	self.ragdollSim:frameMove(1)
	local simulFrameRate=1/self.ragdollSim.simulationParam.timestep
	local niter=math.round(simulFrameRate/120)
	print(first, last)
	for i=first, last do
		print(i)
		assert(i<self.mot:numFrames())
		self.ragdollSim:frameMove(niter)
		FFtorque:row(i-first):assign(self.ragdollSim.controlforce)
		assert(math.abs(self.ragdollSim.pdservo.currFrame-i)<0.0001)
		if math.mod(i,4)==0 then RE.renderOneFrame(true) end
	end
end
