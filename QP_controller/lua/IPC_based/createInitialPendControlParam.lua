require("config")
package.projectPath='../Samples/QP_controller/'
package.path=package.path..";../Samples/QP_controller/lua/?.lua" --;"..package.path
package.path=package.path..";../Samples/classification/lua/?.lua" --;"..package.path
require('subRoutines/consoleLib')
require("IPC_based/common")
require("IPC_based/useCases")

function main()
	if #useCase.grpNames==1 then
		grpName=useCase.grpNames[1]
	end

	if grpName==nil then
		print("Error! Please specify grpName. (lua short.lua createInitialPendControlparam  grpName=\\'adsf\\')")
		this('exit',1)
		return
	end
	print('grpName=',grpName)
	local pcp="pendControlParam={"

	local default
	if useCase.keyframes.pendDesiredVel then
		default=useCase.keyframes.pendDesiredVel.default
	else
		default= vector3(0,0,3.0)
		print("useCase.keyframes.pendDesiredVel.default doesn't exist")
	end

	for i,v in ipairs(useCase.segmentations[grpName].names) do
		pcp=pcp.."['keyframe,0,pendDesiredVel,"..grpName..","..v..",z']="..default.z..","
		pcp=pcp.."['keyframe,0,pendDesiredVel,"..grpName..","..v..",x']="..default.x..","
	end
	for i=0,1 do
		local grpName='ignore'
		local v=tostring(i)
		pcp=pcp.."['keyframe,0,pendDesiredVel,"..grpName..","..v..",z']="..default.z..","
		pcp=pcp.."['keyframe,0,pendDesiredVel,"..grpName..","..v..",x']="..default.x..","
	end
	pcp=pcp.."},\n"
	pcp=pcp.."pendOptimizationPath=\n{\n	firstFrames={"
	for i, v in ipairs(useCase.segmentations[grpName].firstFrames) do
		pcp=pcp..tostring(v)..","
	end
	pcp=pcp.."},\n	segments={'ignore,0',"
	for i, v in ipairs(useCase.segmentations[grpName].names) do
		pcp=pcp.."'"..grpName..","..v.."',"
	end
	pcp=pcp.."'ignore,1'},\n},"

	pcp=[[
		-- the initial pendControlParam and pendOptimizationPath can be automatically generated by using createInitialPendControlParam.lua
	]]..pcp
	util.writeFile('temp.lua', pcp)
	print("output to temp.lua")
end

