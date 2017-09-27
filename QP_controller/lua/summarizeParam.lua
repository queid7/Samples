require("config")
package.projectPath='../Samples/QP_controller/'
package.path=package.path..";../Samples/QP_controller/lua/?.lua" --;"..package.path
package.path=package.path..";../Samples/classification/lua/?.lua" --;"..package.path

require("IPC_based/common")
require("IPC_based/useCases")


function ctor()

	targets={'handstand','walk3','backflip', 'roundoff2', 'straddle', 'popa2', 'justinStraightRun_nocart'}
	target_files={'handstand','walk3','backflip', 'roundoff2','straddle', 'popa', 'straightRun_nocart'}
	
	for i, target in ipairs(targets) do
		require('IPC_based/useCase_'..target_files[i])
		useCases.summarizeControlParam2(useCases[target].controlParam, 'cp_'..target)
		useCases.summarizeControlParam2(useCases[target],'cp_all'..target, true)
	end
	this('exit',0)
end
function dtor()
end

function onCallback(w, userData)
end

function renderOneFrame()
	noFrameMove=true
	RE.renderOneFrame(true)
	noFrameMove=nil
end
function onFrameChanged(currFrame)
end
function frameMove(fElapsedTime)
end
