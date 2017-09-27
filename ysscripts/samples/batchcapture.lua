require("config")

package.projectPath='../Samples/QP_controller/'
package.path=package.path..";../Samples/QP_controller/lua/?.lua" --;"..package.path
package.path=package.path..";../Samples/classification/lua/?.lua" --;"..package.path
package.path=package.path..";../Samples/ysscripts/samples/?.lua" --;"..package.path
package.path=package.path..";../Samples/ysscripts/lib/?.lua" --;"..package.path

require('driverDynamicMuscle')

ctor_old=ctor

g_cap_met = false
g_cap_eng = false
g_cap_eff = false
g_cap_spd = false

function ctor()
	ctor_old()

	local captureUntil=1200
	--local captureUntil=100
	--local captureUntil=50
	local tryFunc=function(captureUntil)
		mEventReceiver:onFrameChanged(win, captureUntil)
	end

	local output={pcall(tryFunc, captureUntil)}
	if output[1]==false then
		print(output[2])
	end

	g_capmode=g_capmode or {'spd'}
	prefix=prefix or 'dump'
	--capmode
	for i=1,#g_capmode do
		if g_capmode[i]=='met' then
			g_cap_met = true
		elseif g_capmode[i]=='eng' then
			g_cap_eng = true
		elseif g_capmode[i]=='eff' then
			g_cap_eff = true
		elseif g_capmode[i]=='spd' then
			g_cap_spd = true
		end
	end

	--capture side view
	setCamVPos(30,100,300)	--side view
	mEventReceiver:attachCamera()
	RE.renderer():setScreenshotPrefix('../dump/'..prefix..'__side')
	RE.renderer():screenshot(true)
	for i=0,g_simulated_ifr-1 , 4 do
		mEventReceiver:onFrameChanged(win,i)
		RE.renderOneFrame(false)
	end
	RE.renderer():screenshot(false)

	--capture front view
	setCamVPos(300,100,30)	--front view
	mEventReceiver:attachCamera()
	RE.renderer():setScreenshotPrefix('../dump/'..prefix..'__front')
	RE.renderer():screenshot(true)
	for i=0,g_simulated_ifr-1 , 4 do
		mEventReceiver:onFrameChanged(win,i)
		RE.renderOneFrame(false)
	end
	RE.renderer():screenshot(false)

	--capture push view
	if string.find(g_mode, 'push')~=nil then
		setCamVPos(300,200,200)	--push view
		mEventReceiver:attachCamera()
		RE.renderer():setScreenshotPrefix('../dump/'..prefix..'__push')
		RE.renderer():screenshot(true)
		for i=0,g_simulated_ifr-1 , 4 do
			mEventReceiver:onFrameChanged(win,i)
			RE.renderOneFrame(false)
		end
		RE.renderer():screenshot(false)
	end

	this('exit!',0)
end
