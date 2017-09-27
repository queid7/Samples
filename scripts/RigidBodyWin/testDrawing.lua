require("config")
require("module")
require("common")

function ctor()
	this:create("Button", "button1", "button1");
--	this:create("Value_Slider", "slidery", "slidery");
--	this:widget(0):sliderRange(0,0.5);
--	this:widget(0):sliderValue(0.1);
	this:updateLayout();

	do 
		-- 디버그용으로 사용하기 쉽게 만든 함수들
		dbg.namedDraw("Sphere", vector3(100,0,0), "x")
		dbg.draw("Sphere", vector3(0,100,0), "y") -- named draw와 달리 이름표가 그려지지 않는다.

		-- quater는 회전변환. quater(1,0,0,0) 는 identity. (회전안함)
		dbg.namedDraw("Axes",transf(quater(1,0,0,0), vector3(0,0,100)), "axes")
		dbg.namedDraw("Coordinate", transf(quater(1,0,0,0), vector3(0,0,2)), "coord") -- "Axes"와 같은 기능이지만, meter단위계를 사용함.


		dbg.draw('Entity', "sphere1010.mesh", vector3(0,0,0), "origin") -- 아무 메시나 그리기
		dbg.draw('Entity', "h11.mesh", vector3(200,100,0), "house") -- 아무 메시나 그리기
	end

	do
		-- Ogre의 원래 모양에 충실하게 구현
		local node =RE.ogreRootSceneNode()
		local bgnode =node:createChildSceneNode("Node1")
		local ent1= RE.ogreSceneManager():createEntity("pave_entity", "h12.mesh")
		bgnode:attachObject(ent1)
		bgnode:translate(-230,0,232)
	end
	print('ctor')
end
function frameMove()
end

function onCallback(w, userData)
   if w:id()=="button1" then
	   print("button1\n");
   elseif w:id()=="button2" then
	   print("button2\n");
   elseif w:id()=="sliderx"or w:id()=="slidery" then
   end
   print('oncallback')
end

function dtor()
		print('dtor')
end

