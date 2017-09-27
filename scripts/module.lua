assert=nil

-- derived class doesn't automatically support parent class's member function unless it is derived from luabind_base (and I don't use luabind_base to simply dependencies..)
function defineDerived(super, derivedClasses, functionNames)
	for ii, derived in ipairs(derivedClasses) do
		for i,v in ipairs(functionNames) do
			derived[v]=super[v]
		end
	end
end




--[[
print_old=print
function print(...)
	if {...}[1]==nil then dbg.console() end
	print_old(...)
end
]]--
function assert(bVal)
   if not bVal then
      -- print("assert failed")
      -- dbg.callstack(-1)
      -- debug.debug()

      -- mpi

      if fineLog~=nil then
		  debug.sethook() -- stop all kinds of debugger
		  fineLog("assert failed")
		  fineLog(dbg.callstackString(3))
		  fineLog(util.tostring(dbg.locals()))
      end
      print("assert failed: type dbg.traceBack() or dbg.callstack() for more information.")
--      dbg.callstack()
--      debug.debug()
      dbg.console() -- my debugger.
   end
   return bVal
end

function convertMotionToMotDOF(srcSkeleton, srcMot, tgtSkeleton)
	local motdof= MotionDOF(tgtSkeleton.dofInfo)
	motdof:resize(srcMot:numFrames())

	local pt=MotionUtil.PoseTransfer(srcSkeleton, tgtSkeleton)
	for i=0, srcMot:numFrames()-1 do
		pt:setTargetSkeleton(srcMot:pose(i))
		tgtSkeleton:getPoseDOF(motdof:row(i))
	end
	return motdof
end
require("RigidBodyWin/subRoutines/pickle")

__globals={
	EQ_THR=0.000001
}-- keep seldomly used globals here.

-- LUAclass method is for those who prefer tables instead of userdata for representing object. 
-- So this is a simple wrapper for the metatable functionality.

-- usage: class definition>>>
--
--        MotionLoader=LUAclass()
--
--   ...  MotionLoader:__init(a,b,c)

--        VRMLloader=LUAclass(MotionLoader)
--   ...  VRMLloader:__init(a,b,c)
--              MotionLoader.__init(self,a,b,c)
--        end
-- usage: instantiation>>>
--
--  loader=VRMLloader:new(a,b,c) 
--[[
function LUAclass(baseClass)

   
   local classobj={}
   if __classMTs==nil then
      __classMTs={}
      __classMTs.N=0
   end
   
   __classMTs.N=__classMTs.N+1
   local classId=__classMTs.N
   __classMTs[classId]={__index=classobj}
   classobj.__classId=classId

   classobj.new=function (classobj, ...)
		   local new_inst={}
		   setmetatable(new_inst, __classMTs[classobj.__classId])
		   new_inst:__init(unpack({...}))
		   return new_inst
		end
   if baseClass~=nil then
      setmetatable(classobj, {__index=baseClass})
   end

   return classobj	 
end
function LUAclass_getmetatable(classobj)
	return __classMTs[classobj.__classId]
end
]]
function LUAdelete(classInstance)
	-- simply calls __finalize member function 
	-- (sometimes manual cleanup is needed even in LUA for garbage collection to work correctly. e.g. to break cyclic dependency)
	
   if classInstance and classInstance.__finalize~=nil then
      classInstance:__finalize()
   end
end


function printTable(t, bPrintUserData)
   

   for k,v in pairsByKeys(t) do
      local tv=type(v)
      if tv=="string" or tv=="number" or tv=="boolean" then
	 print(k, v)
      elseif tv=="userdata" then
	 if bPrintUserData==true then
	    print(k, tv, v)
	 else
	    print(k, tv)
	 end
      elseif tv=="table" then
	 kk=pickle(v,80)
	 if string.len(kk)<80 then
	    print(k, kk)
	 else
	    print(k, tv)
	 end
      else
	 print(k,tv)
      end
   end
end


function out(t)
   os.print(t)
end

function util.tostring(...)
   return util.mergeString({...})
end
function util.printFile(fn,...)
   util.outputToFile(fn, util.mergeString({...}))
end

function util.printInfo(t)
   
   local info=class_info(t)
   print("methods:")
   local out=""
   for k,v in pairs(info.methods) do
      out=out.." "..k
   end
   
   print(out)
   print("attributes:")
   out=""
   for k,v in pairs(info.attributes) do
      out=out.." "..v
   end
   
   print(out)
end
-- conversion between c++ index style and lua index style.
function set1(a, b, c, d)
   if d==nil then
	   assert(c)
      a:set(b-1,c)
   else
      a:set(b-1,c-1,d)
   end
end

function get1(a, b, c)
   if c==nil then
      return a:get(b-1)
   else
      return a:get(b-1,c-1)
   end
end

function range_c(a, first,last)
   return a:range(first, last+1)
end

function isnan(t)
   if t<t==false and t>t==false and t~=t then
      return true
   end
   return false
end

function RE.output(str, ...)
	RE._output(str, util.mergeStringShort({...}),1)
end

function RE.output2(str, ...)
	RE._output(str, util.mergeStringShort({...}),2)
end

TextArea=LUAclass()
function TextArea:__init(overlayname, containername, textareaname,x,y,sx,sy ,fontsize)
   if RE.motionPanelValid()==false then return end
   self.overlay=Ogre.createOverlay(overlayname)
   self.overlay:setZOrder(500)
   self.container=Ogre.createContainer(x,y,sx,sy, containername)
   --self.container:setMaterialName("redCircle")
   self.container:setParameter("border_size", "0 0 0 0")
   
   self.element=Ogre.createTextArea(textareaname, 720-6,480-6, 2, 2, fontsize, textareaname,true);
   self.element:setParameter("colour_top", "0 0 0")
   self.element:setParameter("colour_bottom", "0 0 0")	
   --	self.element:setParameter("font_name", "BlueHighway_mod");	
   --	self.element:setParameter("font_name", "IronMaiden")
   self.element:setParameter("font_name", "StarWars")
   --	self.element:setParameter("font_name", "Ogre")
   self.container:addChild(self.element)
   
   self.overlay:add2D(self.container)
   self.overlay:show()

   self.overlayname=overlayname
   self.containername=containername
   self.textareaname=textareaname
end

function TextArea:setCaption(caption)
   if self.overlay ~= nil then
      self.element:setCaption(caption)
   end
end

function TextArea:kill()
   if self.overlay~=nil then
      Ogre.destroyOverlayElement(self.textareaname)-- textArea
      Ogre.destroyOverlayElement(self.containername) -- container
      --      Ogre.destroyAllOverlayElements()
      Ogre.destroyOverlay(self.overlayname)
      self.overlay=nil
   end
end
function TextArea:__finalize()
   self:kill()
end

titlebar={}

function titlebar:create()
   self:destroy()
   
   self.textArea=TextArea("Trace3", "titlebarContainer", "id3",4,4,720,60,30)
end

function titlebar:setCaption(caption)
   if self.textArea~=nil then
      self.textArea:setCaption(caption)
   end
end

function titlebar:destroy()
   if self.textArea~=nil then
      self.textArea:kill()
   end
   self.textArea=nil
end


subtitlebar={}

function subtitlebar:create()
   self:destroy()
   
   self.textArea=TextArea("Trace2", "subtitlebarContainer", "id2", 4,480-60,720, 480-4,10)
end

function subtitlebar:setCaption(caption)
   if self.textArea ~= nil then
      self.textArea:setCaption(caption)
   end
end


function subtitlebar:destroy()
   if self.textArea~=nil then
      self.textArea:kill()
   end
   self.textArea=nil
end


-- call debug.debug() after being called 'dbgtime' times.

function dbg.count(dbgtime)
   print("dbgc:"..dbg._count)

   if dbg._count==dbgtime then
      dbg.console()
   end
   
   dbg._count=dbg._count+1
end



function dbg.get(name, level)
   if level==nil then
      level=4
   end
   local cur=1
   while true do
      k,v=debug.getlocal(level, cur)
      if k==nil then
	 level=level+1
	 if debug.getinfo(level,"n").name==nil then
	    return nil
	 end

	 cur=1
      else
	 if k==name then
	    return v
	 else
	    cur=cur+1
	 end
      end
   end
end

function dbg.out(name)
   aa=dbg.get(name,5)
   if type(aa)=="table" then
      printTable(aa)
   else
      print(aa)
   end
   return aa
end



function dbg.callstackString(level)
   if level==nil then
      level=4
   end
   szout=""
   while true do
      local info=debug.getinfo(level)
      local k=info.name
      if k==nil then
	 break
      else
	 szout=szout..(info.short_src..":"..info.currentline..":"..k)
	 level=level+1
      end
   end
   return szout
end


function dbg.startTrace2() -- full trace to trace.txt
   dbg.filePtr, msg=io.open("trace.txt", "w")
   if dbg.filePtr==nil then
      print(msg)
      return
   end
   debug.sethook(dbg._traceCHook, "l")	
end
function dbg.startTrace3() -- minimal trace to trace.txt (much faster)
   dbg.filePtr, msg=io.open("trace.txt", "w")
   if dbg.filePtr==nil then
      print(msg)
      return
   end
   debug.sethook(dbg._traceCHook3, "c")	
end

function dbg.startDebugConsole() -- almost unusable
   debug.sethook(dbg.debug, "l")	
end

function dbg._traceHook (event, line)
   local s = debug.getinfo(2,'S').short_src
   print(s .. ":" .. line)
end


function dbg._traceCHook3 (event, line)
	local info=debug.getinfo(2,'Sl')
	if info.currentline==-1 then
		-- local info=debug.getinfo(3) 
		-- if info then
		-- 	local l=info.currentline
		-- 	if l~=-1 and l~=dbg._traceCHookPrevLine then
		-- 		dbg.filePtr:write(' ', l)
		-- 		dbg._traceCHookPrevLine=l
		-- 	end
		-- end
	else
		local s=info.source
		if dbg._traceCHookPrevLine==s then
			dbg.filePtr:write(':', info.currentline)
		else
			dbg.filePtr:write('\n',s,':', info.currentline,'\n')
			dbg._traceCHookPrevLine=s
		end
	end
   dbg.filePtr:flush()
end

function dbg.outputToFile(fn, str, postfix)
   if dbg.outputToFilePtr==nil then
      dbg.outputToFilePtr={}
   end
   if dbg.outputToFilePtr[fn]==nil then
      dbg.outputToFilePtr[fn], msg=io.open(fn,"a")
      if dbg.outputToFilePtr[fn] == nil then
	 print(msg)
	 return
      end
   end

   postfix=postfix or "\n"
   local filePtr=dbg.outputToFilePtr[fn]
   filePtr:write(str..postfix)
   filePtr:flush()
end
   


function dbg.help(obj)
   util.printInfo(obj)
   return dbg.console()
end


function dbg.startDebug() 
   -- This functino is to catch where the error occurred. 
   -- dbg.finalize() should be manually called after an error occurred. (e.g. in dtor() function) 
   dbg.g_currLine=1
   dbg.g_lines={}
   dbg.g_max_lines=300
   debug.sethook(dbg.remember, "l")
end

function dbg.remember(event, line)
   local dbg=dbg   
   local s=debug.getinfo(2).short_src   
   local cl=dbg.g_currLine
   dbg.g_lines[cl]=s..":"..line
   cl=cl+1
   --edit cl==x according to dbg.g_max_lines 
   if cl==300 then cl=1 end
   dbg.g_currLine=cl
end

function dbg.finalize()
   debug.sethook()
   if dbg.g_lines~=nil then
      print("dbg finalize")

      cl=dbg.g_currLine

      local gso=dbg.g_max_lines-1

      local stack=array:new()
      for i=1,dbg.g_max_lines do
	 cl=cl-1
	 if cl==0 then cl=gso end
	 stack:pushBack(gso-i.."\t"..dbg.g_lines[cl])
      end

      for i=stack:size(),1,-1 do
	 print(stack[i])
      end
   end
end


function dbg.drawLine2(objectlist, startpos, endpos, nameid, color )
   local lines=vector3N() 
   lines:setSize(2)
   lines:at(0):assign(startpos)
   lines:at(1):assign(endpos)
   
   if nameid==nil then
      nameid=RE.generateUniqueName()
   end
   objectlist:registerObject(nameid, "BillboardLineList",'solidlightblue', lines:matView(),0.5)
end

function dbg.drawLine(objectlist, startpos, endpos, nameid, color )
   local lines=vector3N() 
   lines:setSize(2)
   lines:at(0):assign(startpos)
   lines:at(1):assign(endpos)
   assert(startpos.x==startpos.x)
   
   if nameid==nil then
      nameid=RE.generateUniqueName()
   end
   objectlist:registerObject(nameid, "LineList", color or dbg.linecolor, lines:matView(),0)
end

function dbg.drawTraj(objectlist, matrix, nameid)
	if nameid==nil then
		nameid=RE.generateUniqueName()
	end
	objectlist:registerObject(nameid, "LineList", dbg.linecolor, matrix, 0)
end

function dbg.drawArrow(objectlist, startpos, endpos, nameid,_thick)
   if RE.motionPanelValid() then
	   local node=objectlist:registerEntity(nameid, "arrow2.mesh")
	   local thick=_thick or 10
	   node:resetToInitialState()
	   local dist=(startpos-endpos):length()
	   node:scale(thick/10, dist/50, thick/10)
	   local q=quater()
	   q:axisToAxis(vector3(0,1,0), (endpos-startpos))
	   node:rotate(q)
	   node:translate(endpos)
   end
end

function dbg.drawAxes(objectlist, tf, nameid, scale)
   if RE.motionPanelValid() then
      local tfg=objectlist:registerEntity(nameid, "axes.mesh")
      tfg:setScale(2,2,2)
	  if scale then
		  tfg:setPosition(tf.translation*scale)
	  else
		  tfg:setPosition(tf.translation)
	  end
      tfg:setOrientation(tf.rotation)
   end
end

function dbg.drawCoordinate(objectlist, tf, nameid, offset)
   
	local tf2=transf()
	if offset then
		tf2.translation:assign(tf.translation*100+offset*100)
	else
		tf2.translation:assign(tf.translation*100)
	end
	tf2.rotation:assign(tf.rotation)
	dbg.drawAxes(objectlist, tf2, nameid)
	
	if not dbg._coordinates then
		dbg._coordinates={}
	end
	dbg._coordinates[nameid]=tf2
end

function dbg.eraseAllDrawn()
	if dbg.objectList then
		dbg.objectList:clear()
	end
end


-- dbg.draw('Sphere' or 'Line' or 'Arrow' or 'Axes' or 'Quads'...)
-- to see usage, search drawLine, drawArrow, drawAxes, and so on
function dbg.draw(type, ...)
	if dbg.objectList==nil then
		dbg.objectList=Ogre.ObjectList()
	end
	if type=='registerObject' then
		dbg.objectList:registerObject(...)
	else
		dbg['draw'..type](dbg.objectList, ...)
	end
end
function dbg.erase(type, name)
	if dbg.objectList==nil then
		dbg.objectList=Ogre.ObjectList()
	end
	dbg.objectList:erase(name)
end

function dbg._namedDraw(type, ...)
	dbg.draw(type, ...)
	local pos
	local nameid
	local color='blue'
	if type=="Sphere" then
		local p={...}
		pos=p[1]
		nameid=p[2]
		if p[3] then color=p[3] end
	elseif type=='Axes' then
		local p={...}
		pos=p[1].translation
		nameid=p[2]
		scale=p[3]
		if scale then
			pos=pos*scale
		end
	elseif type=='Coordinate' then
		local p={...}
		pos=p[1].translation*100 + (p[3] or vector3(0,0,0))*100
		nameid=p[2]
	elseif type=="Line" then
		local p={...}
		pos=p[1]
		nameid=p[3]
		if p[4] then color=p[4] end
	elseif type=="Arrow" then
		local p={...}
		pos=p[1]
		nameid=p[3]
	elseif type=="registerObject" then
		local p={...}
		pos=p[4][0]
		nameid=p[1]
	end
	if pos then
	--if false then
		local mat
		if string.find(color,'ed',nil,true)~=nil then --red
			mat=CT.mat(1,4, 0.7,0,0,1)
		elseif string.find(color,'reen',nil,true)~=nil then --green
			mat=CT.mat(1,4, 0,0.5,0,1)
		else
			mat=CT.mat(1,4, 0,0,1,1) -- blue
		end
		local obj=dbg.objectList:registerObject(nameid.."_mt", "MovableText", nameid, mat, 8)
		if obj then obj:setPosition(pos.x, pos.y+20, pos.z) end
	end
	return pos,nameid
end

function dbg.namedDraw(type, ...)
	local pos,nameid=dbg._namedDraw(type, ...)
	if pos then
		RE.output2('namedDraw_'..nameid, table.tostring2({type,...})) -- save for later playback
	end
end

function dbg.drawQuads(objectList, name, mat, _materialName) 
	_materialName=_materialName or "redCircle"
	dbg.objectList:registerObject(name, "QuadList", _materialName, mat,0)
end


function dbg.drawEntity(objectList, entity, pos, nameid, _scale)

   if RE.motionPanelValid() then
	   if _scale==nil then _scale=1 end
	   local comEntity= objectList:registerEntity(nameid, entity)
	   if comEntity then
		   comEntity:setScale(_scale, _scale, _scale)
		   comEntity:setPosition(pos.x, pos.y, pos.z)
	   end
   end
end

function dbg.drawSphere(objectList, pos, nameid, _materialName, _scale)



   if _scale==nil then
      _scale=5 -- 5 cm
   end

   if _materialName~=nil then
      comEntity=objectList:registerEntity(nameid, "sphere1010.mesh", _materialName)
   else
      comEntity=objectList:registerEntity(nameid, "sphere1010.mesh")
   end

   if comEntity then
	   comEntity:setScale(_scale, _scale, _scale)
	   comEntity:setPosition(pos.x, pos.y, pos.z)
   end
end

Ogre.ObjectList.drawSphere=dbg.drawSphere
if CImage then
function CImage:setPixel(x,y, R,G,B, width)
	width=width or 1
	self:drawBox(TRect(x-width+1,y-width+1,x+width,y+width),R, G,B)
end
end
function quater:rotationY()
   local rot_y=quater()
   local offset=quater()
   self:decompose(rot_y, offset)
   return rot_y
end

function quater:distance(q2)
   local q=self*q2:inverse()
   q:align(quater(1,0,0,0))
   return q:rotationAngle()
end

-- returns q such tat q*self=q2
function quater:rotationVecTo(q2)
	local q=q2*self:inverse()
	q:align(quater(1,0,0,0))
	return q:rotationVector()
end

function quater:dot(q)
   return self.x*q.x+self.y*q.y+self.z*q.z+self.w*q.w
end				

function quater:offsetQ()
   local rot_y=quater()
   local offset=quater()
   self:decompose(rot_y, offset)
   return offset
end

function matrix3:Transpose()
   local out=matrix3(self)
   out:transpose()
   return out
end

function matrix4:getTranslation()
   local out=vector3()
   out.x=self._14
   out.y=self._24
   out.z=self._34
   return out
end

function TStrings:__tostring()
	local str=''
	for i=0, self:size()-1 do
		str=str..tostring(i).. (self(i) or "").."\n"
	end
	return str
end

if QuadraticFunctionHardCon then

-- addSquared(3,0,4,1,5,2,-1) : add(3x+4y+5z-1)^2
function QuadraticFunctionHardCon:add(...)
	local param={...}

	local n21=table.getn(param)

	local n=math.floor(n21/2)

	local index=intvectorn(n)
	local coef=vectorn(n+1)
	for i=0, n-1 do 
		index:set(i, param[i*2+2])
		coef:set(i, param[i*2+1])
	end
	coef:set(n, param[n*2+1])
	self:addSquared(index, coef)
end
--- w*(Ax-b)^2
function QuadraticFunctionHardCon:addSystem(w,A,b)
	local n=A:cols()
	local nc=A:rows()-- num constraints
	local index=intvectorn(n)
	index:colon(0, n,1)

	assert(b:size()==nc)
	local coef=vectorn(n+1)
	for i=0, nc-1 do
		coef:range(0,n):assign(A:row(i))
		coef:set(n, b(i)*-1)
		coef:rmult(math.sqrt(w))
		self:addSquared(index, coef)
	end
end

-- addCon(3,0,4,1,5,2,-1) : add(3x+4y+5z-1=0)
function QuadraticFunctionHardCon:con(...)
	local param={...}

	local n21=table.getn(param)

	local n=math.floor(n21/2)

	local index=intvectorn(n)
	local coef=vectorn(n+1)
	for i=0, n-1 do 
		index:set(i, param[i*2+2])
		coef:set(i, param[i*2+1])
	end
	coef:set(n, param[n*2+1])
	self:addCon(index, coef)
end

-- Ax=b
function QuadraticFunctionHardCon:conSystem(A,b)
	local n=A:cols()
	local nc=A:rows()-- num constraints
	local index=intvectorn(n)
	index:colon(0, n,1)

	assert(b:size()==nc)
	local coef=vectorn(n+1)
	for i=0, nc-1 do
		coef:range(0,n):assign(A:row(i))
		coef:set(n, b(i)*-1)
		self:addCon(index, coef)
	end
end

function QuadraticFunctionHardCon:solve()
	local A=matrixn()
	local b=vectorn()
	self:buildSystem(A,b)
	local x_lambda=A:LeftDiv(b:column()):column(0)
	return x_lambda:range(0, self.numVar)
end
end

-- when you want to use 0 index
vector=LUAclass()
function vector:__tostring()
   local out="vector:"
   for i=0, self:size()-1 do
      out= out .. tostring(i)..": " .. self(i)
   end
   return out
end
function vector:foreach(fn, param)
   for i=0, self:size()-1 do
      if param==nil then
	 fn(self(i))
      else
	 fn(self(i), param)      
      end
   end
end

function vector:__init(n)
   self.tf={}
   
   if n~=nil then
      self.n=n
   else
      self.n=0
   end
end

function vector:__call(key)
   return self.tf[key+1]
end

function vector:set(key, value)
   assert(key<self.n)
   self.tf[key+1]=value
end

function vector:size()
   return self.n
end

function vector:resize(n)
   local prev_n=self.n
   self.n=n
   
   for i=prev_n, n do
      self.tf[i]=nil
   end
end

function vector:push_back(x)
   self.tf[self.n+1]=x
   self.n=self.n+1
end

function vector:pushBack(x)
   self.tf[self.n+1]=x
   self.n=self.n+1
end

function vector:back()
   return self.tf[self.n]
end

function vector:__finalize()
   if self.__dtor~=nil then
      self:__dtor()
   end
end


function array:foreach(fn, param)
	if param==nil then
		for i=1, #self do
			local o=fn(self[i])
			if o then self[i]=o end
		end
	else
		for i=1, #self do
			local o=fn(self[i], param)      
			if o then self[i]=o end
		end
	end
end



-- assuming array of tables
function array:findTable(key, val)
	if val==nil then
		for i=1, table.getn(self) do
			if self[i][key] then
				return self[i]
			end
		end
	else
		for i=1, table.getn(self) do
			if self[i][key]==val then
				return self[i]
			end
		end
	end
	return {}
end



RE.connectedSkins=vector()
function RE.connectedSkins:__dtor()
   print("dtor called")
   for i=0,self:size()-1 do
      RE.motionPanel():motionWin():detachSkin(self(i).skin)
      self:set(i,nil)
   end
end

function RE.createConnectedVRMLskin(skel, mot)
   local skin_info={}
   skin_info.skel=skel
   skin_info.mot=mot
   skin_info.skin=RE.createVRMLskin(skel, false)
   skin_info.skin:scale(100,100,100)
   skin_info.skin:applyMotionDOF(mot)
   skin_info.skin:startAnim()
   
   RE.motionPanel():motionWin():addSkin(skin_info.skin)
   
   
   RE.connectedSkins:push_back(skin_info)
   return skin_info.skin
end

function RE.connectSkin(skin)
   local skin_info={}
   skin_info.skin=skin
   RE.motionPanel():motionWin():addSkin(skin)
   RE.connectedSkins:push_back(skin_info)
end

function RE.removeAllConnectedSkins()
   RE.motionPanel():motionWin():detachAllSkin()
   RE.connectedSkins:resize(0)
end

CT={}


function CT.QP(numVar, numCon)
	return QuadraticFunctionHardCon(numVar, numCon)
end

function CT.block(nr, nc, ...)

   local tbl={...}

   local nrow=0
   local ncol=0

   for j=1,nc do
      ncol=ncol+tbl[j]:cols()
   end

   for i=1,nr do
      nrow=nrow+tbl[(i-1)*nc+1]:rows()
   end

   local out=matrixn(nrow, ncol)

   crow=0
   local rsize
   for i=0,nr-1 do
      ccol=0
      rsize=tbl[i*nc+1]:rows()
      for j=0,nc-1 do
	 local mat=tbl[i*nc+j+1]
	 assert(mat:rows()==rsize)
	 out:range(crow, crow+rsize, ccol, ccol+mat:cols()):assign(mat)
	 ccol=ccol+mat:cols()
      end
      assert(ccol==ncol)
      crow=crow+rsize
   end
   assert(crow==nrow)

   return out
end


function CT.transfIdentity()
   local mat=transf()
   mat.translation:zero()
   mat.rotation:identity()
   return mat
end

function CT.pair(v)
   return {v, v:copy()}
end

USE_LUNA_GEN=true
if USE_LUNA_GEN then
	if str==nil then
		str={}
		function str.left(str,n)
			if n<0 then
				return string.sub(str, 1, #str+n)
			end
			return string.sub(str,1,n) 
		end
		function str.right(str,n)
			return string.sub(str,-1*n) 
		end
		function str.length(str)
			return #str
		end
		function str_include(str, str2)
			return select(1,string.find(str, str2,1, true))~=nil
		end
		function str.filename(fullpath)
			return os.filename(fullpath)
		end
	end
	util.isFileExist=os.isFileExist

	if util.PerfTimer2==nil then
		util.PerfTimer2=LUAclass()
		function util.PerfTimer2.__init() end
		function util.PerfTimer2.reset() end
		function util.PerfTimer2.start() end
		function util.PerfTimer2.stop() end
		function util.PerfTimer2.pause() end
	end
	util.PerfTimer=util.PerfTimer2

	function MotionLoader.new(skelfilename, mot_files)
		local l=MotionLoader._create(skelfilename)
		for i,v in ipairs(mot_files) do
			l:_append(v)
		end
		return l
	end
	if Physics then
		Physics.setParameter=function(...) end
	end
	TStrings.set_old=TStrings.set
	function TStrings:set(i, c)
		if c then
			self:set_old(i,c)
		else
			self:set(i," ")
		end
	end
	function CT.vec(...)
		local vec=vectorn()
		-- if type(select(1,...))=='number' then
			vec:setValues(...)
			return vec
			-- else
		-- 	vec:setValues( unpack(select(1,...)))
		-- 	return vec
		-- end
	end
	function CT.ivec(...)
		assert(type(select(1,...))=='number')
		local vec=intvectorn()
		vec:setValues(...)
		return vec
	end
function CT.mat(n,m, ...)
		assert(type(select(1,...))=='number')
   local mat=matrixn(n,m)
   mat:setValues(...)
   return mat
end

function CT.matrixn(row1, ...)
	assert(false)
   local mat=matrixn()
   mat:assign({row1,...})
   return mat
end
function math.__checkVec(v)
   if type(v)=="table" then
	   local vec=vectorn()
	   vec:setValues(unpack(v))
      return vec
   end
   return v
end
else
function CT.mat(n,m, ...)
   local mat=matrixn(n,m)
   mat:setValues(...)
   return mat
end

function CT.matrixn(row1, ...)
   local mat=matrixn()
   mat:assign({row1,...})
   return mat
end
	function CT.vec(...)
		tbl={...}
		if type(tbl[1])=="table" then tbl=tbl[1] end
		local vec=vectorn()
		vec:assign(tbl)
		return vec
	end
	function CT.ivec(...)
		tbl={...}
		if type(tbl[1])=="table" then tbl=tbl[1] end
		local vec=intvectorn()
		vec:assign(tbl)
		return vec
	end
function math.__checkVec(v)
   if type(v)=="table" then
      return CT.vec(v)
   end
   return v
end
end

function CT.tblVec(vec)
   local tbl={}
   for i=0, vec:size()-1 do
      tbl[i+1]=vec(i)
   end
   return tbl
end

function CT.vecFromVec3(v)
   local vec=vectorn(3)
   vec:setVec3(0,v)
   return vec
end

function CT.eye(n)
   local mat=matrixn(n,n)
   mat:setAllValue(0)
   mat:diag():setAllValue(1)
   return mat
end
function CT.rand(m,n)
	if n==nil then
		-- vectorn
		assert(false)-- not implemented yet
	else
		local mat=matrixn(m,n)
		for i=0,m-1 do
			for j=0,n-1 do
				mat:set(i,j, math.random())
			end
		end
		return mat
	end
end

function CT.inverse(a)
	local mat=matrixn()
	mat:inverse(a)
	return mat
end

function CT.skew(w)
	return CT.mat(3,3, 
	0, -w.z, w.y,
	w.z, 0, -w.x,
	-w.y, w.x, 0)
end

function CT.diag(vec)
   local mat=matrixn(vec:size(), vec:size())
   mat:setAllValue(0)
   mat:diag():assign(vec)
   return mat
end

function CT.colon(startI,endI,step)
	local i=intvectorn()
	i:colon(startI, endI, step)
	return i
end

function CT.xrange(endI)
	return CT.colon(0, endI, 1)
end

function CT.linspace(a,b,size)
   local vec=vectorn()
   vec:linspace(a,b,size)
   return vec
end

function CT.zeros(n,m)
	if m~=nil then
		local mat=matrixn(n,m)
		mat:setAllValue(0)
		return mat
	else
		local vec=vectorn(n)
		vec:setAllValue(0)
		return vec
	end
end
function CT.ones(n,m)
	if m~=nil then
		local mat=matrixn(n,m)
		mat:setAllValue(1)
		return mat
	else
		local vec=vectorn(n)
		vec:setAllValue(1)
		return vec
	end
end

function CT.shearY(normal)
   local temp=normal/normal.y
   local mat=matrix4()
   mat:identity()
   mat._12=temp.x
   mat._32=temp.z
   return mat
end

function matrix4:toLocalDir(gdir)
   local temp=matrix4()
   temp:assign(self)
   temp._14=0
   temp._24=0
   temp._34=0
   return temp:inverse()*gdir
end

function rotate(v, q)
   local v2=v:copy()
   v2:rotate(q)
   return v2
end

function matrix4:toGlobalDir(ldir)
   local temp=matrix4()
   temp:assign(self)
   temp._14=0
   temp._24=0
   temp._34=0
   return temp*ldir
end

function math.pseudoInverse(a)
	local mat=matrixn()
	mat:pseudoInverse(a)
	return mat
end

function math.srInverse(a, alpha)
	local ra=gmbs.RMatrix()
	ra:assign(a)
	return gmbs.srInv(ra,alpha):toMat()
end

function math.nullspaceProjector(jacobian)
	local invJJ=math.pseudoInverse(jacobian)*jacobian
	return CT.eye(invJJ:rows())-invJJ
end

function math.nullspace(m, eps)
	eps=eps or 0.001
	local ra=gmbs.RMatrix()
	ra:assign(m)
	local rb=ra:Nullspace(eps)
	return rb:toMat()
end

function math.smoothClamp(v, v_max)
	local half_pi=math.rad(90)
	
	if v<half_pi*v_max then 
		return math.sin(v/v_max)*v_max
	else
		return v_max
	end
end
function math.smoothClampVec3(vec, maxLen)
	if false then
		local out=vector3()
		local bclamp=false
		out:assign(vec)
		local len=vec:length()
		if len>maxLen then
			out:scale(maxLen/len)
			bclamp=true
		else
			out:scale(sop.mapCos(len, 0, maxLen, 1, maxLen/len))
		end

		return out, bclamp
	else
		local out=vector3()
		local bclamp=false
		out:assign(vec)
		local len=vec:length()
		if len>0.001 then
			local newLen=math.smoothClamp(len, maxLen)
			out:scale(newLen/len)
		end
		if len>maxLen then
			bclamp=true
		end
		return out,bclamp
	end
end

function math.sign(a)
	if a>=0 then
		return 1
	else
		return -1
	end
end
function math.clampVec3(vec, maxLen)
   local out=vector3()
   local bclamp=false
   out:assign(vec)
   local len=vec:length()
   if len>maxLen then
      out:scale(maxLen/len)
      bclamp=true
   end
   return out, bclamp
end
function math.sharpTransition(vec, a,b,len)
   vec:setSize(len)
   local half=math.floor(len/2+0.5)
   for i=0,half-1 do
      vec:set(i, a)
   end

   for i=half, len-1 do
      vec:set(i, b)
   end
end

function math.round(i)
   return math.floor(i+0.5)
end

function math.clamp(i,a,b)
   return math.min(math.max(i,a),b)
end

function BoneForwardKinematics:assign(other)
   local pose=Posture()
   other:getPoseFromGlobal(pose)
   self:setPose(pose)
end

function Bone:isDescendent(parent)
   
   local child=self
   while child~=nil do
      if child==parent then
	 return true
      end
      child=child:parent()
   end
   return false
end




python={}
function python.ipairsToCode(name, tbl)
	local contents={}
   array.pushBack(contents,'')
   array.pushBack(contents,name..'=(')
   for i=1,#tbl do
	   if type(tbl[i])=='string' then
		   array.pushBack(contents,"'"..tbl[i].."',")
	   else
		   assert(false)
	   end
   end
   array.pushBack(contents,')')
   return table.concat(contents,'\n')
end
Octave_wrap=LUAclass()
function Octave_wrap:__init()
	require('pyMrdplot')
	self.pythonHelper=pythonHelper()
	self.pythonHelper:add([[import numpy]])
	self.pythonHelper:add([[def get(a): return eval(a)]])
end
function Octave_wrap:call(name, n, tbl)
	if name=='set' then
		self.pythonHelper:add(tbl[1]..'=numpy.matrix('..self.pythonHelper:tostr(tbl[2])..')')
	end
end
function Octave_wrap:eval(str)
	if str=='close(1)' then
		self.pythonHelper:close()
	else
		self.pythonHelper:add(str)
	end
end

function python.__init()
	if python.server==nil then
		if Octave then
			python.server=Octave()
		else
			python.server=Octave_wrap()
		end
	  if python.server.errorOccurred then
		  os.execute("lua launch.lua ps") -- launch python server
		  for c=1,10 do
			  python.server=nil
			  collectgarbage() 
			  os.sleep(2)
			  print("trying to connect to python server...")
			  python.server=Octave()
			  if python.server.errorOccurred==false then break
			  end
		  end
		  assert(not python.server.errorOccurred)
	  end
  end 
end
function py_exec(name, ...)
   local param={...}
  python.__init()

   return python.server:call(name, table.getn(param), param)
end

function py_eval(str)
  python.__init()
   --print(str)
   python.server:eval(str)
end

gnu_octave={initialized={}}

function gnu_octave.registerOctaveFunction(funcName, funcDef)
   if not gnu_octave.server then 
      gnu_octave.server=Octave() 
   end
   if gnu_octave.initialized[funcName]~=true then
      gnu_octave.server:eval(funcDef)
      gnu_octave.initialized[funcName]=true
   end
end
function gnu_octave.registerMRDplot()
   if not gnu_octave.server then 
      gnu_octave.server=Octave() 
   end
   local currDir=util.getCurrentDirectory()
   local parentDir=os.rightTokenize(string.gsub(currDir, "\\","/"), "/")
   local matlabpath=parentDir.."\\resource\\scripts\\ui\\rigidbodywin\\mrdplot\\matlab"
   gnu_octave.registerOctaveFunction("addMRDpath", "addpath('"..matlabpath.."')")   

end

function gnu_octave.__octaveInit()
   gnu_octave.registerOctaveFunction("plotVec1", gnu_octave.plotVec1)
   gnu_octave.registerOctaveFunction("plotVec2", gnu_octave.plotVec2)
end

function math.plotVec(vec, cont)
	if false then
   gnu_octave.__octaveInit()
   local T=vectorn()
   T:linspace(0, vec:size()-1, vec:size())
   local v=vectorn()
   v:assign(vec)
   if cont==true then
      gnu_octave.server:call("plotVec2", 0, {T:row(),v:row()})
   else
      gnu_octave.server:call("plotVec1", 0, {T:row(),v:row()})
  end
else
		require('pyMrdplot')
		local fig1=pythonHelper()
		fig1:figure()
		fig1:add('grid(True)')
		
		local tt=CT.linspace(0, vec:size()-1, vec:size())
		fig1:plot(tt,vec)
		fig1:savefig('fig1.png')
		fig1:close()
	end
end
function math.plotStitch(c, a, b)
	require("subRoutines/MatplotLib")
	local stitched=c:column(0)
	local xfn=CT.xrange(a:rows())
	local yfn=a:column(0)
	local yfn2=b:column(0)
	local plotter=MatplotLib()
	plotter:figure{1, nsubfig={1,1}, subfig_size={3,3}} -- two by one plotting area.
	plotter:add('grid(True)')
	plotter:subplot(0,0)
	plotter:plot(xfn, yfn)
	--plotter:plot(yfn2, xfn)
	plotter:plot(CT.colon(xfn:size()-1, stitched:size(), 1), yfn2)
	plotter:plot(CT.xrange(yfn:size()+yfn2:size()-1), stitched)
	plotter:xlabel('x')
	plotter:ylabel('t')
	plotter:legends('a', 'b', 'c')
	plotter:savefig('plot.png')
	plotter:close()
end

function math.plotMat(mat)
   gnu_octave.__octaveInit()
   local T=vectorn()
   T:linspace(0, mat:rows()-1, mat:rows())
   local v=mat:copy()
   v:transpose()
   gnu_octave.server:call("plotVec", 0, {T:row(),v})
end

function math.mrdplotFile(filename)
	if gnu_octave.server then  -- use octave server
		gnu_octave.registerMRDplot()

		gnu_octave.server:call("mrd_plot",0,{"load", util.getCurrentDirectory().."\\"..filename}) -- have to be absolute path
		gnu_octave.server:call("mrd_plot", 0, {"plotAll"})      
	else -- use python server by default
		require('pyMrdplot')
		pyMrdplot(filename, 'plotAll')
   		os.open(pyMrdPlot.filename..".png") -- launch image viewer. (debug_plot2.png by default )
	end
end
function math.mrdplotVec(vec1, name1,...)
   input={vec1, name1, ...}
   local mrdplot=MRDplot()
   local nch=table.getn(input)/2
   local ndata=10000
   for i=1,nch do
      ndata=math.min(ndata, input[i*2-1]:size())
   end

   mrdplot:initMRD(nch, ndata,1)
   for i=1,nch do
      mrdplot.names:set(i-1, input[i*2])
      mrdplot.data:column(i-1):assign(input[i*2-1]:range(0,ndata))
   end
   mrdplot:save("temp.mrd")
  
   math.mrdplotFile("temp.mrd")
   
   math.changeChartPrecision(50)
   for i=1, nch do
      math.drawSignals("temp.bmp", mrdplot.data:column(i-1):column())
      RE.motionPanel():scrollPanel():addPanel("temp.bmp")
   end
end

function math.mrdplotMat(mat)

   input={}

   for i=0,mat:cols()-1 do
      input[i*2+1]=mat:column(i)
      input[i*2+2]="column"..i
   end
   math.mrdplotVec(unpack(input))

end

function math.copy(valseq)
   if type(valseq)=="number" or type(valseq)=="string" then
      return valseq
   end
   return valseq:copy()
end

function vector3:Normalize()
   local o=vector3()
   o:assign(self)
   o:normalize()
   return o
end
function vector3:toTable()
	return {"__userdata", "vector3", self.x, self.y, self.z}
end
function vector3.fromTable(t)
	assert(t[3])
	return vector3(t[3], t[4], t[5])
end
function quater:toTable()
	return {"__userdata", "quater", self.w, self.x, self.y, self.z}
end
function quater.fromTable(t)
	return quater(t[3], t[4], t[5], t[6])
end

function vector3:rdiv(v)
	self:assign(self/v)
end
function boolN:toTable()
	local tbl={}
	for i=0, self:size()-1 do
		if self(i) then
			array.pushBack(tbl,i)
		end
	end
	tbl.size=self:size()
	return {"__userdata", "bitvectorn", tbl}
end
function boolN.fromTable(t)
	local bitv=boolN()
	bitv:setSize(t[3].size)
	bitv:clearAll()
	for i, v in ipairs(t[3]) do
		bitv:set(v,true)
	end
	return bitv
end
function Mesh:copy()
   local a=Mesh()
   a:assign(self)
   return a
end

function matrixn:assign33(M)
	self:set(0,0,M._11)
	self:set(0,1,M._12)
	self:set(0,2,M._13)
	self:set(1,0,M._21)
	self:set(1,1,M._22)
	self:set(1,2,M._23)
	self:set(2,0,M._31)
	self:set(2,1,M._32)
	self:set(2,2,M._33)
end
function matrixn:assign44(M)
	self:set(0,0,M._11)
	self:set(0,1,M._12)
	self:set(0,2,M._13)
	self:set(0,3,M._14)
	self:set(1,0,M._21)
	self:set(1,1,M._22)
	self:set(1,2,M._23)
	self:set(1,3,M._24)
	self:set(2,0,M._31)
	self:set(2,1,M._32)
	self:set(2,2,M._33)
	self:set(2,3,M._34)
	self:set(3,0,M._41)
	self:set(3,1,M._42)
	self:set(3,2,M._43)
	self:set(3,3,M._44)
end

function matrixn:identity()
	assert(self:rows()==self:cols())
	self:setAllValue(0)
	self:diag():setAllValue(1)
end
function matrixn:isSimilar(o)
	local mdiff=matrixn()
	mdiff:assign(self-o)
	return mdiff:minimum()>-0.00001 and mdiff:maximum()<0.00001
end

function matrixn:__tostring()

	local out="{"
	local a=self
	local function printRow(i)
		out=out.."\n {["..i.."]={"
		if a:cols()<10 then
			for j=0, a:cols()-1 do
				out=out..string.format("%.3f",a(i,j))..", "
			end
		else
			for j=0, 4 do
				out=out..string.format("%.3f",a(i,j))..", "
			end
			out=out..'..., '
			for j=a:cols()-5, a:cols()-1 do
				out=out..string.format("%.3f",a(i,j))..", "
			end
		end
		out=out.."}"
	end
	if a:rows()<10 then
		for i=0,a:rows()-1 do
			printRow(i)
		end
	else
		for i=0,4 do
			printRow(i)
		end
		out=out..'\n ...\n'
		for i=a:rows()-5, a:rows()-1 do
			printRow(i)
		end
	end

	return out.."\n}\n"
end
function matrixn:calcDerivative_sub( frame_rate)
	local src=self
   assert(src~=nil)
   local dsrc=matrixn()
   
   dsrc:setSize(src:rows(), src:cols())
   
   for i=1, src:rows()-2 do
	   dsrc:row(i):sub(src:row(i+1),src:row(i-1)) 
	   dsrc:row(i):rmult(frame_rate/2.0)
   end
   
   -- fill in empty rows
   dsrc:row(0):assign(dsrc:row(1))
   dsrc:row(dsrc:rows()-1):assign(dsrc:row(dsrc:rows()-2))
   return dsrc
end

function matrixn:derivative( frame_rate, discontinuity)
	local src=self
	assert(type(frame_rate)=='number')
	if discontinuity then
		local dsrc=matrixn()
		dsrc:setSize(src:rows(), src:cols())

		local segFinder=SegmentFinder(discontinuity)

		local cols=dsrc:cols()
		for i=0, segFinder:numSegment()-1 do
			local s=segFinder:startFrame(i)
			local e=segFinder:endFrame(i)

			dsrc:range(s,e,0, cols):assign(matrixn.calcDerivative_sub(src:range(s,e, 0, cols),frame_rate))
		end

		return dsrc
	else
		return matrixn.calcDerivative_sub(src, frame_rate)
	end
end
function matrixn:__div(b)
   --a/b=c
   --a=c*b
   return self*b:Inverse()
end
function matrixn:toTable()
	local data={}
	for i=0, self:rows()-1 do
		data[i+1]={}
		local tbl=data[i+1]
		for j=0, self:cols()-1 do
			tbl[j+1]=self(i,j)
		end
	end
	return {"_userdata", "matrixn", data}
end
function matrixn.fromTable(t)

	local data=t[3]
	local a=matrixn(table.getn(data), table.getn(data[1]))
	for i=0, a:rows()-1 do
		local tbl=data[i+1]
		for j=0, a:cols()-1 do
			a:set(i,j, tbl[j+1])
		end
	end
	return a
end
function matrixn:zero()
	self:setAllValue(0)
end
function matrixn:LeftDiv(b)
   return self:Inverse()*b
end
-- convenient
-- a:sub(1,0,4,0)==a:range(1, a:rows(), 4, a:cols()) 
-- a:sub(1,-3,4,0)==a:range(1, a:rows()-3, 4, a:cols()) 
function matrixn:sub(srow,erow,scol,ecol)
	scol=scol or 0
	ecol=ecol or 0
	if erow<=0 then
		erow=self:rows()-erow
	end
	if ecol<=0 then
		ecol=self:cols()-ecol
	end

	return self:range(srow, erow, scol, ecol)
end

function matrixn.concat(a,b) -- concat row
   local c=matrixn()
   if a:cols()~=b:cols() then error("matrixn:concat") end
   c:resize(a:rows()+b:rows(), a:cols())
   c:range(0,a:rows(),0,a:cols()):assign(a)
   c:range(a:rows(), c:rows(),0,b:cols()):assign(b)
   return c
end

function matrixn.__concat(a,b) -- concat col
   local c=matrixn()
   if a:rows()~=b:rows() then error("matrixn:__concat") end
   c:resize(a:rows(), a:cols()+b:cols())
   c:range(0,a:rows(),0,a:cols()):assign(a)
   c:range(0,a:rows(),a:cols(),c:cols()):assign(b)
   return c
end

   
function matrixn:Transpose()
   local Ct=matrixn()
   Ct:assign(self)
   Ct:transpose()
   return Ct
end

function matrixn:Inverse()
   local Ct=matrixn()
   Ct:inverse(self)
   return Ct
end

function matrixn:__eq(b)
   local a=self

   local tol=__globals.EQ_THR
	local abs=math.abs

   if a:rows()~=b:rows() then return false end
   if a:cols()~=b:cols() then return false end
   for i=0,a:rows()-1 do
      for j=0, a:cols()-1 do
	 if abs(a(i,j)-b(i,j))>tol then
	    return false
	 end
      end
   end
   return true
end


function matrixn:pushBackUtil(tbl)
   local v=vectorn()
   v:assign(tbl)
   self:pushBack(v)
end
function matrixn:copy()
   local a=matrixn()
   a:assign(self)
   return a
end

if USE_LUNA_GEN then
else
	function matrixn:setValues(a,...)
		if type(a)=="table" then
			self:setValue(a)
		else
			self:setValue({a,...})
		end
	end
end

function matrixn:__unm()
   local t=matrixn()
   t:assign(self)
   t:rmult(-1)

   return t
end
  
function matrixn:multAdiagB(matA, diagB)

   assert(matA:rows()==diagB:size())

   self:assign(matA)
   for i=0,self:rows()-1 do
      self:column(i):rmult(diagB(i))
   end
end


function matrixn:setSymmetric(a,...)
   local tbl=a
   if type(a)~="table" then
      tbl={a,...}
   end

   local c=1
   for i=0, self:rows()-1 do
      for j=i, self:cols()-1 do
	 self:set(i,j,tbl[c])
	 self:set(j,i,tbl[c])
	 c=c+1
      end
   end
end

function matrixn:range_c(firstRow, lastRow)
	return self:range(firstRow, lastRow+1, 0, self:cols())
end

function matrixn:size()
	return self:rows()
end
function matrixn:sample(refTime)
	local c=vectorn()
	self:sampleRow(refTime, c)
	return c
end
defineDerived(matrixn, {matrixnView}, {"assign33", "isSimilar", "identity", "__tostring", "__div", "derivative", "fromTable", "toTable", "zero", "LeftDiv", "concat", "__concat", "Transpose", "Inverse", "__eq","pushBackUtil","copy", "setValues","__unm","multAdiagB","setSymmetric" ,"range_c", "sub","size","sample"})
function vector3:__eq(b)
   local a=self

	local abs=math.abs
	if abs(a.x-b.x)>__globals.EQ_THR then return false end
   if abs(a.y-b.y)>__globals.EQ_THR then return false end
   if abs(a.z-b.z)>__globals.EQ_THR then return false end
   return true
end

function vector3:squaredDistance(b)
   local len=self:distance(b)
   return len*len
end

function quater:__eq(b)
   local a=self

	local abs=math.abs
	if abs(a.x-b.x)>__globals.EQ_THR then return false end
   if abs(a.y-b.y)>__globals.EQ_THR then return false end
   if abs(a.z-b.z)>__globals.EQ_THR then return false end
   if abs(a.w-b.w)>__globals.EQ_THR then return false end
   return true
end

function quater:Normalize()
	local c=self:copy()
	c:normalize()
	return c
end

function vectorn:sharpTransition(a, b, len)
   math.sharpTransition(self, a,b,len)
end
-- function vectorn:smoothClamp(mag)
-- 	for i=0,self:size()-1 do
-- 		self:set(i, sop.mapSin(self(i), 0, mag, 0, mag))
-- 	end
-- end
function vectorn:smoothClamp(mag1, mag2,w)
	if mag2==nil then
		assert(mag1>0)
		local mag=mag1
		local smc=math.smoothClamp
		for i=0,self:size()-1 do
			self:set(i, smc(self(i), mag))
		end
	else
		assert(mag1==mag2*-1) -- current assumption.. will be loosened later.
		local mag=mag2
		local smc=math.smoothClamp
		if w then
			for i=0,self:size()-1 do
				local s_i=self(i)
				if s_i>0 then
					self:set(i, smc(s_i, mag*w(i)))
				else
					self:set(i, smc(s_i*-1, mag*w(i))*-1)
				end
			end
		else
			for i=0,self:size()-1 do
				local s_i=self(i)
				if s_i>0 then
					self:set(i, smc(s_i, mag))
				else
					self:set(i, smc(s_i*-1, mag)*-1)
				end
			end
		end
 	end
 end
function vectorn:extract(index)
	local out=vectorn(index:size())
	for i=0, index:size()-1 do
		out:set(i, self(index(i)))
	end
	return out
end
function vectorn:zero()
	self:setAllValue(0)
end
if not USE_LUNA_GEN then
	function vectorn:setValues(a,...)
		if type(a)=="table" then
			self:assign(a)
		else
			self:assign({a,...})
		end
	end
end

function vectorn:__concat(b)
	local c=vectorn(self:size()+b:size())
	c:range(0, self:size()):assign(self)
	c:range(self:size(), c:size()):assign(b)
	return c
end

function vectorn:__eq(b)
	local a=self

	local tol=__globals.EQ_THR
	local abs=math.abs
	if a.size and b.size then
		if a:size()~=b:size() then return false end
		for i=0,a:size()-1 do
			if abs(a(i)-b(i))>tol then
				return false
			end
		end

		return true
	else
		return false
	end
end

function vectorn:copy()
   local a=vectorn()
   a:assign(self)
   return a
end


function vectorn:back()
   return self(self:size()-1)
end


function vectorn:concat(b)
   local a_size=self:size()
   self:resize(a_size+b:size())
   self:range(a_size, self:size()):assign(b)
end

function vectorn:subvec(scol, ecol)
	scol=scol or 0
	ecol=ecol or 0
	if ecol<=0 then
		ecol=self:size()-ecol
	end
	return self:range(scol, ecol)
end


defineDerived(vectorn, {vectornView}, {"extract", "sharpTransition", "zero", "setValues", "__concat", "__eq", "back", "concat","copy"})


function quaterN:concat(b)
	local osize=self:size()
	self:resize(osize+b:size())
	for i=osize,self:size()-1 do
		self(i):assign(b(i-osize))
	end
end

function quaterN:normalize()
   for i=0, self:size()-1 do
      self(i):normalize()
   end
end

function quaterN:__tostring()
   return self:matView(0,self:size()):__tostring()
end

function quaterN:setAllValue(q)
   local mat=self:matView(0, self:size())
   mat:column(0):setAllValue(q.w)
   mat:column(1):setAllValue(q.x)
   mat:column(2):setAllValue(q.y)
   mat:column(3):setAllValue(q.z)
end

function quaterN:copy()
	local out=quaterN()
	out:assign(self)
	return out
end

function quaterN:smooth(kernelsize)
	if true then
		-- renormalize
		math.filter(self:matView(0, self:size()), kernelsize)
		for i=0,self:size()-1 do
			self(i):normalize()
		end
	else
		assert(self:size()>kernelsize)
		math.filterQuat(self:matView(0, self:size()), kernelsize)
	end
end
defineDerived(quaterN, {quaterNView}, {"smooth", "copy", "normalize", "__tostring","setAllValue"})

function vector3:copy()
   local a=vector3()
   a:assign(self)
   return a
end
function vector3:Rotate(q)
   local a=vector3()
   a:rotate(q, self)
   return a
end
function quater:copy()
   local a=quater()
   a:assign(self)
   return a
end
function transf:copy()
   local a=transf()
   a:assign(self)
   return a
end

function transf:identity()
	self.rotation:identity()
	self.translation:zero()
end
function transf:toTable()
	return {"__userdata", "transf", self.translation.x, self.translation.y, self.translation.z, self.rotation.w, self.rotation.x, self.rotation.y, self.rotation.z}
end
function transf.fromTable(t)
	local tt=vector3(t[3], t[4], t[5])
	local r=quater()
	r.w=t[6]
	r.x=t[7]
	r.y=t[8]
	r.z=t[9]
	return transf(r,tt)
end
			
function transf:axisToAxis(p1, v1, p2, v2)
   local qi=quater(1,0,0,0)
   local pi=vector3(0,0,0)
   local q=quater()
   q:axisToAxis(v1,v2)

   self:assign(transf(qi, p2)*transf(q, pi)* transf(qi,-p1))
end

function transf:leftMultRotation(q)
	self:assign(transf(q, vector3(0,0,0))*self)
end
function transf:leftMultTranslation(v)
	self:assign(transf(quater(1,0,0,0),v)*self)
end

function transf:__tostring()
	return "R:"..tostring(self.rotation).." T:"..tostring(self.translation)
end

function boolN:setSize(n)
	self:resize(n)
end
function boolN:count(val)
	if val ==nil then val=true end
	local count=0
	for i=0, self:size()-1 do
		if self(i)==val then
			count=count+1
		end
	end

	return count
end
boolNView.count=boolN.count

function vector3N:sample(i)
	return self:sampleRow(i)
end
function vector3N:concat(b)
	local osize=self:size()
	dbg.console()
	self:resize(osize+b:size())
	for i=osize,self:size()-1 do
		self(i):assign(b(i-osize))
	end
end
function vector3N:smooth(kernelsize)
	math.filter(self:matView(0, self:size()), kernelsize)
end

function vector3N:at(i)
	if i>=0 then
		return self:row(i)
	end
	return self:row(self:rows()+i)
end

function vector3N:__eq(b)
	return self:matView()==b:matView()
end
function vector3N:derivative(com, frameRate)
   self:resize(com:size())
   for i=1, com:size()-2 do
      self(i):assign((com(i+1)-com(i-1))*frameRate*0.5)
   end
   self(0):assign(self(1))
   self(self:size()-1):assign(self(self:size()-2))
end
function vector3N:__tostring()
	return self:matView():__tostring()
end

function vector3N:copy()
	local out=vector3N()
	out:assign(self)
	return out
end
defineDerived(vector3N, {vector3NView}, {'smooth', "copy", "__eq", "at","sample", "derivative", "__tostring"})

function Ogre.SceneNode:transform(t)
   self:setOrientation(t.rotation)
   self:setPosition(t.translation)
end


function math.smoothTransition(a)
   return (-2.0)*a*a*a+(3.0)*a*a -- y=-2x^3+3x^2
end

function math.simpleSpline(timing, value)

   timing=math.__checkVec(timing)
   value=math.__checkVec(value)

   local out=vectorn()
   out:setSize(math.round(timing(timing:size()-1))+1)

   for i=0, timing:size()-2 do
      local t1=math.round(timing(i))
      local t2=math.round(timing(i+1))
      out:range(t1, t2+1):smoothTransition(value(i),value(i+1), t2-t1+1)
   end

   return out
end

function math.linearCurve(timing, value)
   return math.piecewiseLinearCurve(timing, value)
end

function math.piecewiseLinearCurve(timing, value)
   timing=math.__checkVec(timing)
   value=math.__checkVec(value)

   local out=vectorn()
   out:setSize(math.round(timing(timing:size()-1))+1)

   for i=0, timing:size()-2 do
      local t1=math.round(timing(i))
      local t2=math.round(timing(i+1))
      out:range(t1, t2+1):linspace(value(i),value(i+1), t2-t1+1)
   end

   return out
end

require('RigidBodyWin/subRoutines/PiecewiseLinearCurve')
-- type== array of "sinCurve", "simpleSpline", "elipseCurve", "piecewiseLinearCurve",..

function math.flexSpline(timing, value, type)
   timing=math.__checkVec(timing)
   value=math.__checkVec(value)

   local out=vectorn()
   out:setSize(math.round(timing(timing:size()-1))+1)

   for i=0, timing:size()-2 do
      local t1=math.round(timing(i))
      local t2=math.round(timing(i+1))
      local tp=type[i+1]
      
      out:range(t1,t2+1):assign(math[tp]({0,t2-t1},{value(i), value(i+1)}))
   end
   return out
end
function math.sinCurve(timing, value)
   timing=math.__checkVec(timing)
   value=math.__checkVec(value)

   assert(timing:size()==2)
   assert(value:size()==2)

   local out=vectorn()
   out:setSize(math.round(timing(timing:size()-1))+1)

   local halfPi=math.pi/2
   local a=value(1)-value(0)
   local b=value(0)

   for i=0,out:size()-1 do
      local rad=sop.map(i,0, out:size()-1,0,halfPi)
      out:set(i, math.sin(rad)*a+b)
   end
   return out
end

-- slope goes from somewhere to zero
function sop.mapSin(i, a, b, c,d)
   local halfPi=math.pi/2
   local rad=sop.map(i,a, b, 0, halfPi)
   local aa=d-c
   return math.sin(rad)*aa+c
end

-- slope goes from zero to somewhere
function sop.mapCos(i,a,b,c,d)
   local halfPi=math.pi/2
   local rad=sop.map(i,a, b, 0, halfPi)
   return sop.map(math.cos(rad), 1,0, c, d)
end

function math.clamp(a,mina,maxa)
   if a<mina then
      return mina
   elseif a>maxa then
      return maxa
   end
   return a
end

function math.powCurve(timing, value, exp)
   timing=math.__checkVec(timing)
   value=math.__checkVec(value)

   assert(timing:size()==2)
   assert(value:size()==2)

   local out=vectorn()
   out:setSize(math.round(timing(timing:size()-1))+1)

   local a=value(1)-value(0)
   local b=value(0)
   for i=0, out:size()-1 do
      local domain=sop.map(i,0,out:size()-1,0,1)
      out:set(i, math.pow(domain, exp)*a+b)
   end
   return out
end

function math.SQR(a)
   return a*a
end

function math.elipseCurve(timing, value, exp)
   timing=math.__checkVec(timing)
   value=math.__checkVec(value)

   assert(timing:size()==2)
   assert(value:size()==2)

   local out=vectorn()
   out:setSize(math.round(timing(timing:size()-1))+1)

   local a=value(1)-value(0)
   local b=value(0)
   for i=0, out:size()-1 do
      local domain=1-sop.map(i,0,out:size()-1,0,1)
      out:set(i, math.sqrt(1-domain*domain)*a+b)
   end
   return out
end

function math.spline(timing, value)
   timing=math.__checkVec(timing)
   value=math.__checkVec(value)
   
   local out=vectorn()
   out:setSize(math.round(timing(timing:size()-1))+1)

   local curveFit=math.NonuniformSpline(timing, value:column())

   local timing2=vectorn()
   timing2:linspace(0, out:size()-1, out:size())

   curveFit:getCurve(timing2, out:column())

   return out
end   

--namespace 
stitch={}

function stitch._stitch(mot, mot1, mot2, smoothness)
   assert(mot1:cols()==mot2:cols())
   assert(mot:rows()==mot1:rows()+mot2:rows()-1)
   assert(mot:cols()==mot1:cols())
   
   if smoothness==nil then
      smoothness=0
   end
   
   if smoothness==-1 then
      local c0c=m2.c0concat()
      c0c(mot, mot1, mot2)
   elseif smoothness==0 then
      local c0s=m2.c0stitch()
      c0s(mot, mot1, mot2)
   elseif smoothness==1 then
      local c1s=m2.c1stitchPreprocess(mot1:rows(), mot2:rows(), 2, false)
      c1s(mot, mot1, mot2)			
   elseif smoothness==4 then
      local c1s=m2.c1stitchPreprocess(mot1:rows(), mot2:rows(), 2, false)
      m2.quaterNN_linstitch(c1s, mot, mot1, mot2)
   end		
end

function stitch._stitch_online(mot, mot1, mot2, smoothness)
   assert(mot1:cols()==mot2:cols())
   assert(mot:rows()==mot1:rows()+mot2:rows()-1)
   assert(mot1:rows()==2)
   assert(mot:cols()==mot1:cols())
   
   if smoothness==nil then
      smoothness=0
   end
   
   if smoothness==-1 then
      local c0c=m2.c0concat()
      c0c(mot, mot1, mot2)
   elseif smoothness==0 then
      local c0s=m2.c0stitchOnline()
      c0s(mot, mot1, mot2)
   elseif smoothness==1 then
      local c1s=m2.c1stitchPreprocessOnline(mot1:rows(), mot2:rows(), 2)
      c1s(mot, mot1, mot2)			
   elseif smoothness==4 then
      local c1s=m2.c1stitchPreprocessOnline(mot1:rows(), mot2:rows(), 2)
      m2.quaterNN_linstitch(c1s, mot, mot1, mot2)
   end		
end

function stitch.matrixn(mot1, mot2, spread, smoothness)
   local len1=mot1:rows()-1
   local len2=mot2:rows()-1
   local len=len1+len2
   
   --dbg.startTrace()

   mot=matrixn(len+1, mot1:cols())
   
   --	print("stitch", mot:rows(), mot:cols())
   assert(len1>spread)
   --assert(len2>spread)
   if len2<=spread then
      spread=len2-1
   end
   
   local concatOnly=false
   
   local function mrange_c(mot, first, last)
      return mot:range(first, last+1, 0, mot:cols())
   end
   mrange_c(mot, 0, len1-spread):assign(mrange_c(mot1, 0, len1-spread))
   mrange_c(mot, len1+spread, len):assign(mrange_c(mot2, spread, len2))
   
   --	print("stitch", mot:rows(), mot:cols())
   if concatOnly then
      mrange_c(mot, len1-spread, len1):assign(
	 mrange_c(mot1, len1-spread, len1))
      mrange_c(mot, len1, len1+spread):assign(
	 mrange_c(mot2, 0, spread))
   else
      stitch._stitch(mrange_c(mot, len1-spread, len1+spread),
		     mrange_c(mot1, len1-spread, len1),
		     mrange_c(mot2, 0, spread), smoothness)
   end
   --	print("stitch", mot:rows(), mot:cols())
   
   return mot
end

function stitch.matrixn_online(mot1, mot2, spread, smoothness)
   local len1=mot1:rows()-1
   local len2=mot2:rows()-1
   local len=len1+len2
   
   mot=matrixn(len+1, mot1:cols())
   
   if len2<=spread then
      spread=len2-1
   end
   
   local function mrange_c(mot, first, last)
      return mot:range(first, last+1, 0, mot:cols())
   end
   
   mrange_c(mot, 0, len1-1):assign(mrange_c(mot1, 0, len1-1))
   mrange_c(mot, len1+spread, len):assign(mrange_c(mot2, spread, len2))
   
   stitch._stitch_online(mrange_c(mot, len1-1, len1+spread),
			 mrange_c(mot1, len1-1, len1),
			 mrange_c(mot2, 0, spread), smoothness)
   
   return mot
end

-- append vector3N, quaterN, vectorn, ...
function stitch.concat_vec(inout, input)
   local inout_size=inout:size()
   
   if inout_size==0 then
      inout:assign(input)
   else
      inout:resize(inout_size+input:size()-1)
      inout:range(inout_size-1, inout:size()):assign(input:range(0, input:size()))
   end
end


-- append matrixn
function stitch.concat_mat(inout, input)
   local inout_size=inout:rows()
   
   if inout_size==0 then
      inout:assign(input)
   else
      inout:resize(inout_size+input:rows()-1, input:cols())
      inout:range(inout_size-1, inout:rows(),0, input:cols()):assign(input)
   end
end

function stitch.append_vec(inout, input, spread, smoothness)
   local inout_size=inout:size()
   
   if inout_size==0 then
      inout:assign(input)
   else
      local prevLen=inout_size-1
      assert(prevLen>spread)
      
      inout:resize(prevLen+input:size())
      local newLen=inout:size()-1

      local function range_c(v, first, last)
	 return v:matView(first, last+1)
      end
      
      local mot1=range_c(inout, prevLen-spread, prevLen)
      local mot2=input:matView(0, input:size())
      
      --		print("mot1size", mot1:rows(), mot2:rows(), spread, prevLen, newLen)
      
      -----------------------------------------
      -- prevLen-spread                   newLen(92)
      --         prevLen(74)     mot1:11 mot2: 19 
      --             |
      --   |
      --                                     |
      range_c(inout, prevLen-spread, newLen):assign(stitch.matrixn(mot1, mot2, spread-1, smoothness))
   end
end

function stitch.append_mat(inout, input, spread, smoothness)
   local inout_size=inout:rows()
   
   if inout_size==0 then
      inout:assign(input)
   else
      local prevLen=inout_size-1
      assert(prevLen>spread)
      assert(inout:cols()==input:cols())
      inout:resize(prevLen+input:rows(), input:cols())
      local newLen=inout:rows()-1

      local function range_c(v, first, last)
	 return v:range(first, last+1, 0, v:cols())
      end
      
      local mot1=range_c(inout, prevLen-spread, prevLen)
      local mot2=input
      
      --		print("mot1size", mot1:rows(), mot2:rows(), spread, prevLen, newLen)
      
      -----------------------------------------
      -- prevLen-spread                   newLen(92)
      --         prevLen(74)     mot1:11 mot2: 19 
      --             |
      --   |
      --                                     |
      range_c(inout, prevLen-spread, newLen):assign(stitch.matrixn(mot1, mot2, spread-1, smoothness))
   end
end


function stitch.append_vec_online(inout, input, spread, smoothness)
   local inout_size=inout:size()
   
   if inout_size==0 then
      inout:assign(input)
   else
      local prevLen=inout_size-1
      assert(prevLen>spread)
      
      inout:resize(prevLen+input:size())
      local newLen=inout:size()-1

      local function range_c(v, first, last)
	 return v:matView(first, last+1)
      end
      
      local mot1=range_c(inout, prevLen-1, prevLen)
      local mot2=input:matView(0, input:size())
      
      range_c(inout, prevLen-1, newLen):assign(stitch.matrixn_online(mot1, mot2, spread-1, smoothness))
   end
end

Displacement=LUAclass()
function Displacement:__init(skel)
   self.skel=skel
   self.map=vectorn()
   self.remainingFrames=0
   self.totalDuration=0
end

function Displacement:packState()
	local out={}
	out.map=self.map:copy()
	out.remainingFrames=self.remainingFrames
	out.totolDuration=self.totalDuration
	return out
end

function Displacement:unpackState(instate)
	self.map=instate.map
	self.remainingFrames=instate.remainingFrames
	self.totolDuration=instate.totalDuration
end

function Displacement:__finalize()
   self.skel=nil
   self.map=nil
end

function Displacement:calc(pose1, pose2)
   self.map:resize(pose1:size())
   self.map:sub(pose2, pose1)
   local rootOri=quater()
   rootOri:difference(pose1:toQuater(3), pose2:toQuater(3))
   self.map:setQuater(3, rootOri)
end

function Displacement:setAutoDecay(duration)
	self.totalDuration=duration
	self.remainingFrames=duration
end
function Displacement:autoApply(inout, update)

	if self.remainingFrames>0 then
		self:apply(inout, sop.clampMap(self.remainingFrames, 0, self.totalDuration, 0, 1))
		if update then
			self.remainingFrames=self.remainingFrames-1
		end
	end
end

function Displacement:apply(inout, importance)

   local rootOri=inout:toQuater(3)
   local displacement=self.map:toQuater(3)
   displacement:scale(importance)
   --   debug.debug()
   inout:radd(self.map*importance)
   inout:setQuater(3, displacement*rootOri)
end

function table.radd(tbl,key,val)
	tbl[key]=tbl[key]+val
end
function Displacement:apply2(inout, startTime, currFrame, duration)
   
   if startTime~=-1 then
      local importance=sop.clampMap(currFrame, 
				    startTime,
				    startTime+duration,
				    1,
				    0)

      if importance~=0 then

	 self:apply(inout, importance)
      end
   end
end

DisplacementPos=LUAclass()
function DisplacementPos:__init(duration)
   self.startTime=-1
   self.disp=vector3()
   self.duration=duration
end

function DisplacementPos:calc(time, pos1, pos2)
   self.disp:sub(pos2,pos1)
   self.startTime=time
end

function DisplacementPos:apply(time, pos)
   local importance=sop.clampMap(time, self.startTime,
				 self.startTime+self.duration,
				 1,0)
   if importance~=0 then
      pos:assign(pos+self.disp*importance)
	  return true
   end
   return false
end

DisplacementMaps=LUAclass()
function DisplacementMaps:__init(duration)
	self.duration=duration
	self.maps=array()
	self.startTime=-1
end
function DisplacementMaps:calc(time, pos1, pos2)
	local newmap=DisplacementPos(self.duration)
	newmap:calc(time, pos1, pos2)
	self.maps:pushBack(newmap)
end
function DisplacementMaps:apply(time, pos)
	local c=0
	for i,map in ipairs(self.maps) do
		if not map:apply(time, pos) then
			c=c+1
		end
	end
	for i=1,c do
		self.maps:popFront()
	end
end

function DisplacementMaps:toTable()
	return {"__userdata", "DisplacementMaps", LUAclass_getProperty(self, true)}
end
function DisplacementMaps.fromTable(t)

	local map=DisplacementMaps(t[3].duration)

	for i=1,#t[3].maps do
		map.maps[i]=DisplacementPos(t[3].duration)
	end
	LUAclass_setProperty(map,t[3])
	return map
end

DisplacementRot=LUAclass()
function DisplacementRot:__init(duration)
   self.startTime=-1
   self.disp=quater()
   self.duration=duration
end

function DisplacementRot:calc(time, pos1, pos2)
   self.disp:difference(pos1, pos2)
   self.startTime=time
end

function DisplacementRot:apply(time, pos)
   local importance=sop.clampMap(time, self.startTime,
				 self.startTime+self.duration,
				 1,0)
   if importance~=0 then
      local tt=self.disp:copy()
      tt:scale(importance)
      pos:assign(tt*pos)
   end
end



--class 'SegmentFinder'
SegmentFinder=LUAclass()

function SegmentFinder:__init(discontinuity)
   self.vStart=intvectorn()
   self.vEnd=intvectorn()
   self.vStart:pushBack(0)
   for i=1, discontinuity:size()-1 do
      if discontinuity(i) then
	 self.vEnd:pushBack(i)
	 self.vStart:pushBack(i)
      end
   end
   self.vEnd:pushBack(discontinuity:size())
   assert(self.vStart:size()==self.vEnd:size())

end

function SegmentFinder:numSegment()
   return self.vStart:size()
end

function SegmentFinder:startFrame(iseg)
   return self.vStart(iseg)
end

function SegmentFinder:endFrame(iseg)
   return self.vEnd(iseg)
end



function MotionDOF.transformation(pose, starti)
	return transf(pose:toQuater(starti+3), pose:toVector3(starti));
end

function MotionDOF.setTransformation(pose, starti, t)
	pose:setQuater(3+starti, t.rotation);
	pose:setVec3(starti, t.translation);
end

function MotionDOF.mergeRoot(pose2, dpose2)
	local tf1=MotionDOF.transformation(pose2,0)
	local tf2=MotionDOF.transformation(pose2,7)
	--assert(math.abs(tf2.rotation:length()-1)<0.001)
	local tf=tf1*tf2
	tf.rotation:normalize()

	local v=vectorn(pose2:size()-7)
	v:range(7,v:size()):assign(pose2:range(14,pose2:size()))
	MotionDOF.setTransformation(v,0,tf)

	if dpose2 then
		-- assumes self-local local lin and ang vel
		local dpose=vectorn(pose2:size()-7)
		local v1=dpose2:toVector3(0)
		local v2=dpose2:toVector3(7)
		local w1=dpose2:toVector3(4)
		local w2=dpose2:toVector3(11)
		
		local vv=rotate(v1,tf1.rotation)+rotate(v2,tf.rotation)
		local w=rotate(w1,tf1.rotation)+rotate(w2,tf.rotation)
		local ri=tf.rotation:inverse()
		vv:rotate(ri)
		w:rotate(ri)
		dpose:assign(dpose2:range(7,dpose2:size()))
		dpose:setVec3(0, vv)
		dpose:setVec3(4, w)
		return v, dpose
	end
	return v
end
function MotionDOF:convertFromDeltaRep(starttf)
   local id=InterframeDifference()
   id:initFromDeltaRep(starttf, self:matView())
   id:reconstruct(self, self.dofInfo:frameRate())
end

function MotionDOF.convertDPoseToDState(pose, dpose, numSphericalJoint)

	--assert(false) -- incorrect. do not use
	
	local dstate=dpose:copy()
	-- convert linVel and angVel from parent-local to self-local
	dstate:setVec3(0, rotate(dstate:toVector3(0), pose:toQuater(3):inverse()))
	dstate:setVec3(4, rotate(dstate:toVector3(4), pose:toQuater(3):inverse()))
	if numSphericalJoint==2 then
		dstate:setVec3(7+0, rotate(dstate:toVector3(7+0), pose:toQuater(7+3):inverse()))
		dstate:setVec3(7+4, rotate(dstate:toVector3(7+4), pose:toQuater(7+3):inverse()))
	end
	return dstate
end

function math.blendLen(weightS, weightE, lena, lenb)
	local avgWeight=(weightS+weightE)/2
	local len=lena*(1-avgWeight)+lenb*avgWeight
	len=math.round(len)
	return len
end

function MotionDOF.blend(weightS, weightE, a, b) -- a, b should be a delta representation
	local len=math.blendLen(weightS, weightE, a:length(), b:length())
	local c=MotionDOF(a.dofInfo)
	c:resize(len+1)
	local posea=vectorn()
	local poseb=vectorn()
	for i=0, len do
		a:matView():sampleRow(a:length()*(i/len), posea)
		b:matView():sampleRow(b:length()*(i/len), poseb)
		a.dofInfo:blendDelta(c:row(i), posea, poseb, sop.map(i, 0, len, weightS, weightE))
	end
	return c
end

function MainLib_VRMLloader_calcTotalMass(skel)
	local mass=0
	for i=1,skel:numBone()-1 do
		mass=mass+skel:VRMLbone(i):mass()
	end
	return mass
end

function MotionDOF.diffPose(dofInfo, time_step, out, p1, p2)
	-- pose difference is tricky because of quaternions and angles
	-- on the other hand, velocity difference is easy
	-- even for ball joint:
	--  v1=dR
	out:sub(p2,p1)
	VRMLloader.projectAngles(out) -- align angles
	out:rmult(1/time_step)

	local q=quater()
	local v=vector3()
	for ibone=1, dofInfo:skeleton():numBone()-1 do
		if dofInfo:hasQuaternion(ibone) then
			local qi=dofInfo:startR(ibone)
			q:difference(p1:toQuater(qi), p2:toQuater(qi))
			v=q:rotationVector()
			v:scale(1/time_step)
			out:set(qi,0) -- unused
			out:setVec3(qi+1,v)
		end
	end
end

function vector3N.blend(weightS, weightE, a, b)
	assert(a:rows()>0)
	assert(b:rows()>0)
	local len=math.blendLen(weightS, weightE, a:rows()-1, b:rows()-1)
	local c=vector3N(len+1)
	local posea=vector3()
	local poseb=vector3()

	local len2=len
	if len2==0 then len2=1 end
	for i=0, len do
		posea=a:sampleRow((a:rows()-1)*(i/len2))
		poseb=b:sampleRow((b:rows()-1)*(i/len2))
		c:row(i):interpolate(sop.map(i, 0, len2, weightS, weightE), posea, poseb)
	end
	return c
end

function quaterN.blend(weightS, weightE, a, b)
	local len=math.blendLen(weightS, weightE, a:size()-1, b:size()-1)
	local c=quaterN(len+1)
	local posea=quater()
	local poseb=quater()
	local len2=len
	if len2==0 then len2=1 end
	for i=0, len do
		posea=a:sampleRow((a:size()-1)*(i/len2))
		poseb=b:sampleRow((b:size()-1)*(i/len2))
		c:row(i):interpolate(sop.map(i, 0, len2, weightS, weightE), posea, poseb)
	end
	return c
end
function vectorn.blend(weightS, weightE, a, b)
	local len=math.blendLen(weightS, weightE, a:size()-1, b:size()-1)
	local c=vectorn(len+1)
	local len2=len
	if len2==0 then len2=1 end
	for i=0, len do
		local posea=a:sample((a:size()-1)*(i/len2))
		local poseb=b:sample((b:size()-1)*(i/len2))
		c:set(i, sop.map(sop.map(i, 0, len2, weightS, weightE),0,1, posea, poseb))
	end
	return c
end
function matrixn.blend(weightS, weightE, a, b) 
	if a:cols()~=b:cols() then
		if a:cols()>b:cols() then
			return a:copy()
		else
			return b:copy()
		end
	end
	local len=math.blendLen(weightS, weightE, a:rows()-1, b:rows()-1)
	local c=matrixn(len+1, a:cols())
	local len2=len
	if len2==0 then len2=1 end
	for i=0, len do
		local posea=vectorn()
		a:sampleRow((a:rows()-1)*(i/len2),posea)
		local poseb=vectorn()
		b:sampleRow((b:rows()-1)*(i/len2),poseb)
		c:row(i):interpolate(sop.map(i, 0, len2, weightS, weightE), posea, poseb)
	end
	return c
end


function Fltk.ChooseFile(title, path, mask, write)
	if write==nil then write=false end
	local fn=Fltk.chooseFile(title, path, mask,write)
	if fn=="" then return nil end
	return fn
end

function MotionDOF:copy()
   local a=MotionDOF(self.dofInfo)
   a:assign(self)
   return a
end

function MotionDOFview:copy()
   return MotionDOF.copy(self)
end

MotionDOFcontainer=LUAclass()

function MotionDOFcontainer:exportMot(filename)

   local binaryFile=util.BinaryFile()
   binaryFile:openWrite(filename)
   binaryFile:packInt(2) -- version
   binaryFile:packInt(self.mot:numFrames())
   binaryFile:packInt(self.mot.dofInfo:numDOF())
   binaryFile:pack(self.mot:matView())
   binaryFile:pack(self.discontinuity)
   binaryFile:pack(self.conL)
   binaryFile:pack(self.conR)
   if self.signals then
	   for k,v in pairs(self.signals) do --signals has to be matrixn type
		   binaryFile:pack(k)
		   binaryFile:pack(v)
	   end
	   binaryFile:pack('(end)')
   end
   binaryFile:close()
end

function MotionDOFcontainer:sub(startF, endF)

	local out=MotionDOFcontainer(self.mot.dofInfo)
	out.mot:assign(self.mot:range(startF, endF))
	out.discontinuity:assign(self.discontinuity:range(startF, endF))
	out.conL:assign(self.conL:range(startF, endF))
	out.conR:assign(self.conR:range(startF, endF))
	for k,v in pairs(self.signals) do
		out.signals[k]=v:sub(startF, endF):copy()
	end
	return out
end

function extractConstraints(mot, type)

   discontinuity=boolN(mot:numFrames())

   for i=0,mot:numFrames()-1 do
      discontinuity:set(i, mot:isConstraint(i, type))
   end

   return discontinuity
end

function MotionDOFcontainer.loadToTable(fn)
	local out={}
	local binaryFile=util.BinaryFile()
	binaryFile:openRead(filename)
	local version=binaryFile:unpackInt()
	assert(version==1) -- version
	out.numFrames= binaryFile:unpackInt()
	out.numDOF=binaryFile:unpackInt()
	out.data=matrixn()
	binaryFile:unpack(out.data)
	
	local self=out
	self.discontinuity=boolN()
	self.conL=boolN()
	self.conR=boolN()
	binaryFile:unpack(self.discontinuity)
	binaryFile:unpack(self.conL)
	binaryFile:unpack(self.conR)
	binaryFile:close()
	return out
end

function MotionDOFcontainer:resample(src, skip)
	local nf=math.floor(src:numFrames()/skip)-skip
	self:resize(nf)
	for i=0, nf-1 do
		self.mot:row(i):assign(src.mot:row(i*skip))
		self.conL:set(i, src.conL(i*skip))
		self.conR:set(i, src.conR(i*skip))
		if i==0 then
			assert(src.discontinuity(0))
			self.discontinuity:set(0, true)
		else
			self.discontinuity:set(i, src.discontinuity:range((i-1)*skip+1, i*skip+1):count(true)>0)
		end
	end
end
function MotionDOFcontainer:__init(dofInfo, filename)
   self.mot=MotionDOF(dofInfo)
   self.discontinuity=boolN()
   self.conL=boolN()
   self.conR=boolN()
   self.signals={}
   
   if filename~=nil then
	   if type(filename)~='string' then
		   local motdof=filename
		   self:resize(motdof:numFrames())
		   self.mot:assign(motdof)
	   elseif string.upper(str.right(filename,3))=="MOT" then
	    local tempMot=Motion(dofInfo:skeleton())
	    dofInfo:skeleton():loadAnimation(tempMot, filename)
	    self.discontinuity=extractConstraints(tempMot,FootstepDetection.IS_DISCONTINUOUS)
	    self.conL=extractConstraints(tempMot,FootstepDetection.CONSTRAINT_LEFT_FOOT)
	    self.conR=extractConstraints(tempMot,FootstepDetection.CONSTRAINT_RIGHT_FOOT)
	    self.mot:set(tempMot)
	elseif string.upper(str.right(filename,3))=="BVH" then

		local dofScale=0.01 -- millimeters to meters
		local dofRot=quater(math.rad(-90), vector3(0,1,0))
		*quater(math.rad(-90), vector3(1,0,0)) -- change to Y_up

		local loader1=RE.motionLoader(filename)
		local loader2=RE.motionLoader(filename)
		rotateSkeleton(loader1, loader2,dofRot)

		-- convert to meter (millimeters, Z_up)
		loader2:scale(dofScale)
	    local tempMot=loader2.mMotion

	    self.mot:set(tempMot)
	 else
	    local binaryFile=util.BinaryFile()
	    binaryFile:openRead(filename)
	    local version=binaryFile:unpackInt()
	    self:resize(binaryFile:unpackInt())
	    local numDOF=binaryFile:unpackInt()

	    if numDOF~=self.mot.dofInfo:numDOF() then
	       print("Warning! numDOF doesn't match! importing data to self.mat", numDOF, self.mot.dofInfo:numDOF())
	       self.mat=matrixn()
	       binaryFile:unpack(self.mat)
	    else
	       binaryFile:unpack(self.mot:matView())
	    end
	    binaryFile:unpack(self.discontinuity)
	    binaryFile:unpack(self.conL)
	    binaryFile:unpack(self.conR)
		if version==2 then
			while true do
				local k=binaryFile:unpackStr()
				if k=='(end)' then break end
				local mat=matrixn()
				binaryFile:unpack(mat)
				self.signals[k]=mat
			end
		else assert(version==1)
		end
	    binaryFile:close()
	 end
   end
end

function MotionDOFcontainer:resize(nframes)
   self.mot:resize(nframes)
   self.discontinuity:resize(nframes)
   self.conL:resize(nframes)
   self.conR:resize(nframes)
end

function MotionDOFcontainer:concat(mot) -- mot is not a container type but MotionDOF itself.
   self:resize(self:numFrames()+mot:numFrames())
   self.mot:range(self:numFrames()-mot:numFrames(),
		  self:numFrames()):assign(mot)
   self.discontinuity:set(self:numFrames()-mot:numFrames(), true)
end
function MotionDOFcontainer:upsample(nup)
   omot=self.mot
   odis=self.discontinuity
   oconL=self.conL
   oconR=self.conR

   self.mot=MotionDOF(omot.dofInfo)
   self.mot.dofInfo:setFrameRate(omot.dofInfo:frameRate()*nup)

   assert(self.mot.dofInfo:frameRate()==omot.dofInfo:frameRate()*nup)

   local segFinder=SegmentFinder(odis)

   local newFrame=omot:numFrames()*nup
   self.mot:resize(newFrame)
   self.discontinuity=boolN(newFrame)
   self.conL=boolN(newFrame)
   self.conR=boolN(newFrame)
   local cols=omot:matView():cols()
   for ii=0, segFinder:numSegment()-1 do
      local s=segFinder:startFrame(ii)
      local e=segFinder:endFrame(ii)
      
      if s~=e then

	 for kk=s,e-2 do

	    local q=omot(kk+1):toQuater(3)
	    if omot(kk):toQuater(3):dot(q)<0 then
	       omot(kk+1):setQuater(3, -q)
	    end
	 end
	 local timing=vectorn()
	 timing:linspace(0, e-s-1, e-s)
	 local curveFit=math.NonuniformSpline(timing, omot:matView():range(s,e, 0, cols))

	 local timing2=vectorn()
	 local last=(e-s-1)*nup
	 timing2:linspace(0, e-s-1, last+1)

	 curveFit:getCurve(timing2, self.mot:matView():range(s*nup, s*nup+last+1, 0, cols))


	 for j=s*nup+last+1, e*nup-1 do
	    self.mot:row(j):assign(self.mot:row(s*nup+last))
	 end

	 for kk=s*nup,e*nup-1 do

	    local q=self.mot(kk):toQuater(3)
	    q:normalize()
	    self.mot(kk):setQuater(3,q)
	 end


	 self.discontinuity:set(s*nup, true)
      end
      
   end

   for i=0, self.conL:size()-1 do
      local tt=math.min(math.round(i/nup), omot:numFrames()-1)
      self.conL:set(i, oconL(tt))
      self.conR:set(i, oconR(tt))
   end

   print(omot:numFrames(), self:numFrames())
end      


function MotionDOFcontainer:numFrames()
   return self.mot:numFrames()
end
function MotionDOFcontainer:row(i)
   return self.mot:row(i)
end



function MotionDOFcontainer:isConstraint(iframe, type)
   if type==Motion.IS_DISCONTINUOUS then
      return self.discontinuity(iframe)
   elseif type==FootstepDetection.CONSTRAINT_LEFT_FOOT then
      return self.conL(iframe)
   else
      assert(type==FootstepDetection.CONSTRAINT_RIGHT_FOOT)
      return self.conR(iframe)
   end
end



function MotionDOFcontainer:setConstraint(iframe, type, val)
   if type==Motion.IS_DISCONTINUOUS then
      self.discontinuity:set(iframe, val)
   elseif type==FootstepDetection.CONSTRAINT_LEFT_FOOT then
      self.conL:set(iframe, val)
   else
      assert(type==FootstepDetection.CONSTRAINT_RIGHT_FOOT)
      self.conR:set(iframe, val)
   end
end

if Liegroup then
	function Liegroup.dse3:__tostring()
		return "m: "..self(0)..','..self(1)..','..self(2).." f: "..self(3)..','..self(4)..','..self(5)
	end
	function Liegroup.Inertia:__tostring()
		return "I: "..tostring(self:getDiag()).." "..tostring(self:getSymm())..
		" r: "..tostring(self:getOffDiag())
	end
end
--require('scilua')

liegroup={}
liegroup.dse3=LUAclass()

function liegroup.dse3:__tostring()
	return "m: "..self.m:__tostring().." f: "..self.f:__tostring()
end

function liegroup.dse3:__init(m, f, mz, fx, fy, fz)
	if mz then
		self.m=vector3(m,f,mz)
		self.f=vector3(fx,fy,fz)
	elseif m and f then
		self.m=m
		self.f=f
	else
		self.m=vector3()
		self.f=vector3()
	end
end

function liegroup.dse3:copy()
	return liegroup.dse3(self.m, self.f)
end

function liegroup.dse3:dAd(T)
	local tmp=self.m-T.translation:cross(self.f)
	local invR=T.rotation:inverse()
	return liegroup.dse3(rotate(tmp,invR), rotate( self.f, invR))
end

liegroup.se3=LUAclass()
function liegroup.se3:__init(w, v, wz, vx, vy, vz)
	if wz then
		self.w=vector3(w,v,wz)
		self.v=vector3(vx,vy,vz)
	elseif w then
		if v then
			self.w=w
			self.v=v
		else
			self.w=w:toVector3(0)
			self.v=w:toVector3(3)
		end
	else
		self.w=vector3()
		self.v=vector3()
	end
end
function matrix4:radd(mat)
	self:assign(self+mat)
end
function matrix4:setSkew(w)
	self._11=0
	self._12=-1*w.z
	self._13=w.y
	self._21=w.z
	self._22=0
	self._23=-1*w.x
	self._31=-1*w.y
	self._32=w.x
	self._33=0
end
function matrix4:invSkew()
	local out=vector3()
	out.z=0.5*(self._21-self._12)
	out.y=0.5*(self._13-self._31)
	out.x=0.5*(self._32-self._23)
	return out
end

function liegroup.se3:to_dse3()
	return liegroup.dse3(self.w, self.v)
end
function liegroup.se3:M()
	local m=matrix4()
	m:setSkew(self.w)
	m:leftMultTranslation(self.v)
	return m
end
function liegroup.se3:V()
	local w=self.w
	local v=self.v
	return CT.vec(w.x, w.y, w.z, v.x, v.y, v.z)
end
function liegroup.Ad(t)
	local m6=matrixn(6,6)
	local theta=m6:sub(0,3,0,3)
	theta:assign33(t:M())
	m6:sub(0,3,3,6):zero()
	m6:sub(3,6,0,3):mult(CT.skew(t.translation),theta)
	m6:sub(3,6,3,6):assign(theta)
	return m6
end
function liegroup.dAd(t)
	--return liegroup.Ad(t):Transpose()
	local m6=matrixn(6,6)
	local theta=m6:sub(0,3,0,3)
	theta:assign33(t:inverse():M())
	m6:sub(0,3,3,6):mult(theta, CT.skew(-t.translation))
	m6:sub(3,6,0,3):zero()
	m6:sub(3,6,3,6):assign(theta)
	return m6
end
function liegroup.invdAd(t)
	--return self:dAd(t:inverse()) -- easy-to-understand-but-slow version
	local m6=matrixn(6,6)
	local theta=m6:sub(0,3,0,3)
	theta:assign33(t:M())
	m6:sub(0,3,3,6):mult(theta, CT.skew(-t.rotation:inverse()*t.translation))
	m6:sub(3,6,0,3):zero()
	m6:sub(3,6,3,6):assign(theta)
	return m6
end

function liegroup.se3:copy()
	return liegroup.se3(self.w, self.v)
end
function liegroup.se3:Ad_pos(p)
	return liegroup.se3(self.w, p:cross(self.w)+self.v)
end
function liegroup.se3:Ad_ori(r)
	return liegroup.se3(rotate(self.w, r), rotate(self.v, r))
end
-- result=T*[self]*Inv(T)
function liegroup.se3:Ad(T)
	local tmp=rotate(self.w,T.rotation )
	return liegroup.se3(tmp, T.translation:cross(tmp)+rotate(self.v, T.rotation))
end

function liegroup.se3:__tostring()
	return "w: "..self.w:__tostring().." v: "..self.v:__tostring()
end

function matrix4:multScale(b)
	local o=matrix4()
	o:assign(self)
	o:rmult(b)
	return o
end
function matrix4:zero()
	self:identity()
	self:rmult(0)
end
function transf:M()
	local m=matrix4()
	m:setRotation(self.rotation)
	m:leftMultTranslation(self.translation)
	return m
end
function transf:twist(tf2, timestep)
	local t1=matrix4()
	t1:setRotation(self.rotation)
	t1:leftMultTranslation(self.translation)
	local t2=matrix4()
	t2:setRotation(tf2.rotation)
	t2:leftMultTranslation(tf2.translation)
	local dotT=(t2-t1):multScale(1/timestep)
	local v=t1:inverse()*dotT
	return liegroup.se3(vector3(v._23*-1, v._13, v._12*-1), 
						vector3(v._14, v._24, v._34))
end
function transf:twist_nonlinear(tf2, timestep)
	-- dotT*tf1=tf2
	local inv_tf1=self:inverse()
	local dotT_timestep=tf2*inv_tf1
	local dotT=Liegroup.se3()
	dotT:log(dotT_timestep)
	-- body-vel= exp(Ad_{inv_t1}*v) *theta_dot
	-- where t2=exp(v theta)*t1
	local invt=1/timestep
	return liegroup.se3(dotT:W()*invt, dotT:V()*invt):Ad(inv_tf1)
end

function transf:integrate(V, timestep)
	local t1=matrix4()
	t1:setRotation(self.rotation)
	t1:leftMultTranslation(self.translation)
	local t1inv_dotT=matrix4()
	t1inv_dotT:zero()
	t1inv_dotT:setSkew(V.w)
	t1inv_dotT:leftMultTranslation(V.v)
	t1inv_dotT:rmult(timestep) -- t2-t1=dotT*timestep
	t1:radd(t1*t1inv_dotT)
	self:assign(t1)
end

function Motion:rootTransformation(iframe)
   local pose=self:pose(iframe)
   return transf(pose.rotations(0), pose.translations(0))
end

function Motion:assign(mot)
	self:init(mot, 0, mot:numFrames())
end

function MotionUtil.renormalize(motionDOF)
   local dofInfo=motionDOF.dofInfo
   for i=1, dofInfo:skeleton():numBone()-1 do
      if dofInfo:hasQuaternion(i) then
	 motionDOF:quatViewCol(dofInfo:startR(i)):normalize()
      end
   end
end

function MotionUtil.insertRootJoint(skel, matRootPos, matRootOri, rootName, srcMot)
	assert(skel:VRMLbone(1):HRPjointType(0)==MainLib.VRMLTransform.FREE)

	local bone=skel:VRMLbone(1)
	skel:insertChildJoint(bone, 'XYZ', 'ZXY', '__temp', true)
	skel:VRMLbone(2):setName(skel:VRMLbone(1):name())
	skel:VRMLbone(1):setName(rootName)

	local q0=quater()
	local q1=quater()
	local nmot=MotionDOF(skel.dofInfo)
	nmot:resize(srcMot:numFrames())
	local nc=nmot:cols()
	assert(nc==srcMot:cols()+7)

	local mot=srcMot
	local roottf=transf()
	local matRoot=transf()
	for i=0,srcMot:numFrames()-1 do
		roottf=MotionDOF.rootTransformation(srcMot:row(i))
		matRoot.translation:assign(matRootPos:row(i):toVector3(0))
		matRoot.rotation:assign(matRootOri:row(i):toQuater(0))
		-- matRoot* x =roottf
		local x=matRoot:inverse()*roottf
		nmot:row(i):range(14,nmot:cols()):assign(mot:row(i):range(7, mot:cols()))
		MotionDOF.setRootTransformation(nmot:row(i), matRoot)
		nmot:row(i):setVec3(7, x.translation)
		nmot:row(i):setQuater(10, x.rotation)
	end
	skel:_changeVoca(MotionLoader.HIPS, bone:childHead());

	return nmot
end

function util.BinaryFile:unpackUserData()
   local out=nil
   local type=self:_unpackInt()
--   print("type: "..type)
   if type==util.BinaryFile.TYPE_INT then
      out=self:_unpackInt()
   elseif type==util.BinaryFile.TYPE_FLOAT then
      out=self:_unpackFloat()
   elseif type==util.BinaryFile.TYPE_FLOATN then
      local v=vectorn()
      self:_unpackVec(v)
      out=v
   elseif type==util.BinaryFile.TYPE_FLOATMN then
      local m=matrixn()
      self:_unpackMat(m)
      out=m
   elseif type==util.BinaryFile.TYPE_STRING then
      out=self:_unpackStr()
   elseif type==util.BinaryFile.TYPE_EOF then
      return nil
   elseif type==util.BinaryFile.TYPE_BITN then
      local bb=boolN()
      self:_unpackBit(bb)
      out=bb
      print(bb)
   else
      print("UnknownType"..tostring(type))
   end
--   print("unpack userdata")
   return out
end 

function util.BinaryFile:load(filename)
   self:openRead(filename)

   local out={}
   local cc=1
   while true do

      local v=self:unpackUserData()
      if v==nil then break end
      out[cc]=v
      cc=cc+1
   end
   return out
end
SaveTable={
   clone=function (t) local nt={}; for i,v in pairs(t) do nt[i]=v end return nt end
}

function SaveTable:pickle_(root, fn)
    if type(root) ~= "table" then 
    error("can only pickle tables, not ".. type(root).."s")
  end
  self._tableToRef = {}
  self._refToTable = {}
  local savecount = 0
  self:ref_(root)
  local s = util.BinaryFile()
  s:openWrite(fn)

  while table.getn(self._refToTable) > savecount do
    savecount = savecount + 1
    local t = self._refToTable[savecount]
    for i, v in pairs(t) do
       if type(v)=="table" then self:ref_(v) end
    end
  end

  local numRef=table.getn(self._refToTable)

  s:packInt(numRef)
  for i=numRef,1,-1 do

     local tbl=self._refToTable[i]

     for k, v in pairs(tbl) do

	s:packInt(1)
	if type(v)=="table" then
	   s:packInt(1) -- reference
	   s:packAuto(k)
	   s:packInt(self._tableToRef[v])
	else
	   s:packInt(0)
	   s:packAuto(k)
	   s:packAuto(v)
	end
     end
     s:packInt(0)
  end
  s:close()
end

function util.BinaryFile:packBool(b)
   if b then self:packInt(1)
   else self:packInt(0)
   end
end
function util.BinaryFile:unpackBool()
   local b=self:unpackInt()
   if b ==1 then return true end
   return false
end

function util.BinaryFile:packAuto(v)
   if type(v)=="number" then
      self:packInt(0)
      self:packFloat(v)
   elseif type(v)=="string" then
      self:packInt(1)
      self:pack(v)
   elseif type(v)=="boolean" then
      self:packInt(2)
      self:packBool(v)
   elseif type(v)=="userdata" then
      if v.w then
		  if type(v.w)=="function" then
			  self:packInt(6) -- quaterN
		  else
			  self:packInt(4) 	-- quater
		  end
      elseif v.z then
		  if type(v.z)=="function" then
			  self:packInt(7) -- vector3N 
		  else
			  self:packInt(3)	-- vector3
		  end
      else
	 self:packInt(5)	-- vectorn or matrixn
      end
      self:pack(v)
   elseif type(v)=="table" then assert(false) end
end

function util.BinaryFile:unpackAuto()
   local type=self:unpackInt()
   local v
   if type==0 then
      v=self:unpackFloat()
   elseif type==1 then
      v=self:unpackStr()
   elseif type==2 then
      v=self:unpackBool()
   elseif type==3 then
      v=vector3()
      self:unpack(v)
   elseif type==4 then
      v=quater()
      self:unpack(v)
  elseif type==6 then
	  v=quaterN()
	  self:unpack(v)
  elseif type==7 then
	  v=vector3N()
	  self:unpack(v)
   else
      v=self:unpackUserData()
   end
   return v
end



function SaveTable:ref_(t)
  local ref = self._tableToRef[t]
  if not ref then 
    if t == self then error("can't pickle the pickle class") end
    table.insert(self._refToTable, t)
    ref = table.getn(self._refToTable)
    self._tableToRef[t] = ref
  end
  return ref
end


function SaveTable.unpickle_(fn)

 local s = util.BinaryFile(true) -- load to memory.
  if not s:openRead(fn) then
     print(fn)
     if fineLog then fineLog("openReadFailed"..fn) end
--     error("openRead"..fn)
  end

  if fineLog then fineLog("openRead"..fn) end
   
  local _tableToRef = {}
  local _refToTable = {}

  local numRef=s:unpackInt()

  for i=numRef,1,-1 do

     _refToTable[i]={}
     local tbl=_refToTable[i]

     while s:unpackInt()==1 do
	local ref=s:unpackInt()
	if ref==1 then
	   k=s:unpackAuto()
--	   print("unpack", k)
	   local iii=s:unpackInt()
	   v=_refToTable[iii]
	else
	   k=s:unpackAuto()

--	   print("unpack", k)
	   v=s:unpackAuto()
	end

--	print("unpack", v)
	tbl[k]=v
     end
  end

  s:close()
  return _refToTable[1]
end


function util.compareTable(tbl1, tbl2,level, key)
	assert(tbl1~=nil)
	assert(tbl2~=nil)

	if level==nil then level=0 key="" end

	local count=0
	local maxCount=10
	local ignorePattern=util.compareTableIgnore
	for k,v in pairs(tbl1) do
		if k ~=ignorePattern then

			if type(v)=="table" then
				if type(tbl2[k])~="table" then
					print(string.rep("  ", level).."2x  "..tostring(k))
				else
					count=count+util.compareTable(v, tbl2[k], level+1, key.."."..k)
					if level==0 and count>maxCount then
						dbg.console()
						return
					end
				end
			else
				local neq=tbl2[k]~=v
				if type(v)=='number' then
					if not select(1,pcall(math.abs, tbl2[k]-v)) then dbg.console() end
					neq=math.abs(tbl2[k]-v)>__globals.EQ_THR
				end

				if neq then
					print(key)
					print(string.rep("  ", level).."2X  "..tostring(k))
					count=count+1
					if level==0 and count>maxCount then
						dbg.console()
						return
					end
					-- else
					--    print(string.rep("  ", level).."O  "..tostring(k))
				end
			end
		end
	end

	for k,v in pairs(tbl2) do
		if tbl1[k]==nil then
			print(string.rep("  ", level).."1X  "..tostring(k))
		end
	end

	if level==0 then
		if count~=0 then
			tt1=tbl1
			tt2=tbl2

			function comp(k)
				print(tt1[k])
				print(tt2[k])
			end

			--	 debug.debug()
			dbg.console()

		else
			print("identical")
		end
	end

	return count
end

function util.saveTable(tbl, filename)
	assert(filename~=nil and type(filename)=="string")
   SaveTable:clone():pickle_(tbl, filename)
end


function util.loadTable(filename)
	assert(util.isFileExist(filename))
   return SaveTable.unpickle_(filename)
end

function FlLayout:addFloatSlider(title, val, vmin, vmax)
   assert(float_options~=nil)
   float_options[title]=val
   self:create("Value_Slider", title, title,1)
   self:widget(0):sliderRange(vmin, vmax)
   self:widget(0):sliderValue(val)
end

function FlLayout:updateFloatOptions(w)
   for k,v in  pairs(float_options) do
      if w:id()==k then
	 float_options[k]=w:sliderValue()
	 break
      end
   end
end
function FlLayout:menuItems(...)
	local tbl={...}
	self:widget(0):menuSize(table.getn(tbl))
	for i=1, table.getn(tbl) do
		self:widget(0):menuItem(i-1, tbl[i])
	end
end


function MotionLoader:setVoca(bones)
	local strToVoca=
	{	 
		["hips"]=MotionLoader.HIPS,
		["left_hip"]=MotionLoader.LEFTHIP,
		["left_knee"]=MotionLoader.LEFTKNEE,
		["left_heel"]=MotionLoader.LEFTANKLE,
		["left_ball"]=MotionLoader.LEFTTOES,
		["left_collar"]=MotionLoader.LEFTCOLLAR,
		["left_shoulder"]=MotionLoader.LEFTSHOULDER,
		["left_elbow"]=MotionLoader.LEFTELBOW,
		["left_wrist"]=MotionLoader.LEFTWRIST,
		["right_hip"]=MotionLoader.RIGHTHIP,
		["right_knee"]=MotionLoader.RIGHTKNEE,
		["right_heel"]=MotionLoader.RIGHTANKLE,
		["right_ball"]=MotionLoader.RIGHTTOES,
		["right_collar"]=MotionLoader.RIGHTCOLLAR,
		["right_shoulder"]=MotionLoader.RIGHTSHOULDER,
		["right_elbow"]=MotionLoader.RIGHTELBOW,
		["right_wrist"]=MotionLoader.RIGHTWRIST,
		["chest"]=MotionLoader.CHEST,
		["chest2"]=MotionLoader.CHEST2,
		["neck"]=MotionLoader.NECK,
		["head"]=MotionLoader.HEAD,
	}
	for k,v in pairs(bones) do
		if type(v)=="string" then
			if strToVoca[k] then
				self:_changeVoca(strToVoca[k], self:getBoneByName(v))
			else
				print("error?", k)
				debug.debug()
			end
		else
			print("error2?")
		end
	end

	-- automatic vocaburary assignment of child bones
	local function derive(voca_parent, voca_child)

		if self:getTreeIndexByVoca(voca_parent)~=-1 
			and self:getTreeIndexByVoca(voca_child)==-1 then
			local pbone=self:getBoneByVoca(voca_parent)
			local pchildbone=pbone:childHead()
			if pchildbone~=nil then
				assert(pchildbone:voca()==-1)
				self:_changeVoca(voca_child, pchildbone)
			end
		end
	end

	derive(MotionLoader.LEFTCOLLAR, MotionLoader.LEFTSHOULDER)
	derive(MotionLoader.LEFTSHOULDER, MotionLoader.LEFTELBOW)
	derive(MotionLoader.LEFTELBOW, MotionLoader.LEFTWRIST)
	derive(MotionLoader.RIGHTCOLLAR, MotionLoader.RIGHTSHOULDER)
	derive(MotionLoader.RIGHTSHOULDER, MotionLoader.RIGHTELBOW)
	derive(MotionLoader.RIGHTELBOW, MotionLoader.RIGHTWRIST)

	if false then -- print name-voca map
		print("voca")
		for i=1, self:numBone()-1 do
			print(self:bone(i):name(), self:bone(i):voca())
		end
	end
end



if RE.motionPanel==nil or RE.motionPanel()==nil then
	-- console mode compatibilty functions
	RE._motionPanel=
		{
			motionWin=function()
				return 
				{
					changeCurrFrame=function() end,
					detachSkin=function() end,
					addSkin=function() end,
					detachAllSkin=function() end
				}
			end
		}

	function RE.motionPanel()
		return RE._motionPanel
	end
	if not RE.rendererValid or not RE.rendererValid() then
		require('RigidBodyWin/subRoutines/fastMode')
	end
end

function os.encodeToDivx(folderName, outputFileName)
      
	local jpegFile='../'..folderName..'/00001.jpg'
	if os.isUnix() then
		--if string.upper(ext)=="WMV" or arg[2]=="-r" then
			--os.execute('avidemux --load '..arg[1]..' --save '..fn.."_r.avi --quit")
			--fn_ext=fn.."_r.avi"
		--end
		-- msmpeg4v2
		--os.execute('mencoder '..fn_ext..' -o '..fn..'_.avi -ovc lavc -oac lavc -lavcopts acodec=mp3:vcodec=msmpeg4v2 -of lavf -lavfopts format=avi')
		-- mpeg4 (divx)
		--os.execute('mencoder '..fn_ext..' -o '..fn..'_.avi -ovc lavc -oac lavc -lavcopts acodec=mp3:vcodec=mpeg4:vbitrate=2000 -of lavf -lavfopts format=avi')
		-- nosound (libavcodec)
		--os.execute('mencoder '..fn_ext..' -o '..fn..'_.avi -ovc lavc -nosound -lavcopts vcodec=mpeg4:vbitrate=2000 -of lavf -lavfopts format=avi')
		-- nosound (xvid)
		--os.execute('mencoder '..fn_ext..' -o '..fn..'_.avi -nosound -ovc xvid -xvidencopts bitrate=2000:me_quality=6:pass=1')

		local fn=os.processFileName(folderName)
		if false then
			os.deleteFiles(folderName..'/00000.jpg')
			os.execute2('cd virtualDub', 'avidemux --load '..jpegFile..' --fps 30 --save _'..fn..".avi --quit")
			os.execute('avidemux --load virtualDub/_'..fn..'.avi&')
			--os.execute2('cd virtualDub', 'mencoder _'..fn..'.avi'..' -o '..outputFileName..' -ovc lavc -nosound -lavcopts vcodec=mpeg4:vbitrate=4000 -of lavf -lavfopts format=avi')
			os.execute2('cd virtualDub', 'avconv -i _'..fn..'.avi'..' -y -b 2000k '..outputFileName..' -mbd rd -flags +mv4+aic -trellis 2 -cmp 2 -subcmp 2 -g 300 -pass 2 ')
			--os.deleteFiles(folderName..'/*.jpg')
		else
			if os.isFileExist('virtualDub/'..outputFileName) then
				os.deleteFiles('virtualDub/'..outputFileName)
			end
			os.execute2('cd virtualDub', 'avconv -i ../'..folderName..'/%05d.jpg -r 30 -y -b 2000k '..outputFileName..' -mbd rd -flags +mv4+aic -trellis 2 -cmp 2 -subcmp 2 -g 300 -pass 2 ')
			if os.isFileExist('virtualDub/'..outputFileName) then
				os.execute('avidemux --load virtualDub/'..outputFileName..'&')
				os.deleteFiles(folderName..'/*.jpg')
			end
		end
	else
		local inputFileName=string.gsub(jpegFile,"/","\\\\")
		--createVirtualDubJobFile("default_divx.vcf", inputFileName, outputFileName)
		local configFile='default_divx.vcf'
		do	
			local file, msg=io.open(configFile,"r")
			local sfile, smsg=io.open("virtualDub/jobs.script", "w")

			if file==nil then
				print(msg)
				return
			end

			if sfile==nil then
				print(smsg)
				return
			end
			sfile:write("VirtualDub.Open(\""..inputFileName.."\",\"\",0);\n")

			for line in file:lines() do
				sfile:write(line.."\n")
			end

			sfile:write("VirtualDub.project.ClearTextInfo();\n")
			sfile:write("// -- $reloadstop --\n")
			sfile:write("VirtualDub.SaveAVI(\""..outputFileName.."\");\n")
			sfile:write("VirtualDub.audio.SetSource(1);\n")
			sfile:write("VirtualDub.Close();\n")
			sfile:close()
			file:close()
		end
		os.execute("runJobs.bat")
	end
end

-- multiple line strings are not indented correctly in emacs lua-mode. 
-- so I defined them separately here.
gnu_octave.plotVec1=[[
function plotVec1(T, Y)
	%figure(1)
	hold off;
	subplot(1,1,1);
	T
	Y
	plot(T,Y)
	print()
	%			print("foo.png", "-dpng")
	]]


	gnu_octave.plotVec2=[[
	function plotVec2(T, Y)
		%figure(1)
		hold on;
		subplot(1,1,1);
		T
		Y
		plot(T,Y)
		print()
		%			print("foo.png", "-dpng")
		]]

