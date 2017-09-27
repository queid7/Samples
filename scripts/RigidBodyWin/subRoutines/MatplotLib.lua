MatplotLib=LUAclass()

function MatplotLib:__init()
	print('matplotLib wrapper started')
	self.pythonCodes=array()
	self:addLine(
	[[
# -*- coding: UTF-8 -*-
from numpy import *
import matplotlib
matplotlib.use('Agg')
from pylab import *
import sys
import os
	]]
	)
end
function MatplotLib:add(...)
	local args={...}
	local c=""
	for i,arg in ipairs(args) do
		if type(arg)=='table' and getmetatable(arg) then

		else
			c=c..tostring(arg)
		end
	end
	self:addLine(c)
end
function MatplotLib:legends(...)
	self:add("legend(('"..table.concat({...}, "','").."'))")
end
function MatplotLib:legends2(options, ...)
	local optionstr=self.optionstring(options)
	self:add("legend(('"..table.concat({...}, "','").."')"..optionstr..")")
end
-- 0-indexing. set plot region to (xth row, yth column)
function MatplotLib:subplot(x,y) 	
	self:add('ax=subplot(',self.param.nsubfig[1],',',self.param.nsubfig[2],',',(x+y*self.param.nsubfig[1])+1,')')
end
function MatplotLib:set_xlim(a,b)
	a=tostring(a)
	b=tostring(b)
	self:add('ax.set_xlim('..a..','..b..')')
end
function MatplotLib:set_ylim(a,b)
	a=tostring(a)
	b=tostring(b)
	self:add('ax.set_ylim('..a..','..b..')')
end

-- figure{figid, nsubfig={nrow, ncol}, subfig_size={width, height}}
function MatplotLib:figure(param)
	local defaultParam={
		1, -- figure id
		nsubfig={1,1},
		subfig_size={12,3},
	}
	if param then
		assert(type(param)=='table')
		param=table.merge(defaultParam, param)
		local p=param
		local fig_id=p[1]
		self:add('figure(',fig_id,', figsize=(',p.nsubfig[2]*p.subfig_size[1],
			',',p.nsubfig[1]*p.subfig_size[2],'))')
	else
		param=defaultParam
		self:add('figure(1)')
	end
	self.param=param
end

function MatplotLib:tostr(a)
	local str="("
	if type(a)=='userdata' and getmetatable(a) and getmetatable(a).luna_class=="LVec" then
		for i=1,a:size() do
			str=str..tostring(a(i))..","
		end
	else
		for i=0,a:size()-1 do
			str=str..tostring(a(i))..","
		end
		-- print('unsupported format:',a)
	end
	return str..")"
end
function MatplotLib.optionstring(options)
	local optionstr='';
	for k,v in pairs(options) do
		if type(v)=='string' then
			optionstr=optionstr..','..k.."='"..v.."'"
		else
			optionstr=optionstr..','..k.."="..v..""
		end
	end
	return optionstr
end
-- options: {linestyle='dotted'}
function MatplotLib:plot(a,b, options)
	assert(a:size()==b:size())
	if options then
		local optionstr=self.optionstring(options)
		self:add('plot('..self:tostr(a)..","..self:tostr(b)..optionstr..')')
	else
		self:add('plot('..self:tostr(a)..","..self:tostr(b)..')')
	end
end
function MatplotLib:savefig(fn)
	self:add('savefig("'..fn..'")')
	self.fn=fn
	if os.isUnix() then
		self:add('os.system("gnome-open '..fn..'")')
	else
		self:add('os.system("start '..fn..'")')
	end
end
function MatplotLib:xlabel(a)
	self:add('xlabel("'..a..'")')
end
function MatplotLib:ylabel(a)
	self:add('ylabel("'..a..'")')
end
function MatplotLib:addLine(l)
	self.pythonCodes:pushBack(l)
end
function MatplotLib:close()
	self:add('close(1)')
	self.fn=self.fn or 'temp'
	util.writeFile(self.fn..'.py', table.concat(self.pythonCodes,'\n'))
	os.execute('python '..self.fn..'.py')
	print('matplotLib wrapper finished (see '..self.fn..' )')
	self:__init()
end

   
