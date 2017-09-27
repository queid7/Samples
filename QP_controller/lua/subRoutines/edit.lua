
require("config")

require("module")

--dbg.startTrace()
function ctor()
	local prefix='MWSnap'
	--local images={'27.jpg','17.jpg','9.jpg',  '0.jpg', }
	local images={}
	for i=1, 13 do 
		images[i]=string.format('%03d', i)..'.bmp'
	end
	local cropX={108, 1146}
--	local cropX={{151,337},{181,357}, {206,375},{277,448}, }
	local cropY={308,866 }
	
	if type(cropX[1])~='table' then
		local cropX_old=cropX
		cropX={}
		for i=1, table.getn(images) do
			cropX[i]=deepCopyTable(cropX_old)
		end
	end
	
	cropMerge(prefix, images, cropX, cropY)
--	crop(prefix, images, cropX, cropY)


	this("exit",{})
end

function dtor()
end

function onCallback(w, userData)
end

function frameMove(fElapsedTime)
end
function crop(prefix, images, cropX, cropY)
	local lim={}
	local offset=10

	for i, fn in ipairs(images) do
	end

	for i, cropx in ipairs(cropX) do
		lim[i]=CImage()
		local fn=images[i]
		lim[i]:Load(prefix..fn)
		local out=CImage()
		out:create(cropx[2]-cropx[1], cropY[2]-cropY[1])
		out:blit(lim[i], TRect(cropx[1], cropY[1], cropx[2], cropY[2]), 0, 0)
		out:save(prefix..fn..'_crop.jpg',24)
	end
end
function cropMerge(prefix, images, cropX, cropY)
	local lim={}
	local offset=10

	for i, fn in ipairs(images) do
		lim[i]=CImage()
		lim[i]:Load(prefix..fn)
	end

	local totalWidth=0
	for i, cropx in ipairs(cropX) do
		totalWidth=totalWidth+cropx[2]-cropx[1]+offset
	end
	totalWidth=totalWidth-offset
	local out=CImage()
	out:create(totalWidth, cropY[2]-cropY[1])
	local w=0
	for i, cropx in ipairs(cropX) do
		out:blit(lim[i], TRect(cropx[1], cropY[1], cropx[2], cropY[2]), w, 0)
		w=w+cropx[2]-cropx[1]+offset
	end
	out:save(prefix..'.jpg',24)
	out:Save(prefix..'2.jpg')
end
