#!/usr/bin/python
import os

#cd ../../work;./ysbatch "/home/yoonsang/Data/Research/2013_4_newQP/Code/taesooLib_yslee/Samples/ysscripts/samples/batchcapture.lua" "print('hihi');prefix='test';require('useCaseMuscle_full_tong_repeat');useCase=useCases.full_tong_repeat"

ysbatch = 'cd ../../work;./ysbatch'

batchcapture = '/home/yoonsang/Data/Research/2013_4_newQP/2_SIGASIA_Submit/Code/yslee/Samples/ysscripts/samples/batchcapture.lua'
batchcapture_reference = '/home/yoonsang/Data/Research/2013_4_newQP/2_SIGASIA_Submit/Code/yslee/Samples/ysscripts/samples/batchcapture_reference.lua'
capturescript = batchcapture_reference

usecase_pairs = [
        #('g2592_gait'),
        #('full_soldier'),
        #('full_lean'),
        #('full_srun'),
        #('full_frun'),
        #('full_tong'),
        #('full_ipfrun'),
        #('full_ipnrun'),
        ('full_cmurun'),    # to upload
        ]

for usecase_pair in usecase_pairs:
    usecase = usecase_pair

    savename = '%s_%s'%('ref',usecase)
    luacode = 'print(\'%s\');prefix=\'%s\';require(\'useMuscle_%s\');useCase=useCases.%s'%\
                (savename, savename, usecase, usecase)

    os.system('%s \"%s\" \"%s\"'%(ysbatch, capturescript, luacode))
    #print '%s \"%s\" \"%s\"'%(ysbatch, capturescript, luacode)
