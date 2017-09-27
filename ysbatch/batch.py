#!/usr/bin/python
import os
import numpy as np

def pyobj2luatablestr(obj, depth=0, iskey=False): 
    indent = '\t'*depth
    indent_elem = '\t'*(depth+1)

    if isinstance(obj, list):
        luastr = '{'+'\n'
        for value in obj:
            luastr += indent_elem+pyobj2luatablestr(value, depth+1)+',\n'
        luastr += indent+'}'
    elif isinstance(obj, dict):
        luastr = '{'+'\n'
        for key, value in obj.items():
            luastr += indent_elem+pyobj2luatablestr(key, depth+1,True)+'='
            luastr += pyobj2luatablestr(value, depth+1)+',\n'
        luastr += indent+'}'
    elif isinstance(obj, np.ndarray):
        luastr = '{'
        for v in obj:
            luastr += str(v)+', '
        luastr += '}'
    elif isinstance(obj, unicode):
        luastr = str(obj) if iskey else repr(obj.encode('ascii', 'ignore'))
    else:
        luastr = str(obj) if iskey else repr(obj)

    return luastr

#cd ../../work;./ysbatch "/home/yoonsang/Data/Research/2013_4_newQP/Code/taesooLib_yslee/Samples/ysscripts/samples/batchcapture.lua" "print('hihi');prefix='test';require('useCaseMuscle_full_tong_repeat');useCase=useCases.full_tong_repeat"

ysbatch = 'cd ../../work;./ysbatch'

#researchDir = '/home/yoonsang/Data/Research/'
researchDir = '/media/sda1/2.Phd-ubuntu/Research/'

batchcapture = researchDir+'2013_4_newQP/2_SIGASIA_Submit/Code/yslee/Samples/ysscripts/samples/batchcapture.lua'
batchcapture_reference = researchDir+'2013_4_newQP/2_SIGASIA_Submit/Code/yslee/Samples/ysscripts/samples/batchcapture_reference.lua'
capturescript = batchcapture

usecase_pairs = [

        ('g2592_gait', ['1x'], []),

        #('g2592_gait', ['a.1_push80_'], []),
        #('g2592_tong', ['a.01_mscl2_push160_'], []),
        #('g2592_gait', ['a.1_push40_'], []),
        #('g2592_tong', ['a.01_mscl2_push100_'], []),
        #('g2592_gait', ['a.1_push120_'], []),
        #('g2592_tong', ['a.01_mscl2_push220_'], []),

        #('g2592_soldier', ['a_.1'], []),
        #('g2562_soldier', ['a_.1'], []),
        #('full_soldier', ['default'], []),

        #('g2592_lean', ['a_.1'], []),
        #('g2562_lean', ['a_.1'], []),
        #('full_lean', ['default'], []),

        #('g2592_srun', ['a_.01'], []),
        #('g2562_srun', ['a_.01'], []),
        #('full_srun', ['default'], []),

        #('g2592_frun', ['a_.01'], []),
        #('g2562_frun', ['a_.01'], []),
        #('full_frun', ['a_.01'], []),

        #('g2592_gait', ['a.1_lfcfm5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_efrtm5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_efcim5'], ['met','eng','eff','spd']),

        #('g2592lhip_gait', ['a.1_efrtm5'], ['met','eng','eff','spd']),
        #('g2592lhip_gait', ['a.1_efcim5'], ['met','eng','eff','spd']),
        #('g2592lhip_gait', ['a.1'], ['met','eng','eff','spd']),

        #('g2592_gait', ['a.1_kneeLEX.15'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_kneeLEX.15_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_ankLPF.2'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_ankLPF.2_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hipAB.3'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hipAB.3_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hipLAB.3'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hipLAB.3_efcim5'], ['met','eng','eff','spd']),

        #('g2592_gait', ['a.1_ankPF.2_efcim5'], []),
        #('g2592_gait', ['mscl_gluts.3'], []),
        #('g2592_gait', ['a_.1_mscl_lglut.2'], []),
        #('g2592_gait', ['a_.1_obj_lfootcf'], []),
        #('g2592lhip_gait', ['a_.1'], []),

        #('g2592_lean', ['a_.1'], ['met','eng','eff','spd']),
        #('g2592_soldier', ['a_.1'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_ankLPF.1'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_ankLPF.1_efcim5'], ['met','eng','eff','spd']),

        #('g2592_gait', ['a.1_ankLPF.01'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_ankLPF.01_efrtm5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_ankLPF.01_efcim5'], ['met','eng','eff','spd']),

        #('g2592_gait', ['a.1_ankLPF.1_efcim5'], []),

        #('g2592_gait', ['a.1_hipLAB.2'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hipLAB.2_efrtm5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hipLAB.2_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hipAB.2'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hipAB.2_efrtm5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hipAB.2_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_kneeLEX.1'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_kneeLEX.1_efrtm5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_kneeLEX.1_efcim5'], ['met','eng','eff','spd']),

        #('g2592dimx2_gait', ['a.1_ankLPF.01'], ['met','eng','eff','spd']),
        #('g2592dimx2_gait', ['a.1_ankLPF.01_efrtm5'], ['met','eng','eff','spd']),
        #('g2592dimx2_gait', ['a.1_ankLPF.01_efcim5'], ['met','eng','eff','spd']),

        #('g2592dimx2_gait', ['a.1_ankLPF.01'], ['met','eng','eff','spd']),
        #('g2592dimx2_gait', ['a.1_kneeLEX.1'], ['met','eng','eff','spd']),
        #('g2592dimx2_gait', ['a.1_hipLAB.2'], ['met','eng','eff','spd']),
        #('g2592dimx2_gait', ['a.1_lfcfm5'], ['met','eng','eff','spd']),
        #('g2592dimx2lhip_gait', ['a.1'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hipAB.2'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hamst_psoas_tl.8_ankPF_mf.2'], ['met','eng','eff','spd']),

        #('g2592_gait', ['a.1_hamst_psoas_tl.9_ankPF_mf.2'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hamst_psoas_ol.8_ankPF_mf.2'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hamst_psoas_ol.9_ankPF_mf.2'], ['met','eng','eff','spd']),

        #('g2592_gait', ['a.1_hamst_psoas_tl.8'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hamst_psoas_tl.9'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hamst_psoas_ol.8'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hamst_psoas_ol.9'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hamst_tl.8'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hamst_ol.8'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_psoas_tl.8'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_psoas_ol.8'], ['met','eng','eff','spd']),
        #('g2592lhip_gait', ['a.1_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_kneeLEX.2'], ['met','eng','eff','spd']),

        #('g2592_gait', ['a.1_kneeLEX.2_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_kneeEX.2'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_kneeEX.2_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_ankPF.1'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_ankPF.1_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hipAB.2_efrtm5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hipAB.2_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hipAB.3'], ['met','eng','eff','spd']),

        #('g2592_gait', ['a.1_kneeEX.15'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_kneeEX.15_efcim5'], ['met','eng','eff','spd']),

        #('g2592lhip_gait', ['a.1'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hipAB.3_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_lankpffom6'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_lrankpffom5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_lrankpffom6'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hamst_tlol.9'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_psoas_tlol.9'], ['met','eng','eff','spd']),

        #('g2592_gait', ['a.1_hamst_psoas_tlol.9'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hamst_psoas_tlol.9_ankPF_mf.2'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hamst_psoas_ol.6_ankPF_mf.2'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hamst_psoas_ol.7_ankPF_mf.2'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_lankpffom5'], ['met','eng','eff','spd']),

        #('g2592_gait', ['a.1_hamst_psoas_tl.8_ankPF_mf.2_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hamst_psoas_tl.8_ankPF_mf.2_efrtm5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hamst_psoas_ol.8_ankPF_mf.2_efcim5'], ['met','eng','eff','spd']),

        #('g2592_gait', ['a.1_hamst_psoas_ol.8_ankPF_mf.2_efrtm5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hipAB.15'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hipAB.15_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_kneeEX.15'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_kneeEX.15_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hipAB.4_efcim5'], ['met','eng','eff','spd']),

        #('g2592_gait', ['a.1_hipAB.4'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_lankpffom4'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_lrankpffom4'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_lankpffom6'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_lrankpffom6'], ['met','eng','eff','spd']),
        #('g2592lhip_gait', ['a.1'], ['met','eng','eff','spd']),

        #('g2592_gait', ['a.1_ankPF.15_efcim5'], ['met','eng','eff','spd']),
        #('g2592lhip_gait', ['a.1_efcim5'], ['met','eng','eff','spd']),
        #('g2592lhip_gait', ['a.1_efrtm5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hamst_psoas_tl.9_ankPF_mf.2_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hamst_psoas_tl.9_ankPF_mf.2_efrtm5'], ['met','eng','eff','spd']),

        #('g2592_gait', ['a.1_gluts.15'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_gluts.15_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_lankpffom3'], ['met','eng','eff','spd']),

        #('g2592_gait', ['a.1_lfcfm4'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_lfcfm5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_lfcfm6'], ['met','eng','eff','spd']),
        #('g2592lhip_gait', ['a.1'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_lrankpffom4'], ['met','eng','eff','spd']),
        #('full_frun', ['a_.01'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_ankPF.15_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_gluts.15'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_gluts.15_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_kneeEX.15'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_kneeEX.15_efcim5'], ['met','eng','eff','spd']),

        #('full_frun', ['a_.01'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_kneeLEX.15'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_kneeLEX.15_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_lfcfm4'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_lfcfm5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_lfcfm2'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_lfcfm3'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_kneeEX.05'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_kneeEX.05_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_kneeLEX.05'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_kneeLEX.05_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_gluts.1'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_gluts.1_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_gluts.2'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_gluts.2_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_lglut.1'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_lglut.1_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_lglut.2'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_lglut.2_efcim5'], ['met','eng','eff','spd']),

        #('g2592_gait', ['a.1_gluts.1_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_gluts.15_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_gluts.2_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_lglut.2_efcim4'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_lglut.2_efcim6'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_lfcfm3'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_lfcfm4'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_lfcfm5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_lfcfm6'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_kneeLEX.15_efcim6'], ['met','eng','eff','spd']),

        #('g2592_gait', ['a.1_lglut.2_efcim4'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_lglut.3_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_kneeLEX.15_efcim7'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_kneeEX.1_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_gluts.2_efcim4'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_gluts.2_efcim6'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_gluts.3_efcim4'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_gluts.3_efcim5'], ['met','eng','eff','spd']),

        #('g2592_gait', ['a.1_lglut.2_efcim4'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_gluts.3_efcim6'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_lglut.3_efcim4'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_lglut.3_efcim6'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_lglut.2_efcim6'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_lfcfm3'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_lfcfm4'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_lfcfm5'], ['met','eng','eff','spd']),

        #('g2592_gait', ['a.1_gluts.2_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_kneeLEX.1_efcim5'], ['met','eng','eff','spd']),

        #('g2592_gait', ['a_.1_obj_lfootcf'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_gluts.2_efcim4'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_lglut.2_efcim6'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_gluts.2_efcim5'], ['met','eng','eff','spd']),

        #('g2592_gait', ['a.1_gluts.4_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_gluts.5_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_lglut.4_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_lglut.5_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_gluts.2_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_lglut.2_efcim6'], ['met','eng','eff','spd']),

        #('full_cmurun', ['a_.01_mscl_8x'], ['met','eng','eff','spd']),
        #('g2562_cmurun', ['a_.01_mscl_8x'], ['met','eng','eff','spd']),
        #('g2592_cmurun', ['a_.01_mscl_8x'], ['met','eng','eff','spd']),
        #('full_ipfrun', ['default_mscl_4x'], ['met','eng','eff','spd']),

        #('g2592lhip_gait', ['a.1'], []), # not captured

        #('g2592_gait', ['a.1_gluts.4_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_lglut.6_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1'], ['met','eng','eff','spd']),
        #('g2592lhip_gait', ['a.1_efcim5'], ['met','eng','eff','spd']),
        #('g2562_ipfrun', ['a_.01_mscl_2x'], ['met','eng','eff','spd']),
        #('g2592_ipfrun', ['a_.01_mscl_2x'], ['met','eng','eff','spd']),
        #('full_ipnrun', ['a_.01'], ['met','eng','eff','spd']),
        #('g2562_ipnrun', ['a_.01'], ['met','eng','eff','spd']),

        #('g2592_ipnrun', ['a_.01'], ['met','eng','eff','spd']),
        #('full_cmurun', ['a_.01_mscl_4x'], ['met','eng','eff','spd']),
        #('full_cmurun', ['a_.01_mscl_2x'], ['met','eng','eff','spd']),
        #('g2562_cmurun', ['a_.01_mscl_4x'], ['met','eng','eff','spd']),

        #('g2562_ipnrun', ['a_.01'], ['met','eng','eff','spd']),
        #('g2592_ipfrun', ['a_.01'], ['met','eng','eff','spd']),
        #('full_ipfrun', ['a_.01_mscl_2x'], ['met','eng','eff','spd']),
        #('full_ipfrun', ['a_.01'], ['met','eng','eff','spd']),
        #('g2562_ipfrun', ['a_.01_mscl_2x'], ['met','eng','eff','spd']),
        #('g2562_ipfrun', ['a_.01'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_lglut.4_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_lglut.4_efcim6'], ['met','eng','eff','spd']),
        #('full_cmurun', ['a_.01_mscl_8x'], ['met','eng','eff','spd']),
        #('g2562_cmurun', ['a_.01_mscl_4x'], ['met','eng','eff','spd']),
        #('g2592_cmurun', ['a_.01_mscl_4x'], ['met','eng','eff','spd']),
        #('g2592_ipnrun', ['a_.01'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_biankpf_tl.95_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_biankpf_ol.7_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_ankDF.1'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_ankDF.1_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_ankDF.01'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_ankDF.01_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_ankLDF.1'], ['met','eng','eff','spd']),

        #('g2592_gait', ['a.1_ankLDF.1_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_ankLDF.01'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_ankLDF.01_efcim5'], ['met','eng','eff','spd']),
        #('g2562_ipfrun', ['a_.01_mscl_2x'], ['met','eng','eff','spd']),

        #('g2592_gait', ['a.1_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_ankDF.2_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_ankDF.1_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_ankDF.01_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_ankDF.05_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_ankLDF.2_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_ankLDF.1_efcim5'], ['met','eng','eff','spd']),

        #('g2592_gait', ['a.1_ankLDF.05_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_ankLDF.01_efcim5'], ['met','eng','eff','spd']),
        #('g2562_ipfrun', ['a_.01_mscl_2x'], ['met','eng','eff','spd']),

        #('g2562_ipfrun', ['a_.01_mscl_2x'], ['met','eng','eff','spd']),
        #('g2562_ipfrun', ['a_.1_mscl_2x'], ['met','eng','eff','spd']),
        #('g2562_ipfrun', ['a_1_mscl_2x'], ['met','eng','eff','spd']),
        #('g2562_ipfrun', ['a_.01_mscl_2x_4'], ['met','eng','eff','spd']),
        #('g2562_ipfrun', ['a_.01_mscl_2x_5'], ['met','eng','eff','spd']),
        #('g2562_ipfrun', ['a_.01_mscl_2x_6'], ['met','eng','eff','spd']),
        #('g2562_ipfrun', ['a_.01_mscl_2x_7'], ['met','eng','eff','spd']),
        #('g2562_ipfrun', ['a_.01_mscl_2x_8'], ['met','eng','eff','spd']),

        #('g2592_gait', ['a.1_hamst_psoas_tl.8_ankPF_mf.1_efcim5'], ['met','eng','eff','spd']),

        #('g2562_gait', ['a_1'], ['met','eng','eff','spd']),
        #('g2562_gait', ['a_.5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hamst_psoas_tl.8_ankPF_mf.2_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hamst_psoas_tl.8_ankPF_mf.2'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hamst_tl.8_psoas_tl2.6_ankPF_mf.2_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hamst_tl.8_psoas_tl2.6_ankPF_mf.2'], ['met','eng','eff','spd']),

        #('g2592_gait', ['a.1_ankLDF.03_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_ankLDF.02_efcim5'], ['met','eng','eff','spd']),
        #('g2562_gait', ['a_.5'], ['met','eng','eff','spd']),
        #('g2592_lean', ['a_.1'], ['met','eng','eff','spd']),

        #('full_soldier', ['a_.1'], ['met','eng','eff','spd']),
        #('g2562_gait', ['a_.5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_ankLDF.03_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_ankLDF.04_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hamst_psoas_tl.8_ankPF_mf.2_efcim5'], ['met','eng','eff','spd']),

        #('g2592_gait', ['a.1_hamst_tl.8_psoas_tl2.6_ankPF_mf.2_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hamst_psoas_tl.8_ankPF_mf.1_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hamst_tl.8_psoas_tl2.6_ankPF_mf.1_efcim5'], ['met','eng','eff','spd']),
        #('g2562_gait', ['a_.5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_ankLDF.03_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hamst_psoas_tl.8_ankPF_mf.2_efcim7'], ['met','eng','eff','spd']),

        #('g2592_gait', ['a.1_hamst_tl.8_psoas_tl2.6_ankPF_mf.2_efcim7'], ['met','eng','eff','spd']),
        #('g2562_gait', ['a_.3'], ['met','eng','eff','spd']),
        #('g2562_gait', ['a_.5_efcim8'], ['met','eng','eff','spd']),
        #('g2562_gait', ['a_.5_efcim7'], ['met','eng','eff','spd']),

        #('g2562_frun', ['a_.01_nost'], ['met','eng','eff','spd']),
        #('g2592_frun', ['a_.01_nost'], ['met','eng','eff','spd']),
        #('full_frun', ['a_.01_nost'], ['met','eng','eff','spd']),

        #('full_soldier', ['default_nost'], ['met','eng','eff','spd']),
        #('g2562_ipfrun', ['a_.01_mscl_2x_5_nost'], ['met','eng','eff','spd']),
        #('g2562_gait', ['a_.1'], ['met','eng','eff','spd']),
        #('g2562_gait', ['a_.5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hamst_psoas_tl.8_ankPF_mf.2_efcim5_clmpm3'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hamst_psoas_tl.8_ankPF_mf.2_efcim5_clmpm4'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hamst_psoas_tl.8_ankPF_mf.2_efcim5_clmpm5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_ankPF.2_efcim5_clmpm3'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_ankPF.2_efcim5_clmpm4'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_ankPF.2_efcim5_clmpm5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_ankPF.15_efcim5'], ['met','eng','eff','spd']),

        #('g2592_frun', ['a_.01'], ['met','eng','eff','spd']),
        #('g2592_frun', ['a_.01_mscl_2x'], ['met','eng','eff','spd']),
        #('g2562_soldier', ['a_.1'], ['met','eng','eff','spd']),

        #('full_soldier', ['a_.1_g2592'], ['met','eng','eff','spd']),
        #('full_soldier', ['a_.1_2'], ['met','eng','eff','spd']),
        #('full_soldier', ['a_.1_2_g2592'], ['met','eng','eff','spd']),

        #('g2592_gait', ['a.1_hamst_psoas_tl.8_ankPF_mf.2_efcim5_prec1.0'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hamst_psoas_tl.8_ankPF_mf.2_efcim5_h.2'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hamst_psoas_tl.8_ankPF_mf.2_efcim5_h.4'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hamst_psoas_tl.8_ankPF_mf.2_efcim5_h.6'], ['met','eng','eff','spd']),

        #('g2592_gait', ['a.1_hamst_psoas_tl.8_ankPF_mf.2_efcim5_prec.5'], ['met','eng','eff','spd']),
        #('g2562_ipfrun', ['a_.01_mscl_2x_5'], ['met','eng','eff','spd']),
        #('full_soldier', ['a_.1_g2592'], ['met','eng','eff','spd']),

        #('g2592_gait', ['a.1_hamst_psoas_tlol.9_ankPF_mf.2'], ['met','eng','eff','spd']),

        #('g2592_gait', ['a.1_hamst_psoas_tlol.83_ankPF_mf.2_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hamst_psoas_tlol.84_ankPF_mf.2_efcim5'], ['met','eng','eff','spd']),

        #('g2592_gait', ['a.1_hamst_psoas_tlol.85_ankPF_mf.2_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hamst_psoas_tlol.84_ankPF_mf.2_efcim2'], ['met','eng','eff','spd']),
        #('full_soldier', ['a_.1_g2592'], ['met','eng','eff','spd']),
        #('g2562_ipfrun', ['a_.01_mscl_2x_5'], ['met','eng','eff','spd']),

        #('g2592_gait', ['a.1_hamst_psoas_tlol.85_ankPF_mf.2_efcim2'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hamst_tlol.85_psoas_tlol2.8_ankPF_mf.2_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hamst_tlol.85_psoas_tlol2.8_ankPF_mf.2_efcim2'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hamst_psoas_tlol.85_ankPF_mf.2_efcim5_mod'], ['met','eng','eff','spd']),

        #('g2592_gait', ['a.1_hamst_psoas_tlol.9_ankPF_mf.2_efcim5_mod_dd_bl.1'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hamst_psoas_tlol.85_ankPF_mf.2_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hamst_psoas_tl.8_ankPF_mf.2_efcim5_clmpm5'], ['met','eng','eff','spd']),
        #('g2562_ipfrun', ['a_.01_mscl_2x_5'], ['met','eng','eff','spd']),

        #('g2592_gait', ['a.1_age30_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_age50_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_age70_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_age30_efcim3'], ['met','eng','eff','spd']),

        #('g2592_gait', ['a.1_age50_efcim3'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_age70_efcim3'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hamst_psoas_tl.8_ankPF_mf.2'], ['met','eng','eff','spd']),
        #('g2562_ipfrun', ['a_.01_mscl_2x_5'], ['met','eng','eff','spd']),

        #('g2592_gait', ['a.1_hamst_psoas_tl.8_ankPF_mf.2_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_age30_efcim5_cont'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_age50_efcim5_cont'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_age70_efcim5_cont'], ['met','eng','eff','spd']),

        #('g2592_gait', ['da1e1'], ['met','eng','eff','spd']),
        #('g2592_gait', ['da1e2'], ['met','eng','eff','spd']),
        #('g2592_gait', ['da1e3'], ['met','eng','eff','spd']),
        #('g2592_gait', ['da1e4'], ['met','eng','eff','spd']),
        #('g2592_gait', ['da1e5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['da1e6'], ['met','eng','eff','spd']),
        #('g2592_gait', ['da1e7'], ['met','eng','eff','spd']),

        #('g2592_gait', ['a.1_kneelim20m2'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_kneelim20m3'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_kneelim20m4'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_kneelim20m5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_kneelim30m2'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_kneelim30m3'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_kneelim30m4'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_kneelim30m5'], ['met','eng','eff','spd']),

        #('g2592_gait', ['a.1_kneelim20m6'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_kneelim20m7'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_kneelim30m6'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_kneelim30m7'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hamst_psoas_tl.8_ankPF_mf.2_kneelim20m4'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hamst_psoas_tl.8_ankPF_mf.2_kneelim20m5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hamst_psoas_tl.8_ankPF_mf.2_kneelim30m4'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_hamst_psoas_tl.8_ankPF_mf.2_kneelim30m5'], ['met','eng','eff','spd']),

        #('g2592_gait', ['a.1_kneelim20e2'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_kneelim20e1'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_kneelim30e2'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_kneelim30e1'], ['met','eng','eff','spd']),


        #('g2592_gait', ['a.1_back2x_knee20_hip0_m3_efrtm5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_back2x_knee20_hip10_m3_efrtm5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_back2x_knee20_hip20_m3_efrtm5'], ['met','eng','eff','spd']),
        #('g2592_gait', ['a.1_back2x_knee20_hip30_m3_efrtm5'], ['met','eng','eff','spd']),

        #('g2592_gait110', ['a.1','a.1_kneelim30_hiplim10_m4_efrtm5','a.1_back2x_knee20_hip0_m3_efrtm5'], ['met','eng','eff','spd']),
        #('g2592_gait120', ['a.1','a.1_kneelim30_hiplim10_m4_efrtm5','a.1_back2x_knee20_hip0_m3_efrtm5'], ['met','eng','eff','spd']),
        #('g2592_gait130', ['a.1','a.1_kneelim30_hiplim10_m4_efrtm5','a.1_back2x_knee20_hip0_m3_efrtm5'], ['met','eng','eff','spd']),
        #('g2592_gait140', ['a.1','a.1_kneelim30_hiplim10_m4_efrtm5','a.1_back2x_knee20_hip0_m3_efrtm5'], ['met','eng','eff','spd']),

        #('g2592_gait160', ['a.1_back2x_knee20_hip0_m3_efrtm5'], ['met','eng','eff','spd']),
        #('g2592_gait180', ['a.1_back2x_knee20_hip0_m3_efrtm5'], ['met','eng','eff','spd']),
        #('g2592_gait120', ['a.1_ankPF.2_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait140', ['a.1_ankPF.2_efcim5'], ['met','eng','eff','spd']),

        #('g2592_gait160', ['a.1_ankPF.2_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait180', ['a.1_ankPF.2_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait120', ['a.1_hamst_psoas_tl.8_ankPF_mf.2_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait140', ['a.1_hamst_psoas_tl.8_ankPF_mf.2_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait160', ['a.1_hamst_psoas_tl.8_ankPF_mf.2_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait180', ['a.1_hamst_psoas_tl.8_ankPF_mf.2_efcim5'], ['met','eng','eff','spd']),
        #('g2592_gait120', ['a.1_knee20_m3'], ['met','eng','eff','spd']),
        #('g2592_gait140', ['a.1_knee20_m3'], ['met','eng','eff','spd']),


        ###############################################3
        # final avi
        # driverDynamicsMuscle.lua: if true then at line 191

        # locomotion

        #('g2562_soldier', ['a_.1'], []),
        #('g2592_soldier', ['a_.1'], []),   # not captured
        ##('full_soldier', ['default'], []),
        #('full_soldier', ['a_.1_g2592'], []),

        #('g2562_lean', ['a_.1'], []),
        #('g2592_lean', ['a_.1'], []),
        #('full_lean', ['default'], []),

        #('g2562_srun', ['a_.01'], []),
        #('g2592_srun', ['a_.01'], []),
        #('full_srun', ['default'], []), # not captured

        #('g2562_frun', ['a_.01'], []),
        #('g2592_frun', ['a_.01'], []),
        #('full_frun', ['a_.01'], []),

        #('g2562_tong', ['a_.01_mscl_2x'], []),
        #('g2592_tong', ['a_.01_mscl_2x'], []),  # range_error
        #('full_tong', ['a_.01_mscl_2x'], []),

        #('g2562_ipfrun', ['a_.01_mscl_2x_5'], []),   
        #('g2592_ipfrun', ['a_.01_mscl_2x'], []),   
        #('full_ipfrun', ['a_.01_mscl_2x'], []),   

        #('g2562_ipnrun', ['a_.01_mscl_2x'], []),    
        #('g2592_ipnrun', ['a_.01_mscl_2x'], []),    
        #('full_ipnrun', ['a_.01_mscl_2x'], []), 

        #('g2562_gait', ['a_.1'], []),
        #('g2562_gait', ['a_.5'], []),
        #('g2592_gait', ['a.1'], []),    

        ## push

        #('g2592_gait', ['a.1_push80_'], []),
        #('g2592_tong', ['a.01_mscl2_push160_'], []),



        ## pain

        #('g2592_gait', ['a.1_lankpffom4'], []),
        #('g2592_gait', ['a.1_lrankpffom4'], []),

        #('g2592_gait', ['a_.1_obj_lfootcf'], []),

        ## weak

        #('g2592_gait', ['a.1_ankPF.2_efcim5'], []),
        #('g2592_gait', ['a.1_ankLPF.1_efcim5'], []),

        #('g2592_gait', ['a.1_gluts.4_efcim5'], []),
        #('g2592_gait', ['a_.1_mscl_lglut.2'], []),

        #('g2592_gait', ['a.1_ankLDF.1_efcim5'], []),

        ## crouch

        #('g2592_gait', ['a.1_hamst_psoas_tl.8_ankPF_mf.2_efcim5'], []), # not captured

        ## dislocation

        #('g2592lhip_gait', ['a.1_efcim5'], []), # not captured

        # met

        #('g2592_gait', ['a.1'], ['met']),
        #('g2592_soldier', ['a_.1'], ['met']),  # not captured
        #('g2592_lean', ['a_.1'], ['met']),
        #('g2592_srun', ['a_.01'], ['met']),
        #('g2592_frun', ['a_.01'], ['met']),
        ##('g2592_tong', ['a_.01_mscl_2x'], ['met']),  # range_error

        #('g2562_gait', ['a_.1'], ['met']),
        #('g2562_soldier', ['a_.1'], ['met']), 
        #('g2562_lean', ['a_.1'], ['met']),
        #('g2562_srun', ['a_.01'], ['met']),
        #('g2562_frun', ['a_.01'], ['met']),
        #('g2562_tong', ['a_.01_mscl_2x'], ['met']),
        #('g2562_ipfrun', ['a_.01_mscl_2x_5'], ['met']),
        #('g2562_ipnrun', ['a_.01_mscl_2x'], ['met']),

        ##('full_soldier', ['a_.1'], ['met']), 
        #('full_lean', ['default'], ['met']),
        #('full_srun', ['default'], ['met']),
        ##('full_frun', ['a_.01'], ['met']),
        ##('full_tong', ['a_.01_mscl_2x'], ['met']),
        #('full_ipfrun', ['a_.01_mscl_2x'], ['met']),
        ##('full_ipnrun', ['a_.01_mscl_2x'], ['met']),

        ]

for usecase_pair in usecase_pairs:
    usecase = usecase_pair[0]
    luacapmode = pyobj2luatablestr(usecase_pair[2])

    for mode in usecase_pair[1]:
        savename = '%s__%s'%(usecase, mode)
        luacode =\
        'print(\'%s\');prefix=\'%s\';g_mode=\'%s\';require(\'useMuscle_%s\');useCase=useCases.%s;g_capmode=%s'%\
                    (savename, savename, mode, usecase, usecase, luacapmode)

        os.system('%s \"%s\" \"%s\"'%(ysbatch, capturescript, luacode))
        #print '%s \"%s\" \"%s\"'%(ysbatch, capturescript, luacode)
