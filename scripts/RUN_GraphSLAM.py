#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Sat Sept 28 16:19:52 2020

@author: sc

"""

if __name__ == '__main__' and __package__ is None:
    from os import sys
    sys.path.append('../')
    # sys.path.append('../python/GCN')
import multiprocessing as mp
import subprocess, os, sys, time
from pathlib import Path
from tqdm import tqdm
import shlex
import argparse

# from utils import util_parser
# par_model = util_parser.Parse3RScanModel()
# par_label = util_parser.ParseLabelMapping()
parser = argparse.ArgumentParser(description='This script runs GraphSLAM on the target sequences provided with the --txt. ',formatter_class=argparse.ArgumentDefaultsHelpFormatter)
parser.add_argument('--scans', type=str, 
                        default='/media/sc/SSD1TB/dataset/3RScan/data/3RScan/', help='to the directory contains scans',required=False)
parser.add_argument('--pth_out', type=str, default='./result_reconstruction/', help='where to save the prediction',required=False)
parser.add_argument('--pth_model', type=str, default='', help='where to save the prediction',required=True)
parser.add_argument('--txt', type=str, 
                    default='/home/sc/research/PersistentSLAM/python/3DSSG/data/ScanNet20_InSeg_Full/validation_scans.txt', 
                    help='a txt file contains a set of scene id.')
parser.add_argument('--pred_script', type=str, default='../bin/exe_GraphSLAM', help='prediction script')  
parser.add_argument('--fusion', type=int, default=1, help='Fusion or not.')
parser.add_argument('--thread', type=int, default=1, help='The number of threads to be used.')
parser.add_argument('--thread_slam', type=int, default=1, help='The number of threads to be used.')
parser.add_argument('--debug', type=int, default=0, help='')
parser.add_argument('--on_failed', type=int, default=0, help='run but skip successed.')
args = parser.parse_args()



def process(pth_in, pth_out, rendered):
    startTime = time.time()
    try:
        print('Processing file', pth_in)
        process = subprocess.Popen(
            [args.pred_script, 
            '--pth_in',pth_in, 
            '--pth_out',pth_out,
            '--pth_model',args.pth_model,
            '--fusion',str(args.fusion),
            '--save',str(1),
            '--binary',str(1),
            '--thread',str(args.thread_slam),
            '--rendered',str(rendered),
            '--verbose',str(0)
            ], stdout=subprocess.PIPE)
        while True:
            output = process.stdout.readline()
            if process.poll() is not None:
                break
            # if output == '' and process.poll() is not None:
            #     break
            if output:
                print(output.strip())
        output = process.poll()
        # return rc
        # output = subprocess.check_output([args.pred_script, 
        #                                   '--pth_in',pth_in, 
        #                                   '--pth_out',pth_out,
        #                                   '--pth_model',args.pth_model,
        #                                   '--fusion',str(args.fusion),
        #                                   '--save',str(1),
        #                                   '--binary',str(1),
        #                                   '--thread',str(0),
        #                                   '--rendered',str(rendered)
        #              ],
        #     stderr=subprocess.STDOUT)
        # sys.stdout.write(output.decode('utf-8'))
        good = True
    except subprocess.CalledProcessError as e:
        print('[Catched Error]', "command '{}' return with error (code {}): {}".format(e.cmd, e.returncode, e.output.decode('utf-8'))) # omit errors
        good = False
    endTime = time.time()
    print('Processing file', os.path.splitext(os.path.basename(pth_in))[0], 'finished')
    return [pth_in, endTime-startTime, good ]

def get_file_in_folder(path):
    files =[]
    for (dirpath, dirnames, filenames) in os.walk(path):
        for filename in filenames:
            if filename == args.plyname:
                files.append( os.path.abspath( os.path.join(dirpath, filename)) )
        # break   
    return files
def read_lines_from_file(filepath):
    with open(filepath) as f:
        content = f.readlines()
    # you may also want to remove whitespace characters like `\n` at the end of each line
    return [x.strip() for x in content] 

if __name__ == '__main__':
    if args.thread > 1:
        pool = mp.Pool(args.thread)
        pool.daemon = True
    
    ''' Read split '''
    print('read split file..')
    ids = open(args.txt).read().splitlines()
    print('there are',len(ids),'sequences')
    
    # check output folder exist
    Path(args.pth_out).mkdir(parents=True, exist_ok=True)
    
    results=[]
    counter=0
    # should_skip=True
    
    scene_to_process = []
    # scene_to_process = ['4fbad314-465b-2a5d-8445-9d021f278c1e', '20c993af-698f-29c5-84b2-972451f94cfb']
    only_on_failed = args.on_failed>0
    for scan_id in tqdm(sorted(ids)):
        counter+=1
        
        # if counter < 236: continue
        if len(scene_to_process) > 0:
            if scan_id not in scene_to_process:
                continue
            
        if only_on_failed:
            if len(os.listdir(args.pth_out+'/'+scan_id))==3:
                continue
        #     print('false now')
        #     should_skip =False
        # if should_skip: continue
        
        
        # if counter < 35: continue
        if scan_id.find('scene') >=0:    
            # ScanNet
            pth_in = os.path.join(args.scans, scan_id, scan_id+'.sens')
            pth_out  = os.path.join(args.pth_out, scan_id)
            rendered = 1
        else:
            # 3RScan
            pth_in  = os.path.join(args.scans,scan_id,'sequence')
            pth_out = os.path.join(args.pth_out, scan_id)
            rendered = 0
        
        
        if args.thread > 1:
            results.append(
                pool.apply_async(process,(pth_in,pth_out,rendered)))
        else:
            results.append(process(pth_in,pth_out,rendered))
        
        if args.debug>0:
            break
        
    
    if args.thread > 1:
        pool.close()
        pool.join()
    if args.thread > 1:
        results = [r.get() for r in results]
        
    
    for r in results:
        if r[2] is False:
            print('the scans that failed:')
            print(r[0])
