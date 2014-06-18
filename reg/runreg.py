#!/usr/bin/python

###########################################################
#   Regression suite for mapping simulator in Cmpexplore
###########################################################

import sys, os

# ---------------------------------------------------------
#   Runs a specific test
# ---------------------------------------------------------

def run_a_test (tname):
  sys.stdout.write(tname + "...")
  sys.stdout.flush()
  
  # get command line
  f = open(tname + "/run.cmd")
  cmd = f.readlines()[0][:-1]
  f.close()
  
  # prepare command line
  # - adjust the local task file name
  task_idx = cmd.find("-tasks ")
  if task_idx != -1:
    task_idx += len("-tasks ")
    cmd = cmd[:task_idx] + "./reg/" + tname + "/" + cmd[task_idx:]
  # - add the output file name
  out = "./reg/" + tname + "/out.txt"
  cmd += " > " + out
  #print '\n' + cmd
  #sys.exit(0)
  
  # run test
  curdir = os.getcwd()
  os.chdir("..")
  os.system(cmd)
  os.chdir(curdir)
  
  # filter output
  ffilt = open(tname + "/filters.txt")
  for filt in ffilt.readlines():
    if filt != "":
      filt = filt[:-1]
      filtcmd = 'grep -v "' + filt + '" ' + tname + "/out.txt > " + tname + "/tmp.txt"
      #print filtcmd
      os.system(filtcmd)
      mvcmd = "mv " + tname + "/tmp.txt " + tname + "/out.txt"
      os.system(mvcmd)
  ffilt.close()
  
  # diff output
  diffcmd = "diff " + tname + "/out.txt " + tname + "/expected.txt > " + tname + "/diff.txt"
  os.system(diffcmd)
  
  # print test status
  if os.path.getsize(tname + "/diff.txt") > 0:
    print '\t\t\t FAILED'
  else :
    print '\t\t\t OK'
  

# ---------------------------------------------------------
#   Main procedure: Run all tests
# ---------------------------------------------------------

def run_all_tests ():
  print " *** CMPexplore mapping simulator regression *** "
  
  if len(sys.argv) > 1:
    for fname in sys.argv[1:]:
      run_a_test(fname)
  else:
    for fname in os.listdir("./"):
      if os.path.isdir(fname) and fname.startswith("test_"):
        run_a_test(fname)


# ---------------------------------------------------------
#   Main stub
# ---------------------------------------------------------

if __name__ == '__main__':
  run_all_tests()

