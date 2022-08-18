import os
import re
import subprocess
import shutil

exeFileName = "phase_d.exe"
webotsAppLocation = "\"C:\\Program Files\\Webots\\msys64\\mingw64\\bin\webots.exe\""
worldFileName = "z1234567_MTRN4110_PhaseA.wbt"
RED_TEXT = '\033[91m'
END_TEXT_FMT = '\033[0m'


webots = subprocess.Popen(f"{webotsAppLocation} --mode=fast --stdout --stderr ./worlds/{worldFileName}", 
                            stdout = subprocess.PIPE, 
                            stderr = subprocess.STDOUT,
                            encoding='utf8')

hasControllerFinished = False
while not hasControllerFinished:
    if webots.stdout != None:
        line = webots.stdout.readline()
        if re.match("INFO: '(.)+' controller exited successfully.", line):
            hasControllerFinished = True
        
        if re.match("WARNING: '(.)+' controller exited with status: (\d)+.", line):
            print(f"\t{RED_TEXT}WARNING: Controller crashed!{END_TEXT_FMT}", flush = True)
            hasControllerFinished = True

# webots.kill()

subprocess.run([exeFileName])