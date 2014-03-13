#!/usr/bin/python

import os

for file in os.listdir("./test"):
    if file.endswith(".cmp"):
        os.system("./cmpexplore -a2wa -test ./test/" + file);

