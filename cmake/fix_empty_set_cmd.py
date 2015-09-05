#!/usr/bin/env python
import os
import re

data = open(os.sys.argv[1], 'r').read()
newdata = re.sub(r"^set(.*)$\n\t\)",r"set\1)" , data, flags=re.MULTILINE)

open(os.sys.argv[1], "w").write(newdata)
