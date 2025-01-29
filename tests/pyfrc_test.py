'''
    This test module imports tests that come with pyfrc, and can be used
    to test basic functionality of just about any robot.
'''

from pyfrc.tests import test_disabled

import os
import shutil


# Cleanup all the old logs so they don't get deployed to the robot.
def cleanup(dir):
    if os.path.isdir(dir):
        shutil.rmtree(dir)

cleanup("logs")
cleanup("../logs")
cleanup("../ctre_sim/logs")