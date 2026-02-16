#!/usr/bin/env python3
"""
Standalone test script for pursuit flight mission.
Run with: python3 test_pursuit.py
"""

import sys
import os

# Add the package to Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'drone_autonomy'))

# Now import and run
from drone_autonomy.missions.pursuit_flight import main

if __name__ == '__main__':
    main()
