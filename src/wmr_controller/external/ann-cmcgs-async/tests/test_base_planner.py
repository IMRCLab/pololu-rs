#!/usr/bin/env python3
"""
Test importing the base Planner class directly
"""

import sys
import os

# Add the project root to the path
sys.path.insert(0, '/home/christoph/Dokumente/research/projects/dec-mcgs')

try:
    print("Attempting to import base Planner class...")
    from tools.planners.planner import Planner
    print("✅ Successfully imported Planner class")
    print(f"Planner class: {Planner}")
    print(f"Planner methods: {[method for method in dir(Planner) if not method.startswith('_')]}")
    
    # Check if it's abstract
    import inspect
    print(f"Is abstract: {inspect.isabstract(Planner)}")
    
except ImportError as e:
    print(f"❌ Failed to import Planner: {e}")
    import traceback
    traceback.print_exc()