"""Make the package importable for tests run directly from the workspace."""

import os
import sys


sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))
