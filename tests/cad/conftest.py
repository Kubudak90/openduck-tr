# -*- coding: utf-8 -*-
import os
import sys

CAD = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "cad")
if CAD not in sys.path:
    sys.path.insert(0, CAD)


def pytest_configure(config):
    config.addinivalue_line("markers", "slow: FreeCAD derlemesi gerektiren uzun test (-m 'not slow' ile atlanir)")
