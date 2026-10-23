# -*- coding: utf-8 -*-
def pytest_configure(config):
    config.addinivalue_line("markers", "slow: FreeCAD derlemesi gerektiren uzun test (-m 'not slow' ile atlanir)")
