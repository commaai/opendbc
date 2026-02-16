def pytest_collection_modifyitems(items):
    """Put slow misra tests first so xdist distributes them to different workers."""
    misra = []
    rest = []
    for item in items:
        if "misra" in item.nodeid:
            misra.append(item)
        else:
            rest.append(item)
    items[:] = misra + rest
