our goal is to get opendbc to 100% type coverage, checked by pyrefly.
* we want absolutely no hacks
* you should agree with every type hint. our goal is correctness, not making pyrefly happy.
* pyrefly must pass, but not at the expense of correctness.
* it is expected pyrefly itself may have some gaps and bugs.
* in those cases, we should apply carefully considered casts or other remedies with good succinct comments explaining the situation.
* we need to remove all global ignores if they exist.
* we want to run pyrefly in its strictest mode.
