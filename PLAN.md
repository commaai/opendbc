mutation.py is in good shape UX and otherwise now. the only problem is that we have a hardcoded list of tests to run, which makes it hard to add tests later and it's just ugly.

your goal is to replace the hardcoded list of tests with some kind of automatic test ordering, e.g. if we mutate a toyota file, run the toyota test suite first. we may need

the key is that we can't regress the speed of execution much. mutation.py's total runtime absolutely cannot exceed 40s.

remember to not write hacks. we only want to make the code more beautiful and generic than it was.
an example of a hack is some kind of test order caching that relies on a slow pass, then future runs benefit from that. it is certainly possible to write a simple function to order the tests such that one of the first few tgests we run kills the mutant.
