our goal is to write a mutation tester that is super fast
* it will run on all code, not just the git diff like the previous version
* it must finish is <1min on this machine
* it cannot rely on hacks like caching
* it must not introduce any new dependencies
* all surviving mutants must be true positives.
* once all the goals are met, red team the tool to make sure all killed and surviving mutants are correct
* do not stop until all objectives are met!

note that we expect some surviving mutants at this point.
your goal is not to fix those at all, but only to make the test run fast and correctly identify them.
