###########################################################
#   Regression suite for mapping simulator in Cmpexplore
###########################################################

How to run regression:

1. Run in one of the three modes:
   - './runreg' (no parameters) runs all tests
   - './runreg <list_of_test_names>' runs only specified tests
   - './runreg -s <list_of_test_names>' runs all tests, but skips the specified ones

2. The status of every test will be printed out (OK/FAILED).
   To debug a particular test go to the test directory and
   check file 'diff.txt' to see the reason of failure.


How to add a new test:

1. Create a new directory named 'test_<testname>'.
2. Add file 'run.cmd': this file contains the command line
   as if run from the cmpexplore root directory.
3. Add file 'filters.txt': this file contains keyphrases
   for the lines to be filtered out of the output file
   (one filter per line). Those will be passed to "grep -v"
   to post-process the test output.
4. Add file 'expected.txt': this file contains the expected
   test output (already filtered).
   This will be passed to "diff" with the test output
   to determine the test status (OK/FAILED).
5. (Optional)
   Add file 'descr.txt' with a brief description of the test.

As an example, see test_4x4_64t_ideal.

