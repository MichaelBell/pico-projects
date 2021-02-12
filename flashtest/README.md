# Test for aborting and immediately restarting a flash transfer

This test should simply print "Hello, world!" and then loop forever
streaming data from the flash.

However, it appears the first word is lost on the third transfer if you
repeatedly start and abort streaming transfers from the flash.

Adding a 1 microsecond sleep after the abort fixes the test.
