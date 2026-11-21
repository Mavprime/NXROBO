#!/usr/bin/env python3
"""
Test runner for Spark Autonomous Robot Logic & Systems.
Executes all unit tests with formatted test matrix output.
"""
import unittest
import sys
import os
import time

def main():
    test_dir = os.path.dirname(os.path.abspath(__file__))
    loader = unittest.TestLoader()
    suite = loader.discover(start_dir=test_dir, pattern="test_*.py")

    print("=" * 70)
    print(" 🚀 SPARK AUTONOMOUS ROBOT: SYSTEM UNIT & LOGIC TEST SUITE")
    print(f" Directory: {test_dir}")
    print(f" Python:    {sys.version.split()[0]}")
    print("=" * 70)

    start_time = time.time()
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    elapsed = time.time() - start_time

    print("\n" + "=" * 70)
    print(" 📊 TEST SUMMARY MATRIX:")
    print(f" - Total Tests Executed: {result.testsRun}")
    print(f" - Successful Passes:    {result.testsRun - len(result.failures) - len(result.errors)}")
    print(f" - Failures:             {len(result.failures)}")
    print(f" - Errors:               {len(result.errors)}")
    print(f" - Execution Duration:   {elapsed:.3f} s")
    print("=" * 70)

    if result.wasSuccessful():
        print(" ✅ ALL SYSTEM LOGIC TESTS PASSED SUCCESSFULLY!")
        return 0
    else:
        print(" ❌ SOME TESTS FAILED. PLEASE REVIEW ABOVE LOGS.")
        return 1

if __name__ == "__main__":
    sys.exit(main())
