import os
import sys

# Let `pytest test/` work from a source checkout without installing.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

FIXTURES = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'fixtures')
DESCRIPTION = os.path.join(FIXTURES, 'test_robot_description')
