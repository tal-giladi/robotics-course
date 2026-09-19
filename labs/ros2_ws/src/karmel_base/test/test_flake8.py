from ament_flake8.main import main_with_errors
import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    # setup.cfg [flake8]: 120 columns. protocol.py is a copy of robotlab's and is linted there.
    rc, errors = main_with_errors(argv=['--config', 'setup.cfg', '--exclude', './karmel_base/protocol.py'])
    assert rc == 0, 'Found %d code style errors / warnings:\n' % len(errors) + '\n'.join(errors)
