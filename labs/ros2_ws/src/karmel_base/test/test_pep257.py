from ament_pep257.main import main
import pytest


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    # protocol.py is a byte-identical copy of labs/python/robotlab/protocol.py: linted there, not here.
    # D213 contradicts D212 (already ignored by ament); D301 would force r-strings on doctest examples.
    rc = main(argv=['.', 'test', '--add-ignore', 'D213', 'D301', '--exclude', './karmel_base/protocol.py'])
    assert rc == 0, 'Found code style errors / warnings'
