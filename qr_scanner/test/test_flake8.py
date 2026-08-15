# -- Team 2 代码风格检查 — flake8 wrapper --
from ament_flake8.main import main_with_errors
import pytest


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    rc, errors = main_with_errors(argv=[])
    assert rc == 0, \
        f'Found {len(errors)} style violations:\n' + \
        '\n'.join(errors)
