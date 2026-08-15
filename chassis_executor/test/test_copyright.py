# -- Team 2 版权头检查 — ament_copyright linter --
from ament_copyright.main import main
import pytest


@pytest.mark.skip(reason='Auto-generated — copyright header pending.')
@pytest.mark.copyright
@pytest.mark.linter
def test_copyright():
    rc = main(argv=['.', 'test'])
    assert rc == 0, 'Copyright lint failed'
