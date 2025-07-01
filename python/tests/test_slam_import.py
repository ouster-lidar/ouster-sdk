import pytest
import platform

# Check if the platform is macOS and if it's running on ARM architecture
is_mac_arm = platform.system() == 'Darwin' and platform.processor() == 'arm'


@pytest.fixture
@pytest.mark.skipif(is_mac_arm, reason="Skipping tests on MacBook with ARM architecture")
def test_slam_import() -> None:
    """
    Test importing SlamConfig and SlamEngine from ouster.sdk.mapping
    to ensure the import works without error.
    """
    from ouster.sdk.mapping import SlamConfig, SlamEngine  # noqa
    assert (True)
