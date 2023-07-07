import os
import sys
import ouster.cli.core  # noqa: F401


def test_find_plugins(capsys):
    import ouster.cli.plugins
    test_plugin_path = os.path.join(os.path.dirname(__file__), 'ouster', 'cli', 'plugins')
    ouster.cli.plugins.__path__.append(test_plugin_path)
    result = ouster.cli.core.find_plugins(show_traceback=False)
    plugin_names = [plugin.name for plugin in result]
    assert set([
        'ouster.cli.plugins.discover',
        'ouster.cli.plugins.io_type',
        'ouster.cli.plugins.source',
        'ouster.cli.plugins.source_mapping',
        'ouster.cli.plugins.source_osf',
        'ouster.cli.plugins.testing',
        'ouster.cli.plugins.bad_plugin',
    ]).issubset(plugin_names)
    captured = capsys.readouterr()
    assert "Traceback" not in captured.err
    assert f"Run {os.path.basename(sys.argv[0])} {ouster.cli.core.TRACEBACK_FLAG} for debug output" in captured.err
    assert not captured.out
    assert "Failed to load plugin ouster.cli.plugins.bad_plugin due to an error: Whoopsie!" in captured.err

    result = ouster.cli.core.find_plugins(show_traceback=True)
    captured = capsys.readouterr()
    assert "Traceback" in captured.err
    assert not captured.out
    assert "for debug output" not in captured.err
    assert "Failed to load plugin ouster.cli.plugins.bad_plugin due to an error: Whoopsie!" in captured.err
