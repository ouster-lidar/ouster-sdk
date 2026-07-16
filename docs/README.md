## Build

Build docs builds performs installs for docs build, builds sdk-extensions docs, builds ouster-sdk (for API refernce).

```bash
python3.10 -m venv venv310
. venv310/bin/activate
python scripts/dev.py build docs
```

Quick run:

- Skips python api build.
- Skips cpp api build.
- Skips python build needed for python api.

```
./scripts/dev.sh build docs --no-cpp-api --no-python-api --no-python-build
```


To run directly via sphinx installation if you have the environment.
This can be useful for iterating quicker.

```bash
python3 -m sphinx -b html docs docs/_build/html-press
```

## Test

Code used in documentation is at

1. Python code + tests: features/*/_snippets/python/
2. C++ code + tests: features/*/_snippets/cpp/
3. CLI command code + tests: `python/tests/documentation/test_cli_commands.py`

There is additional (pre-existing) example code that feeds into the documentation. These are at:

1. C++: `tests/`
2. Python: `python/src/ouster/sdk/examples/`


All documentation testing can be performed via dev scripts in one go using.

```bash
python3 scripts/dev.py test docs --no-cpp-api --no-python-api
```

or Use help for more options.

```bash
python3 scripts/dev.py test docs --help
```

TODO:

Add interactive tests for viz code in below
file:///Users/akatumalla/src/ouster/team-sdk/docs/ouster-sdk/docs/_build/latest/html/features/mapping/viz-scans-accum.html
