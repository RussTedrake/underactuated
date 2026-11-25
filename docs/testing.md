# Remote packages in tests

Local `pytest` runs can download Drake remote packages on demand. The root
`conftest.py` defaults `XDG_CACHE_HOME` to `/tmp`, so Drake caches them under
`/tmp/drake`.

Each CI test job runs `python -m setup.prefetch_remotes` before pytest, using
the same Python installation, user, and `XDG_CACHE_HOME=/tmp` as the tests.
The prefetch retries failed downloads (see `book/htmlbook/drake_remotes.py`).
The pip job prefetches using the installed underactuated wheel, matching its
tests. During pytest, `DRAKE_ALLOW_NETWORK=lcm:meshcat` disables PackageMap
downloads while allowing Meshcat and LCM. Notebook subprocesses inherit these
settings. An uncached remote package therefore fails instead of downloading
in the middle of a test. This restricts Drake networking; it is not a general
network sandbox for Python or external programs.

To reproduce this policy locally, run from the repository root:

```sh
XDG_CACHE_HOME=/tmp .venv/bin/python -m setup.prefetch_remotes
XDG_CACHE_HOME=/tmp DRAKE_ALLOW_NETWORK=lcm:meshcat .venv/bin/python -m pytest -ra
```

If a test reports that PackageMap networking is disabled and no matching cache
entry exists, check that prefetch and pytest use the same user and cache
directory. If a test adds a new remote package, add it to
`setup/prefetch_remotes.py`.

See Drake's [network policy documentation](https://drake.mit.edu/doxygen_cxx/group__allow__network.html).

# Notebook coverage

`book/test_notebook_coverage.py` fails when a tracked notebook is not
registered with `ipynb_test()` in a `test_*.py` file in its directory. Notebooks
that cannot run in CI go in its `UNTESTED` mapping with a reason. Exercise
notebooks are covered by tests in the solutions checkout; that check is skipped
when `solutions/` is absent.

# Solutions in CI

When the `SOLUTIONS_TOKEN` repository secret is set, each test job clones
`RobotLocomotion/underactuated-solutions` at the revision in
`solutions_sha.txt`, and pytest runs its tests too. Without the secret, the
jobs skip the solutions tests. Fork pull requests do not receive secrets until
a maintainer applies the `approved-for-ci` label, which runs the workflow
through `pull_request_target`.

# CI timings

Each test job prints its 30 slowest pytest phases and uploads a JUnit report as
`pytest-<job-id>`, retained for 14 days and uploaded even on test failure when
available.

CI uses two pytest-xdist workers with `--dist loadfile`, so tests in the same
file stay on one worker. Remote packages are prefetched before workers start,
and native numerical-library thread counts are limited to one to avoid
oversubscribing the runner. Local pytest remains serial by default. To try the
CI configuration:

```sh
OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1 \
  .venv/bin/python -m pytest -n 2 --dist loadfile -ra --durations=30
```

Ubuntu Poetry CI sets `INSTALL_JUPYTER=0` when installing system prerequisites
because its Python environment supplies the notebook dependencies. The
prerequisite script still installs system Jupyter by default for local users
and the pip job.
