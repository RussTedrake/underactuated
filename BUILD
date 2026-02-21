# Root BUILD: third-party Python deps from Poetry (pyproject.toml).
# Pants reads [tool.poetry.dependencies] and dev deps; lockfile via pants generate-lockfiles.
poetry_requirements(
    name="reqs",
    source="pyproject.toml",
    module_mapping={
        "beautifulsoup4": ["bs4"],
        "gradescope-utils": ["gradescope_utils"],
        "mysql-connector-python": ["mysql.connector"],
        "timeout-decorator": ["timeout_decorator"],
    },
)

target(
    name="all_python_tests",
    dependencies=[
        "underactuated/...",
        "book/htmlbook:htmlbook_tests",
        "book/figures:figures_tests",
        "book/figures/exercises:exercises_tests",
        "book:book_tests",
    ],
)
