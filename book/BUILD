# HTML sources for tidy and link-check tests.
files(
    name="html",
    sources=["*.html", ".tidy.config"],
)

# Tidy test: run run_tidy.py (lives in book/htmlbook/tools/tidy).
# Link-check test: run check_html_links_exist on all book HTML (in book/htmlbook/tools/html).
target(
    name="book_tests",
    dependencies=[
        "//book/htmlbook/tools/tidy:tidy_test",
        "//book/htmlbook/tools/html:link_check_test",
    ],
)
