# CSAIL hosting

The live checkout is `/var/www/underactuated` on
`underactuated-r1.csail.mit.edu`. Apache serves its `book` directory at
`https://underactuated.csail.mit.edu/` and enables CGI execution.

SSH first to `login.csail.mit.edu` and complete interactive two-factor
authentication, then use `ssh -o ProxyJump=none underactuated-r1` for the
internal hop. The owner has sudo access; deployments require an interactive
sudo password. Files are managed by `www-data`. Preserve untracked historical
course directories when updating the checkout.

## Bibliography endpoint

`book/htmlbook/elib.cgi` is shared with the manipulation repository. It is
served directly at `https://underactuated.csail.mit.edu/htmlbook/elib.cgi` and
re-executes with this repository's `.venv/bin/python`. Install MySQL
Connector/Python there (the server deployment uses 9.4.0). The existing `venv`
is separate and should not be modified for this endpoint.

The CGI reuses `/etc/elib.json` on the VM, owned by `root:www-data` with mode
640. That file contains the read-only database credentials; keep credentials
out of the repository and document root. The VM can reach CSAIL MySQL.

The metadata installer reads `elib_url` from `book/chapters.json` and POSTs
an array of citation tags. The endpoint returns `entries` keyed by tag and a
`missing` array. It uses parameterized queries, excludes private paper URLs,
and limits requests to 1000 tags and 128 KiB. Missing records and request
failures stop the installer before it changes any HTML.

Update the htmlbook submodule reference when adopting shared code changes.
CGI source updates do not require an Apache restart.
