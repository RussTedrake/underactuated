import json
import re
import requests
from requests.adapters import HTTPAdapter
import time
import traceback

try:
    from urllib import urlencode  # type: ignore[attr-defined]
except:
    from urllib.parse import urlencode


class Hypothesis:
    def __init__(
        self,
        domain=None,
        authority=None,
        username=None,
        token=None,
        group=None,
        limit=None,
        max_search_results=None,
        host=None,
        port=None,
        debug=False,
    ):

        if domain is None:
            self.domain = "hypothes.is"
        else:
            self.domain = domain

        if authority is None:
            self.authority = "hypothes.is"
        else:
            self.authority = authority

        self.app_url = "https://%s/app" % self.domain
        self.api_url = "https://%s/api" % self.domain
        self.query_url = "https://%s/api/search?{query}" % self.domain
        self.anno_url = "https://%s/a" % domain
        self.via_url = "https://via.hypothes.is"
        self.token = token
        self.username = username
        self.single_page_limit = (
            200 if limit is None else limit
        )  # per-page, the api honors limit= up to (currently) 200
        self.max_search_results = (
            2000 if max_search_results is None else max_search_results
        )  # limit for paginated results
        self.group = group if group is not None else "__world__"
        if self.username is not None:
            self.permissions = {
                "read": ["group:" + self.group],
                "update": ["acct:" + self.username + "@" + self.authority],
                "delete": ["acct:" + self.username + "@" + self.authority],
                "admin": ["acct:" + self.username + "@" + self.authority],
            }
        else:
            self.permissions = {}

        self.session = requests.Session()
        self.session.mount(self.api_url, HTTPAdapter(max_retries=3))

        self.debug = debug

    # rps taken from https://github.com/hypothesis/h/blob/e0792a5607da368623fbc68a5f2a32eba56e1d56/conf/nginx.conf#L127
    def search_all(self, params={}, stop_at=None, rps=40):
        """Call search API with pagination, return rows """
        sort_by = params["sort"] if "sort" in params else "updated"
        if stop_at:
            if not isinstance(stop_at, str):
                raise TypeError("stop_at should be a string")

            dont_stop = (
                lambda r: r[sort_by]
                <= stop_at  # when ascending things less than stop are ok
                if "order" in params and params["order"] == "asc"
                else lambda r: r[sort_by] >= stop_at
            )

        limit = 200 if "limit" not in params else params["limit"]  # FIXME hardcoded
        if "max_results" in params:
            max_results = params["max_results"]
        else:
            max_results = self.max_search_results
        if max_results < limit:
            params["limit"] = max_results

        # sup_inf = max if params['order'] = 'asc' else min  # api defaults to desc
        # trust that rows[-1] works rather than potentially messsing stuff if max/min work differently
        nresults = 0
        while True:
            obj = self.search(params)
            rows = obj["rows"]
            lr = len(rows)
            nresults += lr
            if lr is 0:
                return

            stop = None

            if nresults >= max_results:
                stop = max_results - nresults + lr

            if stop_at:
                for row in rows[:stop]:
                    if dont_stop(row):
                        yield row
                    else:
                        return
            else:
                for row in rows[:stop]:
                    yield row

            if stop:
                return

            search_after = rows[-1][sort_by]
            params["search_after"] = search_after

            if self.debug:
                print("searching after", search_after)

            time.sleep(1.0 / rps)

    def search(self, params={}):
        """ Call search API, return a dict """
        if "offset" not in params:
            params["offset"] = 0
        if "limit" not in params or "limit" in params and params["limit"] is None:
            params["limit"] = self.single_page_limit
        query_url = self.query_url.format(query=urlencode(params, True))
        obj = self.authenticated_api_query(query_url)
        return obj

    def authenticated_api_query(self, query_url=None):
        headers = {
            "Content-Type": "application/json;charset=utf-8",
        }
        if self.token:
            headers["Authorization"] = "Bearer " + self.token
        r = requests.get(query_url, headers=headers)
        obj = r.json()
        if r.ok:
            return obj
        else:
            raise Exception( "reason %s data %s" % (r.reason, obj) )

    def get_annotation(self, id=None):
        h_url = "%s/annotations/%s" % (self.api_url, id)
        if self.token is not None:
            obj = self.authenticated_api_query(h_url)
        else:
            obj = requests.get(h_url)
        return obj

    def post_annotation(self, payload):
        headers = {
            "Authorization": "Bearer " + self.token,
            "Content-Type": "application/json;charset=utf-8",
        }
        data = json.dumps(payload, ensure_ascii=False)
        r = requests.post(
            self.api_url + "/annotations",
            headers=headers,
            data=data.encode("utf-8"),
        )
        return r

    def update_annotation(self, id, payload):
        headers = {
            "Authorization": "Bearer " + self.token,
            "Content-Type": "application/json;charset=utf-8",
        }
        data = json.dumps(payload, ensure_ascii=False)
        r = requests.put(
            self.api_url + "/annotations/" + id, headers=headers, data=data
        )
        return r

    def delete_annotation(self, id):
        headers = {
            "Authorization": "Bearer " + self.token,
            "Content-Type": "application/json;charset=utf-8",
        }
        r = requests.delete(self.api_url + "/annotations/" + id, headers=headers)
        return r

class HypothesisAnnotation:
    """Encapsulate one row of a Hypothesis API search."""

    def __init__(self, row):
        self.type = None
        self.id = row["id"]
        self.group = row["group"]
        self.updated = row["updated"][0:19]
        self.permissions = row["permissions"]
        self.user = row["user"].replace("acct:", "")
        self.is_group = self.group not in [None, "__world__", "NoGroup"]
        self.is_world_private = (
            not self.is_group and "group:__world__" not in self.permissions["read"]
        )
        self.is_group_private = (
            self.is_group and "group:" + self.group not in self.permissions["read"]
        )
        self.is_public = (
            not self.is_group
            and not self.is_group_private
            and not self.is_world_private
        )

        if "uri" in row:  # is it ever not?
            self.uri = row["uri"]
        else:
            self.uri = "no uri field for %s" % self.id
        self.uri = self.uri.replace("https://via.hypothes.is/h/", "").replace(
            "https://via.hypothes.is/", ""
        )

        if self.uri.startswith("urn:x-pdf") and "document" in row:
            if "link" in row["document"]:
                self.links = row["document"]["link"]
                for link in self.links:
                    self.uri = link["href"]
                    if self.uri.startswith("urn:") == False:
                        break
            if self.uri.startswith("urn:") and "filename" in row["document"]:
                self.uri = row["document"]["filename"]

        if "document" in row and "title" in row["document"]:
            t = row["document"]["title"]
            if isinstance(t, list) and len(t):
                self.doc_title = t[0]
            else:
                self.doc_title = t
        else:
            self.doc_title = None
        if self.doc_title is None:
            self.doc_title = ""
        self.doc_title = self.doc_title.replace('"', "'")
        if self.doc_title == "":
            self.doc_title = "untitled"

        self.tags = []
        if "tags" in row and row["tags"] is not None:
            self.tags = row["tags"]
            if isinstance(self.tags, list):
                self.tags = [t.strip() for t in self.tags]

        self.text = ""
        if "text" in row:
            self.text = row["text"]

        self.references = []
        if "references" in row:
            self.type = "reply"
            self.references = row["references"]

        self.target = []
        if "target" in row:
            self.target = row["target"]

        self.is_page_note = False
        try:
            if (
                self.references == []
                and self.target is not None
                and len(self.target)
                and isinstance(self.target, list)
                and not "selector" in self.target[0]
            ):
                self.is_page_note = True
                self.type = "pagenote"
        except:
            traceback.print_exc()
        if "document" in row and "link" in row["document"]:
            self.links = row["document"]["link"]
            if not isinstance(self.links, list):
                self.links = [{"href": self.links}]
        else:
            self.links = []

        self.start = self.end = self.prefix = self.exact = self.suffix = None
        try:
            if (
                isinstance(self.target, list)
                and len(self.target)
                and "selector" in self.target[0]
            ):
                self.type = "annotation"
                selectors = self.target[0]["selector"]
                for selector in selectors:
                    if "type" in selector and selector["type"] == "TextQuoteSelector":
                        try:
                            self.prefix = selector["prefix"]
                            self.exact = selector["exact"]
                            self.suffix = selector["suffix"]
                        except:
                            traceback.print_exc()
                    if (
                        "type" in selector
                        and selector["type"] == "TextPositionSelector"
                        and "start" in selector
                    ):
                        self.start = selector["start"]
                        self.end = selector["end"]
                    if (
                        "type" in selector
                        and selector["type"] == "FragmentSelector"
                        and "value" in selector
                    ):
                        self.fragment_selector = selector["value"]
        except:
            print(traceback.format_exc())

