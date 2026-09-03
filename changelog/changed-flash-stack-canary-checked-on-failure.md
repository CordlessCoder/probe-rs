The flash algorithm's stack canary is now read only after a call that failed, rather than after
every call. It is written once when the algorithm is loaded and never rewritten, so it explains a
failure just as well afterwards, and reading it cost a probe round trip on every sector erased and
every page programmed.
