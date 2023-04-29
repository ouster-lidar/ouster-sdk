import re
import os
import sys
import json

commit_re = r'\A[0-9A-Z][\x20-\x7E]{0,88}([a-zA-Z0-9]|[\)])(\n(\n[\20-\x7E]{0,100})+)?(\n)?'
def check_commit_message(msg):

    try:
        match = re.fullmatch(commit_re, msg)
        if match is None:
            print(f"Commit message {msg} does not match regex {commit_re}")
            sys.exit(1)

    except Exception as e:
        return (f'Error trying to compare regex {commit_re} to commit message {msg}: {str(e)}')

commits_string = os.environ.get("COMMITS")
print(commits_string)
commits = json.loads(commits_string)
msgs =  [ commit["message"] for commit in commits ]
for msg in msgs:
    check_commit_message(msg)
