import os
import sys
import re

title_re = r'\A[0-9A-Z][\x20-\x7E]{0,88}'

num_commits = int(os.environ.get("NUM_COMMITS"))
title = os.environ.get("TITLE")
pr_number_string = os.environ.get("PR_NUMBER")

# complain if not squashed
if num_commits != 1:
    print(f"Error: Please squash your {num_commits} commits to 1 commit to lessen possible commit message errors introduced through the the merge UI. Thank you!")
    sys.exit(1)

# check title length with (PR number) added at end
new_title = title + "(" + pr_number_string + ")"
if not re.fullmatch(title_re, title[0]):
    print(f"Error: Please revise your PR title {title} to match the regex {title_re}")
    sys.exit(1)
