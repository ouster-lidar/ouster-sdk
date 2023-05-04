import os
import sys
import re

title_re = r'\A[0-9A-Z][\x20-\x7E]{0,88}([a-zA-Z0-9]|[\)])'
description_re = r'((\n[\20-\x7E]{0,100})+)?(\n)?'

num_commits = int(os.environ.get("NUM_COMMITS"))
title = os.environ.get("TITLE")
pr_number_string = os.environ.get("PR_NUMBER")
pr_description = "\n" + os.environ.get("PR_BODY")

# check title length with (PR number) added at end
new_title = title + " (#" + pr_number_string + ")"
print("new_title:", new_title)
print("pr_number_string:", pr_number_string)
print("pr_description", pr_description)
print("num_commits", num_commits)

error = False
if not re.fullmatch(title_re, new_title):
    print(f"Error: Please revise your PR title {new_title} "
            "to match the regex {title_re} where:\n"
            "* the first letter is a capital letter or numeral 0-9\n"
            "* title is more than 2 characters long\n"
            "* the title is no longer than 90 characters including "
            "the addition of (###) for the PR number")
    error = True

if not re.fullmatch(description_re, pr_description):
    print(f"Error: Please revise your PR description {pr_description} to match the regex {description_re}")
    error = True

# No need to check two new lines since it is determined by GH Merge settings
# commit_re = title_re + body_re
# if not re.fullmatch(commit_re, commit):


if error:
    sys.exit(1)
