import os
import sys
import re

title_re = r'\A[A-Z]'

title = os.environ.get("TITLE")
pr_number_string = os.environ.get("PR_NUMBER")
pr_description = "\n" + os.environ.get("PR_BODY")

# check title length with (PR number) added at end
new_title = title + " (#" + pr_number_string + ")"

error = False
print(f"Checking PR title {repr(new_title)}...")
if not re.fullmatch(title_re, new_title):
    print(f"Error: Please revise the PR title {new_title} "
            "to match the regex {title_re} where:\n"
            "* the first letter is a capital letter")
    error = True

if error:
    sys.exit(1)
