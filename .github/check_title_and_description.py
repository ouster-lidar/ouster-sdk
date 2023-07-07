import os
import sys
import re

title_re = r'\A[0-9A-Z][\x20-\x7E]{0,88}([a-zA-Z0-9]|[\)])'
description_re = r'(((\r)?\n[\20-\x7E]{0,100})+)?(\n)?'

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
            "* the first letter is a capital letter or numeral 0-9\n"
            "* title is more than 2 characters long\n"
            "* the title is no longer than 90 characters including "
            "the addition of (###) for the PR number")
    error = True

print(f"Checking PR description {repr(pr_description)}")
if not re.fullmatch(description_re, pr_description):
    print(f"Error: The PR description {repr(pr_description)}, "
            "which will be treated as your commit message, does "
            "not match the regex {description_re}")
    error = True

if error:
    sys.exit(1)
