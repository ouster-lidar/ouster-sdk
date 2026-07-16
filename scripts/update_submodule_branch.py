#!/usr/bin/env python3
"""
Update .gitmodules file with the appropriate sdk-extensions branch.
This script is intended to be run in CI environments.
"""

import os
import re
import logging
import git
from pathlib import Path

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(levelname)s: %(message)s'
)
logger = logging.getLogger(__name__)


def get_submodule_name(repo):
    if not repo.submodules:
        raise ValueError("No submodules found in .gitmodules")
    return repo.submodules[0].name


def get_submodule_url(repo, submodule_name):
    parent_url = repo.remotes.origin.url
    absolute_url = re.sub(r"[^/]+\.git$", f"{submodule_name}.git", parent_url)
    return absolute_url


def branch_exists(remote_url, branch_name):
    """Check if a branch exists in the remote repository."""
    logger.debug(f"Checking if branch '{branch_name}' exists in remote: {remote_url}")
    try:
        # Use full ref to avoid ambiguity; a short name matches any ref whose path ends with branch_name.
        result = git.cmd.Git().ls_remote(
            "--heads", remote_url, f"refs/heads/{branch_name}"
        )
        exists = bool(result.strip())
        if exists:
            logger.info(f"Branch '{branch_name}' found in remote")
        else:
            logger.info(f"Branch '{branch_name}' not found in remote")
        return exists
    except git.exc.GitCommandError as e:
        logger.warning(f"Error checking branch existence: {e}")
        raise


def get_gitmodules_branch(repo, submodule_name):
    """Return the branch set for the submodule in .gitmodules"""
    result = repo.git.config("--file", ".gitmodules", "--get",
                             f"submodule.{submodule_name}.branch")
    return result.strip() if result else "develop"


def update_gitmodules_branch(repo, submodule_name, branch_name):
    """Update the branch in .gitmodules using git submodule set-branch."""
    logger.info(f"Updating .gitmodules with branch: {branch_name}")
    try:
        repo.git.submodule("set-branch", "--branch", branch_name,
                           submodule_name)
    except git.exc.GitCommandError as e:
        logger.error(f"Failed to set submodule branch via git: {e}")
        raise


def main():
    """Main function to update submodule branch."""
    logger.info("Starting submodule branch update")

    repo = git.Repo(".", search_parent_directories=True)
    # Get branch name from environment
    is_mr_build = os.environ.get("CHANGE_ID") is not None
    if is_mr_build:
        branch_name = os.environ.get("CHANGE_BRANCH")
    else:
        branch_name = os.environ.get("BRANCH_NAME") or os.environ.get("BRANCH")

    gitmodules_path = Path(repo.working_dir) / ".gitmodules"
    if not gitmodules_path.exists():
        raise RuntimeError(f"Error: .gitmodules not found at {gitmodules_path}")

    # Get submodule name and URL
    submodule_name = get_submodule_name(repo)
    submodule_url = get_submodule_url(repo, submodule_name)

    # Update .gitmodules only if needed
    current_gitmodules_branch = get_gitmodules_branch(repo, submodule_name)
    if current_gitmodules_branch != "develop":
        logger.info(f"Retain .gitmodules branch already set: "
                    f"{current_gitmodules_branch}")
    else:
        # Check if branch exists
        if not branch_exists(submodule_url, branch_name):
            logger.warning(f"Branch '{branch_name}' not found in {submodule_name},"
                           " defaulting to 'develop'")
            branch_name = "develop"
        update_gitmodules_branch(repo, submodule_name, branch_name)

    logger.info(f"Final {submodule_name} branch: {branch_name}")
    return branch_name


if __name__ == "__main__":
    main()
