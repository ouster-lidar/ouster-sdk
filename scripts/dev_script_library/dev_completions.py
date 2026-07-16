import os
import click
import psutil
import shutil


def update_shell_profile(profile_path, source_line):
    """Checks if source_line exists in profile_path; if not, appends it."""
    profile_path = os.path.expanduser(profile_path)

    # Create the file if it doesn't exist
    if not os.path.exists(profile_path):
        with open(profile_path, "w") as f:
            f.write(f"\n# Added by Ouster Dev Scripts\n{source_line}\n")
        return

    with open(profile_path, "r") as f:
        content = f.read()

    if source_line not in content:
        with open(profile_path, "a") as f:
            # Adding a newline first ensures we don't append to an existing line
            f.write(f"\n# Added by Ouster Dev Scripts\n{source_line}\n")
        click.echo(f"Updated {profile_path} with completion sourcing.")
    else:
        click.echo(f"Completions already sourced in {profile_path}.")


def completion_dir(ctx):
    return os.path.join(ctx.obj.dev_dir, "..", "completions")


def get_parent_shell():
    current_process = psutil.Process(os.getpid())

    parent = current_process.parent()
    mapping = {
        "pwsh": "powershell",
    }
    while parent:
        name = parent.name().lower()
        if any(s in name for s in ['zsh', 'bash', 'fish', 'powershell', 'pwsh']):
            name = name.replace('.exe', '')
            if name in mapping:
                return mapping[name]
            else:
                return name
        parent = parent.parent()
    return "unknown"


def install_zsh_completions(ctx):
    dest = os.path.expanduser("~/.ouster_dev_completions.zsh")
    shutil.copyfile(os.path.join(completion_dir(ctx), ".ouster_dev_completions.zsh"), dest)

    source_line = f"[[ -f {dest} ]] && source {dest}"
    update_shell_profile("~/.zshrc", source_line)


def install_bash_completions(ctx):
    dest = os.path.expanduser("~/.ouster_dev_completions.bash")
    shutil.copyfile(os.path.join(completion_dir(ctx), ".ouster_dev_completions.bash"), dest)

    source_line = f"[[ -f {dest} ]] && source {dest}"
    # Bash users often use .bashrc or .bash_profile
    update_shell_profile("~/.bashrc", source_line)


@click.command()
@click.option("--shell",
              type=click.Choice(["zsh", "bash"], case_sensitive=False),
              help="The shell to install completions for.",
              default=None)
@click.pass_context
def install_completions(ctx, shell):
    """Install command line completions."""
    if shell is None:
        shell = get_parent_shell()
    if shell == "zsh":
        install_zsh_completions(ctx)
    elif shell == "bash":
        install_bash_completions(ctx)
    else:
        click.echo(f"Unsupported shell '{shell}'. Currently supported shells are: zsh, bash.")


def import_module(click_context):
    click_context.utils_group.add_command(install_completions)


def finalize(click_context):
    pass
