import click
import os
import shutil


additional_build_cleanup_dirs = []
additional_artifact_cleanup_dirs = []


@click.command(name="build")
@click.pass_context
def build_cleanup(ctx):
    """Cleanup Build Directories."""
    shutil.rmtree(ctx.obj.sdk_build_dir, ignore_errors=True)
    for item in additional_build_cleanup_dirs:
        shutil.rmtree(item, ignore_errors=True)


@click.command(name="all")
@click.pass_context
def all_cleanup(ctx):
    """Cleanup ALL directories created by this context."""
    directories_to_clean = [
        # Build directories
        ctx.obj.sdk_build_dir,
        # Artifact directories
        ctx.obj._sdk_artifact_dir,
        ctx.obj._dev_persistant_dir,
    ]
    print("Cleaning up directories created by context...")
    for directory in directories_to_clean:
        if os.path.exists(directory):
            print(f"Removing: {directory}")
            shutil.rmtree(directory, ignore_errors=True)
        else:
            print(f"Skipping (doesn't exist): {directory}")
    # Clean additional directories
    for item in additional_build_cleanup_dirs + additional_artifact_cleanup_dirs:
        if os.path.exists(item):
            print(f"Removing additional: {item}")
            shutil.rmtree(item, ignore_errors=True)
    print("Cleanup complete!")


@click.command(name="artifacts")
@click.pass_context
def artifacts_cleanup(ctx):
    """Cleanup Artifacts Directories."""
    shutil.rmtree(ctx.obj.sdk_artifact_dir, ignore_errors=True)
    for item in additional_artifact_cleanup_dirs:
        shutil.rmtree(item, ignore_errors=True)


def import_module(click_context):
    click_context.cleanup_group.add_command(build_cleanup)
    click_context.cleanup_group.add_command(artifacts_cleanup)
    click_context.cleanup_group.add_command(all_cleanup)


def finalize(click_context):
    pass
