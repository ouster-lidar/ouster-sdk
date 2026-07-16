import click
import os

import build_libs


additional_build_cleanup_dirs = []
additional_artifact_cleanup_dirs = []


@click.command(name="build")
@click.pass_context
def build_cleanup(ctx):
    """Cleanup Build Directories."""
    build_libs.rmtree_readonly(ctx.obj.sdk_build_dir)
    for item in additional_build_cleanup_dirs:
        build_libs.rmtree_readonly(item)


@click.command(name="all")
@click.pass_context
def all_cleanup(ctx):
    """Cleanup ALL directories created by this context."""
    directories_to_clean = [
        # Build directories
        ctx.obj.sdk_build_dir,
        # Use private attributes directly to avoid the property side-effect
        # of creating these directories just to delete them.
        ctx.obj._sdk_artifact_dir,
        ctx.obj._dev_persistent_dir,
    ]
    print("Cleaning up directories created by context...")
    for directory in directories_to_clean:
        if os.path.exists(directory):
            print(f"Removing: {directory}")
            build_libs.rmtree_readonly(directory)
        else:
            print(f"Skipping (doesn't exist): {directory}")
    # Clean additional directories
    for item in additional_build_cleanup_dirs + additional_artifact_cleanup_dirs:
        if os.path.exists(item):
            print(f"Removing additional: {item}")
            build_libs.rmtree_readonly(item)
    print("Cleanup complete!")


@click.command(name="artifacts")
@click.pass_context
def artifacts_cleanup(ctx):
    """Cleanup Artifacts Directories."""
    # Use private attribute directly to avoid the property side-effect of
    # creating the directory just to delete it.
    build_libs.rmtree_readonly(ctx.obj._sdk_artifact_dir)
    for item in additional_artifact_cleanup_dirs:
        build_libs.rmtree_readonly(item)


def import_module(click_context):
    click_context.cleanup_group.add_command(build_cleanup)
    click_context.cleanup_group.add_command(artifacts_cleanup)
    click_context.cleanup_group.add_command(all_cleanup)


def finalize(click_context):
    pass
