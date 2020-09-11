# SHARED_SW
Code shared between the `dev_fw`, `ouster_sw`, and potentially even
`pylasertestrig` repos. Future home of the one true sensor client and related
utilities like visualizers and debugging tools.

See the `README.md` in each subdirectory for details.

## IMPORTANT
Do not modify files both inside and outside `ouster_example/` in a single
commit! It can make mirroring the subtree to github really annoying. Moving a
file from one to the other seems to be ok.

## Contents
* [ouster_example/](ouster_example/README.md) **public** code mirrored on
  [github](https://github.com/ouster-lidar/ouster_example)
* [ouster_python_client/](ouster_python_client/README.md) WIP python client
  sharing code with `ouster_example/ouster_client`

## Ouster Example Directory
The `ouster_example` subdirectory contains code that gets mirrored to github
using `git subtree`. For now, the process is pretty manual:

The repo was set up using `git-subtree`:

* `git remote add bitbucket/ouster_example git@bitbucket.org:ouster_io/ouster_example.git`
* `git subtree add -P ouster_example bitbucket/ouster_example master`

The `ouster_example` repo should no longer be necessary.

To push/pull from github (workflow WIP):

* `git remote add github/ouster_example
  git@github.com:ouster-lidar/ouster_example.git`
* `git checkout -b github github/ouster_example/master`
* `git subtree split -P ouster_example -b github-staging` creates a branch
  suitable for merging back into the local `github`
* `git subtree -P ouster_example merge github` merges the changes from local
  `github` back into the `ouster_example` subtree
