node {
    withEnv(["ARTIFACT_DIR=${WORKSPACE}/artifacts"]) {
        withCredentials([string(credentialsId: 'LIBRARY_PATH', variable: 'LIBRARY_PATH')]) {
            if (params.CLEAN_WORKSPACE) {
                echo "CLEAN_WORKSPACE set: deleting everything"
                cleanWs()
            }
            // env.BRANCH_NAME is only set by multibranch jobs; generic Pipeline jobs need fallbacks.
            def scmBranch = scm.branches[0].name.replaceAll(/^\*\//, '') 
            def checkoutBranch = env.BRANCH_NAME ?: scmBranch ?: 'develop'
            echo "checkoutBranch: ${checkoutBranch}"

            if (!env.BRANCH_NAME) {
                env.BRANCH_NAME = checkoutBranch
            }
            def isMRBuild = env.CHANGE_ID != null
            def extensions = scm.extensions + [
                [$class: 'CleanCheckout'],
                [$class: 'SubmoduleOption',
                 disableSubmodules: false,
                 parentCredentials: true,
                 recursiveSubmodules: true
                ]
            ]
            if (isMRBuild) {
                checkoutBranch = env.CHANGE_BRANCH
                // Filter out MergeWithGitSCMExtension to prevent automatic merge
                extensions = extensions.findAll { extension ->
                    !(extension instanceof jenkins.plugins.git.MergeWithGitSCMExtension)
                }
            }
            dir('ouster-sdk') {
                checkout([
                        $class           : 'GitSCM',
                        branches         : [[name: "*/${checkoutBranch}"]],
                        extensions: extensions,
                        userRemoteConfigs: scm.userRemoteConfigs
                ])

                withCredentials([gitUsernamePassword(credentialsId: 'GITLAB_PACKAGE_ACCESS_TOKEN',
                                        gitToolName: 'Default')]) {
                    sh label: "Update submodule branch and initialize submodules", script: """
                        set -ex
                        python3 -m venv .venv
                        . .venv/bin/activate
                        python3 -m pip install --quiet gitpython
                        python3 scripts/update_submodule_branch.py
                        rm -rf .venv
                    """
                    retry(3) {
                        try {
                            sh label: "git submodule update --remote --recursive", script: """
                                set -ex
                                git submodule update --remote --recursive
                            """
                        } catch (org.jenkinsci.plugins.workflow.steps.FlowInterruptedException e) {
                            throw e
                        } catch (Exception e) {
                            sleep(time: 30, unit: 'SECONDS')
                            throw e
                        }
                    }
                }
            }
            dir(env.ARTIFACT_DIR) { deleteDir() }

            def run_pipeline = load "${LIBRARY_PATH}"
            run_pipeline()
        }
    }
}
