node {
    withEnv(["ARTIFACT_DIR=${WORKSPACE}/artifacts"]) {
        withCredentials([string(credentialsId: 'LIBRARY_PATH', variable: 'LIBRARY_PATH')]) {
            if (params.CLEAN_WORKSPACE) {
                echo "CLEAN_WORKSPACE set: deleting everything"
                cleanWs()
            }
            def checkoutBranch = BRANCH_NAME
            def isMRBuild = env.CHANGE_ID != null
            def extensions = scm.extensions + [
                [$class: 'CleanCheckout'],
                [$class: 'SubmoduleOption',
                 disableSubmodules: false,
                 parentCredentials: true,
                 recursiveSubmodules: true,
                 trackingSubmodules: true
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
            }
            dir(env.ARTIFACT_DIR) { deleteDir() }

            def run_pipeline = load "${LIBRARY_PATH}"
            run_pipeline()
        }
    }
}
