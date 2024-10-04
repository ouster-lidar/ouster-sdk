node {
    withEnv(["ARTIFACT_DIR=${WORKSPACE}/artifacts"]) {
        withCredentials([string(credentialsId: 'LIBRARY_PATH', variable: 'LIBRARY_PATH')]) {
            if (params.CLEAN_WORKSPACE) {
                echo "CLEAN_WORKSPACE set: deleting everything"
                cleanWs()
            }
            dir('ouster-sdk') {
                checkout([
                        $class           : 'GitSCM',
                        branches         : [[name: "*/${BRANCH_NAME}"]],
                        extensions       : scm.extensions + [
                                [$class: 'CleanCheckout'],
                                [$class             : 'SubmoduleOption',
                                 disableSubmodules  : false, parentCredentials: true,
                                 recursiveSubmodules: true,
                                 trackingSubmodules : true
                                ]
                        ],
                        userRemoteConfigs: scm.userRemoteConfigs
                ])
            }
            
            dir(env.ARTIFACT_DIR) { deleteDir() }
            
            run_pipeline = load "${LIBRARY_PATH}"
            run_pipeline()
        }
    }
}