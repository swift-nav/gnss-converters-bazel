#!groovy

/**
 * This Jenkinsfile will only work in a Swift Navigation build/CI environment, as it uses
 * non-public docker images and pipeline libraries.
 */

// Use 'ci-jenkins@somebranch' to pull shared lib from a different branch than the default.
// Default is configured in Jenkins and should be from "stable" tag.
@Library("ci-jenkins") import com.swiftnav.ci.*

def context = new Context(context: this)
context.setRepo("gnss-converters")
def builder = context.getBuilder()

/**
 * - Mount the refrepo to keep git operations functional on a repo that uses ref-repo during clone
 **/
String dockerMountArgs = "-v /mnt/efs/refrepo:/mnt/efs/refrepo"

pipeline {
    // Override agent in each stage to make sure we don't share containers among stages.
    agent any
    options {
        // Make sure job aborts after 7 hours if hanging (fuzz testing goes for 6 hours).
        timeout(time: 7, unit: 'HOURS')
        timestamps()
        // Keep builds for 7 days.
        buildDiscarder(logRotator(daysToKeepStr: '3'))
    }

    stages {
        stage('Build') {
            parallel {
                stage('Build c') {
                    agent {
                        dockerfile {
                            args dockerMountArgs
                        }
                    }
                    steps {
                        gitPrep()
                        script {
                            builder.cmake(workDir: "c", cmakeAddArgs: "-DTHIRD_PARTY_INCLUDES_AS_SYSTEM=true -Dnov2sbp_BUILD=true")
                            builder.make(workDir: "c/build")
                        }
                        script {
                            builder.make(workDir: "c/build", target: "do-all-tests")
                        }
                    }
                }
                stage('Format & Lint c') {
                    agent {
                        docker {
                            image '571934480752.dkr.ecr.us-west-2.amazonaws.com/swift-build-modern:2022-07-29'
                            args dockerMountArgs
                        }
                    }
                    steps {
                        gitPrep()
                        script {
                            builder.cmake(workDir: "c", cmakeAddArgs: "-DTHIRD_PARTY_INCLUDES_AS_SYSTEM=true -Dnov2sbp_BUILD=true")
                            builder.make(workDir: "c/build", target: "clang-format-all-check")
                            builder.make(workDir: "c/build", target: "clang-tidy-all-check")
                        }
                    }
                    post {
                        always {
                            archiveArtifacts(artifacts: 'c/fixes.yaml', allowEmptyArchive: true)
                        }
                    }
                }
                stage('Bazel Build') {
                    agent {
                        docker {
                            image '571934480752.dkr.ecr.us-west-2.amazonaws.com/swift-build-bazel:2022-09-28'
                        }
                    }
                    steps {
                        gitPrep()
                        script {
                            sh('bazel test //...')
                        }
                    }
                }
                stage('Sonarcloud') {
                    agent {
                        docker {
                            image '571934480752.dkr.ecr.us-west-2.amazonaws.com/swift-build-bazel:2022-09-28'
                        }
                    }
                    steps {
                        gitPrep()
                        script {
                            withCredentials([string(credentialsId: 'sonarcloud-gnss-converters-token', variable: 'SONAR_TOKEN')]) {
                                sh('TESTENV=codecov ./ci-build.sh')
                            }
                        }
                    }
                }
                stage('Static Code Analysis') {
                    agent {
                        docker {
                            image '571934480752.dkr.ecr.us-west-2.amazonaws.com/swift-build-bazel:test'
                        }
                    }
                    steps {
                        gitPrep()
                        script {
                            sh('bazel run //bazel:qac_compile_commands')
                        }
                        withEnv(["BUILD_ROOT=${env.WORKSPACE}", "GIT_TAG=${env.GIT_COMMIT}"]) {
                            withCredentials([usernamePassword(credentialsId: 'helix-qac-dashboard-login', usernameVariable: 'QAC_UPLOAD_SERVER_USERNAME', passwordVariable: 'QAC_UPLOAD_SERVER_PASSWORD')]) {
                                sh('qac || true')
                                sh('qac upload')
                            }
                        }
                    }
                }
            }
        }
    }
    post {
        success {
            script {
                def automatedPr = new AutomatedPR(context: context)
                automatedPr.merge()
            }
        }

        failure {
            script {
                def automatedPr = new AutomatedPR(context: context)
                automatedPr.alertSlack()
            }
        }

        always {
            script {
                context.slackNotify(channel: '#positioning-dev')
                context.slackNotify(channel: '#release', branches: ['.*v.*-release'])
            }
        }

        cleanup {
            cleanWs()
        }
    }
}

/**
 * Retrieve test data submodule (it's large and not needed for all stages, so
 * don't fetch it unless needed).
 * @param args
 * @return
 */
def fetchTestData(Map args = [:]) {
    sh 'git submodule update --init --checkout --recursive c/afl/findings'
}
