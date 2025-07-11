#!/usr/bin/env groovy

@Library('ci-jenkins') import com.swiftnav.ci.*

def context = new Context(context: this)
context.setRepo('gnss-converters')
def builder = context.getBuilder()

String dockerMountArgs = '--privileged -v /mnt/efs/refrepo:/mnt/efs/refrepo'

pipeline {
  agent any

  parameters {
    choice(name: 'AGENT', choices: ['docker.fast', 'docker.highmem'], description: 'Jenkins Node to run instances on')
    string(name: 'FUZZ_TIMEOUT', defaultValue: '1h', description: 'Fuzz test duration (ex: "60s", "10m", "6h", "7d")')
    booleanParam(name: 'SLACK_NOTIFICATION', defaultValue: true, description: 'Slack notification will be sent to #fuzz-testing')
  }

  environment {
    BRANCH_NAME=env.GIT_BRANCH.minus('origin/')
    HANG_TIMEOUT='5000'
    MEMORY_LIMIT='1500'
  }

  stages {
    stage('Fuzz') {
      parallel {
        stage('RTCM to SBP') {
          environment {
            AFL_USE_ASAN='1'
            CC='/usr/bin/afl-gcc'
            CXX='/usr/bin/afl-g++'
            CFLAGS='-m32 -fsanitize=address'
            CXXFLAGS='-m32 -fsanitize=address'
            LDFLAGS='-m32 -fsanitize=address'
          }
          agent {
            dockerfile {
              filename 'fuzz/Dockerfile'
              label params.AGENT
              args dockerMountArgs
            }
          }
          steps {
            script {
              sh("echo Running on \${NODE_NAME}")
              sh('echo "" | sudo tee /proc/sys/kernel/core_pattern')

              gitPrep()
              builder.cmake(
                workDir: 'c',
                cmakeAddArgs: '-DTHIRD_PARTY_INCLUDES_AS_SYSTEM=true -DI_KNOW_WHAT_I_AM_DOING_AND_HOW_DANGEROUS_IT_IS__GNSS_CONVERTERS_DISABLE_CRC_VALIDATION=true',
                buildType: 'Release'
              )
              builder.make(
                workDir: 'c/build',
                target: 'rtcm3tosbp'
              )

              sh("timeout --preserve-status '${params.FUZZ_TIMEOUT}' ./jenkins/afl-parallel.sh -t '${env.HANG_TIMEOUT}' -m '${env.MEMORY_LIMIT}' -i c/afl/testcases/rtcm3tosbp -o afl_output c/build/rtcm3tosbp/src/rtcm3tosbp -w 1945:27750")

              def crashes = findFiles(glob: 'afl_output/**/crashes/*')
              def hangs = findFiles(glob: 'afl_output/**/hangs/*')

              if (crashes.length > 0 || hangs.length > 0) {
                error('Detected a crash and/or hang')
              }
            }
          }
          post {
            always {
              zip(zipFile: 'rtcm3tosbp.zip', dir: 'afl_output', archive: true)
            }
            cleanup {
              cleanWs()
            }
          }
        }

        stage('UBX to SBP') {
          environment {
            AFL_USE_ASAN='1'
            CC='/usr/bin/afl-gcc'
            CXX='/usr/bin/afl-g++'
            CFLAGS='-m32 -fsanitize=address'
            CXXFLAGS='-m32 -fsanitize=address'
            LDFLAGS='-m32 -fsanitize=address'
          }
          agent {
            dockerfile {
              filename 'fuzz/Dockerfile'
              label params.AGENT
              args dockerMountArgs
            }
          }
          steps {
            script {
              sh("echo Running on \${NODE_NAME}")
              sh('echo "" | sudo tee /proc/sys/kernel/core_pattern')

              gitPrep()
              builder.cmake(
                workDir: 'c',
                cmakeAddArgs: '-DTHIRD_PARTY_INCLUDES_AS_SYSTEM=true -DI_KNOW_WHAT_I_AM_DOING_AND_HOW_DANGEROUS_IT_IS__GNSS_CONVERTERS_DISABLE_CRC_VALIDATION=true',
                buildType: 'Release'
              )
              builder.make(
                workDir: 'c/build',
                target: 'ubx2sbp'
              )

              sh("timeout --preserve-status '${params.FUZZ_TIMEOUT}' ./jenkins/afl-parallel.sh -t '${env.HANG_TIMEOUT}' -m '${env.MEMORY_LIMIT}' -i c/afl/testcases/ubx2sbp -o afl_output c/build/ubx2sbp/src/ubx2sbp -w 1945:27750")

              def crashes = findFiles(glob: 'afl_output/**/crashes/*')
              def hangs = findFiles(glob: 'afl_output/**/hangs/*')

              if (crashes.length > 0 || hangs.length > 0) {
                error('Detected a crash and/or hang')
              }
            }
          }
          post {
            always {
              zip(zipFile: 'ubx2sbp.zip', dir: 'afl_output', archive: true)
            }
            cleanup {
              cleanWs()
            }
          }
        }

        stage('SBP to SBP decoder') {
          environment {
            AFL_USE_ASAN='1'
            CC='/usr/bin/afl-gcc'
            CXX='/usr/bin/afl-g++'
            CFLAGS='-m32 -fsanitize=address'
            CXXFLAGS='-m32 -fsanitize=address'
            LDFLAGS='-m32 -fsanitize=address'
          }
          agent {
            dockerfile {
              filename 'fuzz/Dockerfile'
              label params.AGENT
              args dockerMountArgs
            }
          }
          steps {
            script {
              sh("echo Running on \${NODE_NAME}")
              sh('echo "" | sudo tee /proc/sys/kernel/core_pattern')

              gitPrep()
              builder.cmake(
                workDir: 'c',
                cmakeAddArgs: '-DTHIRD_PARTY_INCLUDES_AS_SYSTEM=true -DI_KNOW_WHAT_I_AM_DOING_AND_HOW_DANGEROUS_IT_IS__LIBSBP_DISABLE_CRC_VALIDATION=true',
                buildType: 'Release'
              )
              builder.make(
                workDir: 'c/build',
                target: 'sbp2sbp_decoder'
              )

              sh("timeout --preserve-status '${params.FUZZ_TIMEOUT}' ./jenkins/afl-parallel.sh -t '${env.HANG_TIMEOUT}' -m '${env.MEMORY_LIMIT}' -i c/afl/testcases/sbp2sbp_decoder -o afl_output c/build/sbp2sbp/src/sbp2sbp_decoder")

              def crashes = findFiles(glob: 'afl_output/**/crashes/*')
              def hangs = findFiles(glob: 'afl_output/**/hangs/*')

              if (crashes.length > 0 || hangs.length > 0) {
                error('Detected a crash and/or hang')
              }
            }
          }
          post {
            always {
              zip(zipFile: 'sbp2sbp_decoder.zip', dir: 'afl_output', archive: true)
            }
            cleanup {
              cleanWs()
            }
          }
        }

        stage('SBP to SBP encoder') {
          environment {
            AFL_USE_ASAN='1'
            CC='/usr/bin/afl-gcc'
            CXX='/usr/bin/afl-g++'
            CFLAGS='-m32 -fsanitize=address'
            CXXFLAGS='-m32 -fsanitize=address'
            LDFLAGS='-m32 -fsanitize=address'
          }
          agent {
            dockerfile {
              filename 'fuzz/Dockerfile'
              label params.AGENT
              args dockerMountArgs
            }
          }
          steps {
            script {
              sh("echo Running on \${NODE_NAME}")
              sh('echo "" | sudo tee /proc/sys/kernel/core_pattern')

              gitPrep()
              builder.cmake(
                workDir: 'c',
                cmakeAddArgs: '-DTHIRD_PARTY_INCLUDES_AS_SYSTEM=true -DI_KNOW_WHAT_I_AM_DOING_AND_HOW_DANGEROUS_IT_IS__LIBSBP_DISABLE_CRC_VALIDATION=true',
                buildType: 'Release'
              )
              builder.make(
                workDir: 'c/build',
                target: 'sbp2sbp_encoder'
              )

              sh("timeout --preserve-status '${params.FUZZ_TIMEOUT}' ./jenkins/afl-parallel.sh -t '${env.HANG_TIMEOUT}' -m '${env.MEMORY_LIMIT}' -i c/afl/testcases/sbp2sbp_encoder -o afl_output c/build/sbp2sbp/src/sbp2sbp_encoder")

              def crashes = findFiles(glob: 'afl_output/**/crashes/*')
              def hangs = findFiles(glob: 'afl_output/**/hangs/*')

              if (crashes.length > 0 || hangs.length > 0) {
                error('Detected a crash and/or hang')
              }
            }
          }
          post {
            always {
              zip(zipFile: 'sbp2sbp_encoder.zip', dir: 'afl_output', archive: true)
            }
            cleanup {
              cleanWs()
            }
          }
        }

        stage('RTCM to JSON') {
          agent {
            dockerfile {
              filename 'fuzz/Dockerfile'
              label params.AGENT
              args dockerMountArgs
            }
          }
          steps {
            script {
              sh("echo Running on \${NODE_NAME}")
              sh('echo "" | sudo tee /proc/sys/kernel/core_pattern')

              gitPrep()
              sh("cargo +nightly-2022-07-19 fuzz run rtcm2json --fuzz-dir rust/rtcm/fuzz/ -j \$(nproc --all) -- -max_total_time='${params.CARGO_FUZZ_TIMEOUT}' -timeout='${env.HANG_TIMEOUT}' -rss_limit_mb='${env.MEMORY_LIMIT}'")
            }
          }
          post {
            cleanup {
              cleanWs()
            }
          }
        }

        stage('JSON to RTCM') {
          agent {
            dockerfile {
              filename 'fuzz/Dockerfile'
              label params.AGENT
              args dockerMountArgs
            }
          }
          steps {
            script {
              sh("echo Running on \${NODE_NAME}")
              sh('echo "" | sudo tee /proc/sys/kernel/core_pattern')

              gitPrep()
              sh("cargo +nightly-2022-07-19 fuzz run --features=json json2rtcm --fuzz-dir rust/rtcm/fuzz/ -j \$(nproc --all) -- -max_total_time='${params.CARGO_FUZZ_TIMEOUT}' -timeout='${env.HANG_TIMEOUT}' -rss_limit_mb='${env.MEMORY_LIMIT}'")
            }
          }
          post {
            cleanup {
              cleanWs()
            }
          }
        }
      }
    }
  }
  post {
    always {
      script {
        if (params.SLACK_NOTIFICATION) {
          context.slackNotify(channel: '#fuzz-testing', branches: ['.*'])
        }
        if (context.isTagPush()) {
            if ((context.tagName().startsWith("starling-v") || context.tagName().startsWith("orion-v")) && !context.tagName().contains("develop")) {
                def pipelineLink = "https://jenkins.ci.swift-nav.com/blue/organizations/jenkins/standalone%2Fgnss-converters-fuzz-pipeline/detail/" + context.tagName() + "/" +  currentBuild.id 
                if (currentBuild.currentResult == 'SUCCESS') {
                   slackSend(channel:"#release", 
                              color: "good",
                              message: "Fuzz testing completed: " + context.tagName() + "\nTest Artifacts: " + pipelineLink + "/artifacts")
                }
                if (currentBuild.currentResult == 'FAILURE') {
                   slackSend(channel:"#release", 
                              color: "danger",
                              message: "Fuzz testing failed: " + context.tagName() + "\n" + pipelineLink + "/pipeline")
                }
            }
        }
      }
    }
  }
}
