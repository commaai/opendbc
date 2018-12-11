pipeline {
    agent { label 'Debian9IntegrationTests' }

    options {
        disableConcurrentBuilds()
        timestamps()
    }

    stages {
        stage('Prepare') {
            agent { label 'Debian9IntegrationTests' }
            steps {
              script {
                sh "scripts/linux/clean.sh"
                sh "scripts/linux/lint_bash.sh"
              }
            }
        }

        stage('Debian Server Integration Tests') {
            steps {
                lock('loop-debian') {
                    lock('can-bus-1') {
                        timeout(1)
                        {
                            sh "scripts/linux/run_integration_tests.sh ${LOOPSERVERIPADDR}"
                        }
                    }
                }
            }
            post {
                always {
                    junit "scripts/linux/output/run-integration-tests/${LOOPSERVERIPADDR}*.xml"
                }
            }
        }
    }
}
