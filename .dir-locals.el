((nil . ((eval . (add-to-list 'eglot-server-programs
                                   '(arduino-mode .
                                     ("arduino-language-server"
                                      "-clangd" "clangd"
                                      "-cli" "arduino-cli"
                                      "-cli-config" "./arduino-cli.yaml"
                                      "-fqbn" "controllino_rp2:rp2040:controllino_micro")))))
