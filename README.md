# Erratic T25 control firmware

Using a controllino micro to communicate over CAN with motor driver, and using a
group input (8 wires) to get speed instructions from ABB IRC5.

## Compile (and upload) using `arduino-cli`

``` sh
arduino-cli compile --profile controllino_micro
```

Add `--upload --port /dev/ttyACMX` with the correct port specified.

### In nix development shell

``` sh
arduino-cli compile --fqbn controllino_rp2:rp2040:controllino_micro
```

(Does not play well with `sketch.yaml` for some reason)
