{
  description = "Setup for development on nix";

  inputs = {
    arduino-nix.url = "github:bouk/arduino-nix";
    arduino-index = {
      url = "github:bouk/arduino-indexes";
      flake = false;
    };
    flake-parts.url = "github:hercules-ci/flake-parts";
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    systems.url = "github:nix-systems/default-linux";
    treefmt-nix.url = "github:numtide/treefmt-nix";
  };
  outputs =
    inputs@{ flake-parts, ... }:
    flake-parts.lib.mkFlake { inherit inputs; } {
      imports = [
        inputs.treefmt-nix.flakeModule
      ];
      systems = import inputs.systems;
      perSystem =
        {
          config,
          lib,
          pkgs,
          self',
          system,
          ...
        }:
        {
          _module.args.pkgs = import inputs.nixpkgs {
            inherit system;
            overlays =
              let
                inherit (inputs) arduino-index arduino-nix;
                inherit (arduino-nix) mkArduinoPackageOverlay mkArduinoLibraryOverlay;
              in
              [
                arduino-nix.overlay
                (mkArduinoPackageOverlay "${arduino-index}/index/package_index.json")
                (mkArduinoPackageOverlay "${arduino-index}/index/package_controllino_rp2_index.json")
                (mkArduinoLibraryOverlay "${arduino-index}/index/library_index.json")
              ];
          };
          packages.arduino-cli =
            let
              # from https://github.com/clerie/arduino-nix/tree/clerie/arduino-env
              mkArduinoEnv =
                {
                  packages ? [ ],
                  libraries ? [ ],
                  runtimeInputs ? [ ],
                }:
                let
                  arduino-cli = pkgs.wrapArduinoCLI {
                    inherit packages libraries;
                  };
                in
                pkgs.stdenvNoCC.mkDerivation (finalAttrs: {
                  name = "arduino-env";

                  buildInputs = [ pkgs.makeWrapper ];

                  phases = [ "buildPhase" ];

                  buildPhase = ''
                    mkdir -p $out
                    makeWrapper ${arduino-cli}/bin/arduino-cli $out/bin/arduino-cli \
                      --prefix PATH : ${lib.makeBinPath runtimeInputs} \
                      --prefix LD_LIBRARY_PATH : ${lib.makeLibraryPath [ pkgs.udev ]}
                  '';
                });

            in
            mkArduinoEnv {
              packages = with pkgs.arduinoPackages; [
                platforms.controllino_rp2.rp2040."2.0.1"
              ];
              runtimeInputs = with pkgs; [
                (python3.withPackages (ps: with ps; [ pyserial ]))
              ];
              libraries = with pkgs.arduinoLibraries; [
                (inputs.arduino-nix.latestVersion ODriveArduino)
              ];

            };
          devShells.default = pkgs.mkShell {
            inputsFrom = [ config.treefmt.build.devShell ];
            packages = with pkgs; [
              self'.packages.arduino-cli
              arduino-language-server
              picotool
            ];
          };
          treefmt.programs = {
            clang-format.enable = true;
            nixfmt.enable = true;
            yamlfmt.enable = true;
          };
        };
    };
}
