{
  description = "Setup for development on nix";

  inputs = {
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
        { config, pkgs, ... }:
        {
          devShells.default = pkgs.mkShell {
            inputsFrom = [ config.treefmt.build.devShell ];
            packages = with pkgs; [
              arduino-cli
              python3
              (python3.withPackages (python-pkgs: [
                python-pkgs.pyserial
              ]))
              (writeShellScriptBin "a" "${lib.getExe arduino-cli} $@")
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
