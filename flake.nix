{
  description = "Setup for development on nix";

  inputs = {
    flake-parts.url = "github:hercules-ci/flake-parts";
    git-hooks-nix.url = "github:cachix/git-hooks.nix";
    nixpkgs.url = "github:NixOS/nixpkgs/nixos-unstable";
    systems.url = "github:nix-systems/default-linux";
  };
  outputs =
    inputs@{ flake-parts, ... }:
    flake-parts.lib.mkFlake { inherit inputs; } {
      imports = [ inputs.git-hooks-nix.flakeModule ];
      systems = import inputs.systems;
      perSystem =
        { config, pkgs, ... }:
        {
          devShells.default = pkgs.mkShell {
            inputsFrom = [ config.pre-commit.devShell ];
            packages = with pkgs; [
              arduino-cli
              python3
              (python3.withPackages (python-pkgs: [
                python-pkgs.pyserial
              ]))
              (writeShellScriptBin "a" "${lib.getExe arduino-cli} $@")
            ];
          };
          formatter = pkgs.nixfmt-rfc-style;
          pre-commit.settings.hooks = {
            clang-format.enable = true;
            nixfmt-rfc-style.enable = true;
          };
        };
    };
}
