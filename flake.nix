{
  inputs = {
    nixpkgs.url = "github:nixos/nixpkgs/nixos-unstable";
    flake-parts.url = "github:hercules-ci/flake-parts";
    systems.url = "github:nix-systems/default";
    devshell.url = "github:numtide/devshell";
  };

  outputs =
    inputs@{
      flake-parts,
      systems,
      devshell,
      ...
    }:
    flake-parts.lib.mkFlake { inherit inputs; } {
      systems = import systems;
      imports = [
        devshell.flakeModule
      ];
      perSystem =
        { pkgs, ... }:
        let
          inherit (pkgs.lib)
            optionalString
            ;

          isDarwin = pkgs.stdenv.isDarwin;

          colconDefaults = pkgs.writeText "defaults.yaml" ''
            build:
              cmake-args:
                - -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
                - -DPython_FIND_VIRTUALENV=ONLY
                - -DPython3_FIND_VIRTUALENV=ONLY
                ${optionalString isDarwin "- -DCMAKE_BUILD_WITH_INSTALL_RPATH=ON"}
          '';
        in
        {
          devshells.default = {
            env = [
              {
                name = "COLCON_DEFAULTS_FILE";
                value = toString colconDefaults;
              }
            ];
            devshell = {
              packages = with pkgs; [
                pixi
              ];
              startup.activate.text = ''
                if [ -f pixi.toml ]; then
                  ${optionalString isDarwin ''
                    export DYLD_FALLBACK_LIBRARY_PATH="$PWD/.pixi/envs/default/lib:$DYLD_FALLBACK_LIBRARY_PATH"
                  ''}
                  eval "$(pixi shell-hook)";
                fi
              '';
              motd = "";
            };
          };
        };
    };
}
