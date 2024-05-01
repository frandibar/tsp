with import <nixpkgs> {};

(pkgs.python3.withPackages (ps: with ps;
  [googlemaps
   ortools
  ]
)).env
