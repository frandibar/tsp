Given a csv input file with fields
```
nombre,direccion,localidad,lat,long
```

```
$ nix-shell
$ ./tsp.py input.csv
```

The output returns an itinerary joining all locations, starting at the first from the list.
Optimization is done according to distance.
In addition a url is printed for viewing in the browser with google maps.
