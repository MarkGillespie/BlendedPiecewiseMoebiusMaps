# BPM: Blended Piecewise M&ouml;bius Maps
Polyscope implementation of [BPM: Blended Piecewise M&ouml;bius Maps](https://arxiv.org/pdf/2306.12792.pdf) by Rorberg, Vaxman & Ben-Chen.
The code takes in a mesh with disk topology, flattens it with [BFF](https://geometrycollective.github.io/boundary-first-flattening/), and shows the parameterization with either blended M&ouml;bius interpolation, or piecewise-linear interpolation.

|![BPM interpolated camel head](images/BPM.png "BPM interpolation")|![Piecewise-linear camel head](images/PL.png "Piecewise-linear interpolation")|
|:----:|:-----:|
|BPM interpolation|Piecewise-linear interpolation|

## Building
To build the code, you can run
```
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j7
```

Then run the code with
```
bin/run ../data/camel-head.obj
```

Run the tests with
```
bin/test
```
