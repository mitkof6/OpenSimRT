# Building OpenSim (Ubuntu 18.04)

For this to work, we currently maintain a custom version of `opensim-core` until
some of the fixes are included in the mainstream branch. For you to build this,
you can run the script provided in this repository:

`build_opensim-core`

This will build and install OpenSim libraries in the install folder inside
`opensim-core`. After building OpenSim, configure the environmental variables as
follows:

```shell
export OPENSIM_HOME=/path-to-filesystem/opensim-core
export OpenSim_DIR=$OPENSIM_HOME/lib/cmake/OpenSim
export LD_LIBRARY_PATH=$OPENSIM_HOME/lib:$LD_LIBRARY_PATH
export PATH=$OPENSIM_HOME/bin:$PATH
export PATH=$OPENSIM_HOME/libexec/simbody:$PATH
```

The Python bindings are build with the system Python version (substitute X.X
below). To install run:

```shell
cd $OPENSIM_HOME/lib/pythonX.X/site-packages
python3 setup.py install --user
```
To test the Python bindings try importing OpenSim in Python3:

```
cd ~
python3 -c "import opensim"
```

Sometimes the above command fails. The `opensim` folder that is created in
`$OPENSIM_HOME/lib/pythonX.X/site-packages` after running `python3 setup.py
install --user` may not be properly copied in the Python `site-packages`. In
that case, you can copy it manually.

```
cd -rf $OPENSIM_HOME/lib/pythonX.X/site-packages/opensim ~/.local/lib/pythonX.X/site-packages
```

# Pre-build version of OpenSim v4.1 for Ubuntu 18.04

*For Windows or Mac users one can download the pre-build version of
OpenSim v4.1 and setup the Python bindings.*

For convenience a pre-build version of OpenSim v4.1 for Ubuntu 18.04
can be downloaded through the following URL:

https://sourceforge.net/projects/dependencies/files/opensim-core/opensim-core-4.1-ubuntu-18.04.tar.xz/download

Extract and move the opensim-core folder to a convenient
location. Then configure the environmental variables as follows:

```shell
export OPENSIM_HOME=/path-to-filesystem/opensim-core
export OpenSim_DIR=$OPENSIM_HOME/lib/cmake/OpenSim
export LD_LIBRARY_PATH=$OPENSIM_HOME/lib:$LD_LIBRARY_PATH
export PATH=$OPENSIM_HOME/bin:$PATH
export PATH=$OPENSIM_HOME/libexec/simbody:$PATH
```

The Python bindings were build with Python 3.6. To install run:

```shell
cd $OPENSIM_HOME/lib/python3.6/site-packages
python3 setup.py install --user
```
To test the Python bindings try importing OpenSim in python3:

`python3 -c "import opensim"`
