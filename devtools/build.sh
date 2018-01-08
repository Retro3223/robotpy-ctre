pushd ..
RPY_CTRE_DEVDIR=devdir python setup.py build 2>&1 
cp build/lib.linux-x86_64-3.6/ctre/_impl/autogen/* ctre/_impl/autogen/
popd
