## Unit tests - on device

The tests in `test\embedded` run 'on device'.

To run these tests follow:
```sh
mkdir SimpleCAN_test
cd SimpleCAN_test
mkdir lib
cd lib
git clone git clone git@github.com:owennewo/SimpleCAN.git
cd ..
cp lib/SimpleCAN/examples/platformio.ini .
```
then edit `default_envs` in `./platformio.ini` to choose the target you wish to test
