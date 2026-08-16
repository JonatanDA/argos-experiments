# Argos QUPA robot
## How to compile argos3 QUPA robot ?

1. Clone the repository 

```
git clone https://github.com/coral-espol/qupa_v2.git
```
2. Compile the binaries 

```
cd ~/qupa_v2
rm -rf build && mkdir build && cd build

cmake ../src \
  -DCMAKE_BUILD_TYPE=Release \
  -DARGOS_BUILD_FOR=simulator \
  -DCMAKE_INSTALL_PREFIX="$HOME/swarm_robotics/argos3-dist"

make -j"$(nproc)"
make install
```
> Note: You must to edit your **DCMAKE_INSTALL_PREFIX** with your diretory where argos3 was installed. 

To know where argos3 was installed and your own rute put in your terminal the next command.

```
which argos3 
```

the next command line is an example for other location.
```
cmake -DCMAKE_BUILD_TYPE=Release \
      -DARGOS_BUILD_FOR=simulator \
      -DCMAKE_INSTALL_PREFIX="/usr/local" \
      -DCMAKE_PREFIX_PATH="/usr/local" \
      ../qupa
make -j"$(nproc)"
make install
```
3. Test the argos QUPA robot 

```
argos3 -c test_qupa.argos
```
![QUPA ROBOT](qupas_argops3.png)
![QUPA SENSORS BLOBS](bloobs_leds.png)