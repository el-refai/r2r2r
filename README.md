# r2r2r

## Installation
Follow IsaacLab's installation: https://isaac-sim.github.io/IsaacLab/main/source/setup/installation/index.html
Note: You may get an error partway through installing IsaacLab saying that it could not locate `rsl-rl`. This is fine as we do not need a full installation of IsaacLab for our renderer to work.
```
git clone https://github.com/BerkeleyAutomation/xi.git
git submodule init && git submodule update
pip install -e .
cd dependencies/rsrd/
git submodule init && git submodule update
```
Then install all with `pip install -e .` all of the dependencies including those nested within `rsrd`.


## uv Installation
```
git clone https://github.com/el-refai/r2r2r.git
git submodule init && git submodule update
pip install uv
uv venv --python 3.10.15
source .venv/bin/activate
uv pip install torch==2.5.1 torchvision==0.20.1 --index-url https://download.pytorch.org/whl/cu118
uv pip install --upgrade pip
uv pip install 'isaacsim[all,extscache]==4.5.0' --extra-index-url https://pypi.nvidia.com
cd dependencies/IsaacLab
./isaaclab.sh --install
cd ../dependencies/rsrd
git submodule init && git submodule update
cd dependencies/jaxmp
uv pip install -e .
cd ../jaxls
uv pip install -e .
cd ../../../..
uv pip install -e .
cd dependencies/trajgen
uv pip install -e .
uv pip install viser
uv pip install viser[examples]
uv pip install -U "jax[cuda12]"
```
