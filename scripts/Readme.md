# Dependencies for DJI Termal images

// https://docs.conda.io/projects/miniconda/en/latest/

```

mkdir -p ~/miniconda3
wget https://repo.anaconda.com/miniconda/Miniconda3-latest-Linux-x86_64.sh -O ~/miniconda3/miniconda.sh
bash ~/miniconda3/miniconda.sh -b -u -p ~/miniconda3
rm -rf ~/miniconda3/miniconda.sh

```

// install libreria

```
sudo apt-get install exiftool
// in conda use
python3 -m pip install git+https://github.com/detecttechnologies/thermal_base.git@main

conda create --name DJIThermal python=3.9
conda activate DJIThermal

```
