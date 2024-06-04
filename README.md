# TrailblazerML

Collaboration with Legendary Rover scientific club

# Installing Dependencies

Install the project's dependencies using Poetry:

```bash
poetry install
```

## Adding New Dependencies

To add new dependencies to the project, use the poetry add command:

```bash
poetry add package_name
```

For development dependencies, use the --dev flag:

```bash
poetry add package_name --dev
```

## Running the Project

Describe how to run the project, e.g.:

```bash
poetry run python src/main.py
```

# Downloading Data

Download data from [here](https://drive.google.com/open?id=1bTM5eh9wQ4U8p2ANOGbhZqTvDOddFnlI) ~19G to main directory in
repository

after unzipping the directory, and name it `data1500`

```bash
unzip void_1500.zip -d data1500
```

use the following command to unzip the one dataset

```bash
unzip data1500/void_1500-47.zip
```

Run pythonfile `simulation_loop.py` to start simulation  