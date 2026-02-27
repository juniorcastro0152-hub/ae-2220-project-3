# Cosmobee

## Installation

This project uses Python-1.0.1. Pip is the preferred dependency manager for this project

```bash
pip install -r requirements.txt
```

## Usage

Simply run the below command to run the simple 2D Cosmobee simulation.

```bash
python main.py
```

Arguments:

- `--save filename`: Saves the animation to `filename`. The encoding process takes a significant amount of time, and this should only be used once you have made sure the animation looks good.
- `--fps N`: Sets the frames per second of the exported video to `N` (default is 30).
- `--only-animation`: Only shows the animation and not the additional logging plots.
- With no arguments, all plots will be shown and no video will be exported.

Notes:
 just `pip install -r requirements.txt` 
