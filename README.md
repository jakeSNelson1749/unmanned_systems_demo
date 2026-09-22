# Unmanned Systems Basics
- This repo will be used as a template for the first couple of weeks for your development in unmanned systems, feel free to customize this repo as you see fit. 
- In this repo I include some basic scripts to control a drone and also visualize data real time as well as log data if needed 

# Basic Installation with uv
- This repo uses uv to install the basic python packages you need to run some of the code to install uv https://docs.astral.sh/uv/guides/install-python/ to get started do the following
```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
uv python install
```
Then in this repo's directory do the following
```bash
uv init # init the uv
```
Then install everything you need by entering the following command
```
uv sync
```

# Visualizing Telemetry Realtime 
The `run_realtime_plot.py` script runs records and graphs real time telemetry of the quadcopter during flight
```bash
uv run run_realtime_plot.py
```

# Parsing Data
The `parse_student_flight_data.py` script allows students to parse data `.BIN` files from ardupilot, if you want to know more about how this works and where I get the inforamtion from the files check out this repo https://github.com/jn89b/ardupilot_parse refer to `docs/Homework_1_Parser.md` on how more of this code works too
```bash
uv run parse_student_flight_data.py path/to/flight.BIN
```

# Autonomous Demos
The `autonomous_demos` folder showcases basic scripts to allow control of a quadcopter refer to `autonomous_demos/README.md` for more information 

# Taking off a drone in command line
