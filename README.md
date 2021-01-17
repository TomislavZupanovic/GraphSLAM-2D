# GraphSLAM-2D
Simulation of Graph SLAM in 2D environment using Python. The 2D world is generated with specified parameters
like world size and number of objects or landmarks in that environment. Landmarks are generated randomly inside 2D 
world with defined number of those landmarks, then Robot is placed to move and sense in environment with defined 
number of steps. Robot is measuring distances to landmarks inside it's measurement range and that data with
Robot motion actions is the input for SLAM algorithm which is mapping landmarks and robot positions in a 2D world.
SLAM results are estimated Robot and Landmarks positions.
For more details see [Notebook](https://github.com/TomislavZupanovic/GraphSLAM-2D/blob/main/GraphSLAM.ipynb).

**Installation**

- Clone repository on your computer:

`$ git clone https://github.com/TomislavZupanovic/GraphSLAM-2D.git`

- Create and activate virtual environment (for Windows):

`$ python -m venv slam`

`$ slam/Scripts/activate`

- Install requirements:

`$ pip install -r requirements.txt`

- For Linux/MacOS see: [Virtual Environments](https://packaging.python.org/guides/installing-using-pip-and-virtual-environments/)

**Run SLAM**

- Navigate to repository:

`$ cd .../GraphSLAM-2D`

- Run script:

`$ python run.py --steps 20 --num_landmarks 10`

There are options for other parameters but best if default:
 
 `--world_size --measurement_range --measurement_noise and --motion_noise`
 
 **Example results**
 
 <img src="https://github.com/TomislavZupanovic/GraphSLAM-2D/blob/main/images/script_results.jpg" width="600" height="550">

<img src="https://github.com/TomislavZupanovic/GraphSLAM-2D/blob/main/images/Figure1.jpeg" width="600" height="550">