# Balancing Robot

<!-- TABLE OF CONTENTS -->
<details open="open">
  <summary>Table of Contents</summary>
  <ol>
    <li><a href="#about-the-project">About The Project</a>
    </li>
    <li><a href="#installing-prerequisites">Installing Prerequisites</a></li>
      <ul>
        <li><a href="#creating-the-enviroment">Creating the Enviroment</a></li>
        <li><a href="#enviroment-activation">Enviroment Activation</a></li>
        <li><a href="#install-required-packages">Install Required Packages</a></li>
        <li><a href="#install-app-in-editable-mode">Install App in Editable Mode</a></li>
      </ul>
  </ol>
</details>


<!-- ABOUT THE PROJECT -->
## About The Project
Simulation of a balancing robot with mujoco.\
See the [notebook](Python/simulation.ipynb).


<!-- INSTALLING PREREQUISITES -->
## Installing Prerequisites
### Creating the Enviroment
After cloning the repository create a python virtual enviroment.<br>
From the repository root directory run the follwing command:

    python -m venv .venv

### Enviroment Activation
After the enviroment has been created it must be activated

    .venv/Scripts/activate (windows)

    source .venv/bin/activate (linux)

**NOTE:** On Windows if the folder is on a network location the following command may be needed to allow execution

    Set-ExecutionPolicy -Scope CurrentUser -ExecutionPolicy RemoteSigned

### Install Required Packages

    pip install -r requirements.txt

**NOTE:** The requirements.txt file is genrated with the following command. <br>
From the .txt file remove the app if it was installed in the enviroment.

    pip freeze > requirements.txt