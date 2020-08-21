from numpy import ones, pi, array, linspace
from pyleecan.Classes.Simu1 import Simu1
from pyleecan.Classes.InputCurrent import InputCurrent
from pyleecan.Classes.MagFEMM import MagFEMM

from pyleecan.Functions.load import load

from os import chdir
chdir('..')
# Import the machine from a script
efu = load(r'C:\Users\610099764\efu.json')

# Plot the machine
im=efu.plot()

rotor_speed = 2000 # [rpm]

# Create the Simulation
mySimu = Simu1(name="EM_SIPMSM_AL_001", machine=efu)

# Defining Simulation Input
mySimu.input = InputCurrent()

# time discretization [s]
mySimu.input.time.value= linspace(start=0, stop=60/rotor_speed, num=16, endpoint=False)# 16 timesteps

# Angular discretization along the airgap circonference for flux density calculation
mySimu.input.angle.value = linspace(start = 0, stop = 2*pi, num=2048, endpoint=False) # 2048 steps

# Rotor speed as a function of time [rpm]
mySimu.input.Nr.value = ones(16) * rotor_speed

# Stator currents as a function of time, each column correspond to one phase [A]
mySimu.input.Is.value = array(
    [
        [ 1.77000000e+02, -8.85000000e+01, -8.85000000e+01],
        [ 5.01400192e-14, -1.53286496e+02,  1.53286496e+02],
        [-1.77000000e+02,  8.85000000e+01,  8.85000000e+01],
        [-3.25143725e-14,  1.53286496e+02, -1.53286496e+02],
        [ 1.77000000e+02, -8.85000000e+01, -8.85000000e+01],
        [ 2.11398201e-13, -1.53286496e+02,  1.53286496e+02],
        [-1.77000000e+02,  8.85000000e+01,  8.85000000e+01],
        [-3.90282030e-13,  1.53286496e+02, -1.53286496e+02],
        [ 1.77000000e+02, -8.85000000e+01, -8.85000000e+01],
        [ 9.75431176e-14, -1.53286496e+02,  1.53286496e+02],
        [-1.77000000e+02,  8.85000000e+01,  8.85000000e+01],
        [-4.33634526e-13,  1.53286496e+02, -1.53286496e+02],
        [ 1.77000000e+02, -8.85000000e+01, -8.85000000e+01],
        [ 4.55310775e-13, -1.53286496e+02,  1.53286496e+02],
        [-1.77000000e+02,  8.85000000e+01,  8.85000000e+01],
        [-4.76987023e-13,  1.53286496e+02, -1.53286496e+02]
    ]
)

from pyleecan.Classes.MagFEMM import MagFEMM
# Definition of the magnetic simulation (is_mmfr=False => no flux from the magnets)
mySimu.mag = MagFEMM(
    type_BH_stator=0, # 0 to use the B(H) curve,
                           # 1 to use linear B(H) curve according to mur_lin,
                           # 2 to enforce infinite permeability (mur_lin =100000)
    type_BH_rotor=0,  # 0 to use the B(H) curve,
                           # 1 to use linear B(H) curve according to mur_lin,
                           # 2 to enforce infinite permeability (mur_lin =100000)
    angle_stator=0,  # Angular position shift of the stator
    file_name = "", # Name of the file to save the FEMM model
)

# We only use the magnetic part
mySimu.force = None
mySimu.struct = None
mySimu.mag.is_symmetry_a=False   # 0 Compute on the complete machine, 1 compute according to sym_a and is_antiper_a
mySimu.mag.sym_a = 4 # Number of symmetry for the angle vector
mySimu.mag.is_antiper_a=True # To add an antiperiodicity to the angle vector

mySimu.mag.is_get_mesh = True # To get FEA mesh for latter post-procesing
mySimu.mag.is_save_FEA = False # To save FEA results in a dat file
from pyleecan.Classes.Output import Output
myResults = Output(simu=mySimu)
mySimu.run()