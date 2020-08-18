
from os import chdir
from pyleecan.Classes.MachineIPMSM import MachineIPMSM
from pyleecan.Classes.MatMagnetics import MatMagnetics
from pyleecan.Classes.Material import Material
from pyleecan.Classes.Frame import Frame
from pyleecan.Classes.Shaft import Shaft
from pyleecan.Classes.HoleM50 import HoleM50
from pyleecan.Classes.LamHole import LamHole
from pyleecan.Classes.CondType11 import CondType11
from pyleecan.Classes.WindingDW1L import WindingDW1L
from pyleecan.Classes.SlotW11 import SlotW11
from pyleecan.Classes.LamSlotWind import LamSlotWind
from pyleecan.Functions.load import load
mm = 1e-3  # Millimeter
chdir('C:/Users/610099764')
# Lamination setup
stator = LamSlotWind(
    Rint=80.95 * mm,  # internal radius [m]
    Rext=134.62 * mm,  # external radius [m]
    # Lamination stack active length [m] without radial ventilation airducts
    L1=83.82 * mm,
    # but including insulation layers between lamination sheets
    Nrvd=0,  # Number of radial air ventilation duct
    Kf1=0.95,  # Lamination stacking / packing factor
    is_internal=False,
    is_stator=True,
)


# Slot setup
stator.slot = SlotW11(
    Zs=48,  # Slot number
    H0=1.0 * mm,  # Slot isthmus height
    H1=0,  # Height
    H2=33.3 * mm,  # Slot height below wedge
    W0=1.93 * mm,  # Slot isthmus width
    W1=5 * mm,  # Slot top width
    W2=8 * mm,  # Slot bottom width
    R1=0.004  # Slot bottom radius
)


# Winding setup
stator.winding = WindingDW1L(
    qs=3,  # number of phases
    Lewout=0,  # staight length of conductor outside lamination before EW-bend
    p=4,  # number of pole pairs
    Ntcoil=9,  # number of turns per coil
    Npcpp=1,  # number of parallel circuits per phase
    # 0 not to change the stator winding connection matrix built by pyleecan number
    Nslot_shift_wind=2,
    # of slots to shift the coils obtained with pyleecan winding algorithm
    # (a, b, c becomes b, c, a with Nslot_shift_wind1=1)
    is_reverse_wind=True  # True to reverse the default winding algorithm along the airgap
    # (c, b, a instead of a, b, c along the trigonometric direction)
)

# Conductor setup
stator.winding.conductor = CondType11(
    Nwppc_tan=1,  # stator winding number of preformed wires (strands)
    # in parallel per coil along tangential (horizontal) direction
    Nwppc_rad=1,  # stator winding number of preformed wires (strands)
    # in parallel per coil along radial (vertical) direction
    Wwire=0.000912,  # single wire width without insulation [m]
    Hwire=2e-3,  # single wire height without insulation [m]
    Wins_wire=1e-6,  # winding strand insulation thickness [m]
    type_winding_shape=0,  # type of winding shape for end winding length calculation
    # 0 for hairpin windings
    # 1 for normal windings
)


# Rotor setup
rotor = LamHole(
    Rint=55.32 * mm,  # Internal radius
    Rext=80.2 * mm,  # external radius
    is_internal=True,
    is_stator=False,
    L1=stator.L1  # Lamination stack active length [m]
    # without radial ventilation airducts but including insulation layers between lamination sheets
)
rotor.hole = list()
rotor.hole.append(
    HoleM50(
        Zh=8,  # Number of Hole around the circumference
        W0=42.0 * mm,  # Slot opening
        W1=0,  # Tooth width (at V bottom)
        W2=0,  # Distance Magnet to bottom of the V
        W3=14.0 * mm,  # Tooth width (at V top)
        W4=18.9 * mm,  # Magnet Width
        H0=10.96 * mm,  # Slot Depth
        H1=1.5 * mm,  # Distance from the lamination Bore
        H2=1 * mm,  # Additional depth for the magnet
        H3=6.5 * mm,  # Magnet Height
        H4=0,  # Slot top height
    )
)


# Set shaft
shaft = Shaft(Drsh=rotor.Rint * 2,  # Diamater of the rotor shaft [m]
              # used to estimate bearing diameter for friction losses
              Lshaft=1.2  # length of the rotor shaft [m]
              )


# Loading Materials
M400_50A = load('pyleecan/Data/Material/M400-50A.json')
Copper1 = load('pyleecan/Data/Material/Copper1.json')

# Defining magnets
Magnet_prius = Material(name="Magnet_prius")

# Definition of the magnetic properties of the material
Magnet_prius.mag = MatMagnetics(
    mur_lin=1.05,  # Relative magnetic permeability
    Hc=902181.163126629,  # Coercitivity field [A/m]
    alpha_Br=-0.001,  # temperature coefficient for remanent flux density /°C compared to 20°C
    Brm20=1.24,  # magnet remanence induction at 20°C [T]
    # lamination sheet width without insulation [m] (0 == not laminated)
    Wlam=0,
)

# Definition of the electric properties of the material
Magnet_prius.elec.rho = 1.6e-06  # Resistivity at 20°C

# Definition of the structural properties of the material
Magnet_prius.struct.rho = 7500.0  # mass per unit volume [kg/m3]


# Set Materials
stator.mat_type = M400_50A
rotor.mat_type = M400_50A
stator.winding.conductor.cond_mat = Copper1

# Set magnets in the rotor hole
rotor.hole[0].magnet_0.mat_type = Magnet_prius
rotor.hole[0].magnet_1.mat_type = Magnet_prius
rotor.hole[0].magnet_0.type_magnetization = 1
rotor.hole[0].magnet_1.type_magnetization = 1


IPMSM_Prius_2004 = MachineIPMSM(
    name="Toyota Prius 2004",
    stator=stator,
    rotor=rotor,
    shaft=shaft,
    frame=None
)
IPMSM_Prius_2004.save('IPMSM_Toyota_Prius_2004.json')

im = IPMSM_Prius_2004.plot()
