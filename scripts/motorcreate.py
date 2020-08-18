
from os import chdir
from pyleecan.Classes.MachineSIPMSM import MachineSIPMSM
from pyleecan.Classes.MatMagnetics import MatMagnetics
from pyleecan.Classes.Material import Material
from pyleecan.Classes.Frame import Frame
from pyleecan.Classes.Shaft import Shaft
from pyleecan.Classes.HoleM50 import HoleM50
from pyleecan.Classes.LamSlotMag import LamSlotMag
from pyleecan.Classes.CondType12 import CondType12
from pyleecan.Classes.WindingCW1L import WindingCW1L
from pyleecan.Classes.SlotW16 import SlotW16
from pyleecan.Classes.SlotMPolar import SlotMPolar
from pyleecan.Classes.LamSlotWind import LamSlotWind
from pyleecan.Functions.load import load
from pyleecan.Classes.MagnetType11 import MagnetType11

mm = 1e-3  # Millimeter
chdir('C:/Users/610099764')
# Lamination setup
stator = LamSlotWind(
    Rint=20 * mm,  # internal radius [m]
    Rext=59.2 * mm,  # external radius [m]
    # Lamination stack active length [m] without radial ventilation airducts
    L1=22 * mm,
    # but including insulation layers between lamination sheets
    Nrvd=0,  # Number of radial air ventilation duct
    Kf1=0.98,  # Lamination stacking / packing factor
    is_internal=True,
    is_stator=True,
)


# Slot setup
stator.slot = SlotW16(
    Zs=12,  # Slot number
    H0=2.0 * mm,  # Slot isthmus height
    H2=28.2 * mm,  # Slot height below wedge
    W0=0.115,  # Slot isthmus width
    W3=0.008,
    R1=1*mm  # Slot bottom radius
)


# Winding setup
stator.winding = WindingCW1L(
    qs=3,  # number of phases
    Lewout=0.005,  # staight length of conductor outside lamination before EW-bend
    p=5,  # number of pole pairs
    Ntcoil=269,  # number of turns per coil
    Npcpp=4,  # number of parallel circuits per phase
    # 0 not to change the stator winding connection matrix built by pyleecan number
    Nslot_shift_wind=0,
    # of slots to shift the coils obtained with pyleecan winding algorithm
    # (a, b, c becomes b, c, a with Nslot_shift_wind1=1)
    is_reverse_wind=False  # True to reverse the default winding algorithm along the airgap
    # (c, b, a instead of a, b, c along the trigonometric direction)
)

# Conductor setup
stator.winding.conductor = CondType12(
    Wwire=0.56 * mm,  # single wire width without insulation [m]
    Wins_wire=0.02 * mm,  # winding strand insulation thickness [m]
    Wins_cond=0.56*mm,  # single wire
    Nwppc=1
    # 0 for hairpin windings
    # 1 for normal windings
)


# Rotor setup
rotor = LamSlotMag(
    Rint=65 * mm,  # Internal radius
    Rext=69.5 * mm,  # external radius
    is_internal=False,
    is_stator=False,
    L1=stator.L1,  # Lamination stack active length [m],
    Nrvd=0,
    Wrvd=0,
    Kf1=1
    # without radial ventilation airducts but including insulation layers between lamination sheets
)

rotor.slot = SlotMPolar(
    W0=0.5436,
    Zs=10
)
# Set shaft
shaft = Shaft(Drsh=rotor.Rint * 2,  # Diamater of the rotor shaft [m]
              # used to estimate bearing diameter for friction losses
              Lshaft=1.2  # length of the rotor shaft [m]
              )


# Loading Materials
M400_50A = load('pyleecan/Data/Material/M400-50A.json')
Copper1 = load('pyleecan/Data/Material/Copper1.json')
Steel1 = load('pyleecan/Data/Material/Steel1.json')
# Defining magnets
Magnet_prius = Material(name="Magnet_prius")

# Definition of the magnetic properties of the material
Magnet_prius.mag = MatMagnetics(
    mur_lin=1.05,  # Relative magnetic permeability
    Hc=902181.163126629,  # Coercitivity field [A/m]
    alpha_Br=-0.001,  # temperature coefficient for remanent flux density /°C compared to 20°C
    Brm20=0.4,  # magnet remanence induction at 20°C [T]
    # lamination sheet width without insulation [m] (0 == not laminated)
    Wlam=0,
)

# Definition of the electric properties of the material
Magnet_prius.elec.rho = 1.6e-06  # Resistivity at 20°C

# Definition of the structural properties of the material
Magnet_prius.struct.rho = 7500.0  # mass per unit volume [kg/m3]


# Set Materials
stator.mat_type = M400_50A
rotor.mat_type = Steel1
stator.winding.conductor.cond_mat = Copper1

# Set magnets in the rotor hole
rotor.slot.magnet = [MagnetType11(
    Hmag=0.005,
    Lmag=0.95,
    Wmag=0.5463,
    mat_type=Magnet_prius
)]
""" rotor.hole[0].magnet_0.mat_type = Magnet_prius
rotor.hole[0].magnet_1.mat_type = Magnet_prius
rotor.hole[0].magnet_0.type_magnetization = 1
rotor.hole[0].magnet_1.type_magnetization = 1
 """

IPMSM_Prius_2004 = MachineSIPMSM(
    name="Regal Beloit EFU",
    stator=stator,
    rotor=rotor,
    # shaft=shaft,
    frame=None
)
IPMSM_Prius_2004.save('EFU.json')

im = IPMSM_Prius_2004.plot()
