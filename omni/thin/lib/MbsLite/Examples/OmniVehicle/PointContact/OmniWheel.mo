within MbsLite.Examples.OmniVehicle.PointContact;

model OmniWheel

  parameter SI.Acceleration[3] Gravity (each start = inf);
  constant  Real[3] rollerAxisLocal  = { 1, 0, 0 };
  constant  Real[3] vertical         = { 0, 1, 0 };
  constant  Real[3] wheelAxis        = { 0, 0, 1 };

  parameter String  name = "NOT INITIALIZED";

  parameter Integer nOne = 5;

  parameter Integer      n        = -Integer_inf    "Number of rollers";
  parameter Real         psi      = inf             "Roller mecanum angle";
  parameter Real         rollerMass            = inf  ;
  parameter Real         rollerAxialMoi        = inf  "Roller moment of inertia wrt its axis";
  parameter Real         rollerOrthogonalMoi   = inf  "Roller moment of inertia wrt any axis orthogonal to the main roller axis";
  parameter Real         wheelHubMass          = inf  ;
  parameter Real         wheelHubAxialMoi      = inf  "Wheel hub moment of inertia wrt its axis";
  parameter Real         wheelHubOrthogonalMoi = inf  "Wheel hub moment of inertia wrt any axis orthogonal to the main wheelHub axis";
  parameter Real         alpha    = pi / n          "Max angle of the half-sector";
  parameter Real         R        = inf             "Omni wheel outer radius";
  parameter Real         R1       = R * cos(alpha)  "Omni wheel inner radius";
  parameter Real[3]      r0       (each start = inf);
  parameter Real[4]      q0       (each start = inf);
  parameter Real[3]      v0       (each start = inf);
  parameter Real[3]      omega0   (each start = inf);
  parameter Real[3, 3]   T0       = QToT(q0);

  parameter Real[nOne]     RollerAngles            = { (2 * alpha * (i - 1)) for i in 1 : nOne } "Angles between downward vertical { 0, -1, 0 } and roller center radius vectors";
  parameter Real[nOne, 4]  RollerQs                = { QMult(q0, QRot(RollerAngles[i], wheelAxis)) for i in 1 : nOne };
  /*
  parameter Real[nOne, 3]  RollerCenterDirections  = { { sin(RollerAngles[i]), -cos(RollerAngles[i]), 0 } for i in 1 : nOne };
  parameter Real[nOne, 3]  RollerAxisDirections    = { { cos(RollerAngles[i]), sin(RollerAngles[i]),  0 } for i in 1 : nOne };
  parameter Real[nOne, 3]  RollerCenters           = R1 * RollerCenterDirections;
  parameter Real[nOne, 3]  VerticalInRollersAxes   = { { sin(RollerAngles[i]), cos(RollerAngles[i]),  0 } for i in 1 : nOne };
  */
  parameter Real[nOne, 3]  VerticalInRollersAxes   = { QToT(RollerQs[i, :]) * vertical        for i in 1 : nOne };
  parameter Real[nOne, 3]  RollerCenterDirections  = -VerticalInRollersAxes;
  parameter Real[nOne, 3]  RollerAxisDirections    = { QToT(RollerQs[i, :]) * rollerAxisLocal for i in 1 : nOne };
  parameter Real[nOne, 3]  RollerCenters           = R1 * RollerCenterDirections;

  RollerPointContactForcesGeneral[nOne] Contacts
      ( name     = { name + ".Contacts[" + String(i) + "]" for i in 1 : nOne }
      , each n   = n
      , each R   = R
      , each psi = psi
      );

  NPortsHeavyBody[nOne] Rollers
      ( name = { name + ".Rollers[" + String(i) + "]" for i in 1 : nOne }
      , m = rollerMass
      , each I = diagonal({ rollerAxialMoi, rollerOrthogonalMoi, rollerOrthogonalMoi })
      , each N = 2
      , each Gravity = Gravity
      , r(start = { r0 + T0 * RollerCenters[i] for i in 1 : nOne })
      , v(start = { v0 + T0 * cross(omega0, RollerCenters[i]) for i in 1 : nOne })
      , q(start = { RollerQs[i,:] for i in 1 : nOne })
      , omega
          ( start
            = { { 0, 0, omega0[3] } for i in 1 : nOne }
            + omega0[2] * VerticalInRollersAxes
          )
      );

  FixedJoint[nOne] Joints
    ( each nA = { 1, 0, 0 }
    , nB = RollerAxisDirections
    , each rA = { 0, 0, 0 }
    , rB = RollerCenters
    );

  NPortsHeavyBody Wheel
    ( name = "wheel hub"
    , m = wheelHubMass
    , I = diagonal({ wheelHubOrthogonalMoi, wheelHubOrthogonalMoi, wheelHubAxialMoi })
    , N = 1 + nOne
    , Gravity = Gravity
    , r(start = r0)
    , v(start = v0)
    , q(start = q0)
    , omega(start = omega0)
    );

  KinematicPort  InPortK   "takes kinematics from base (which has only an OutPort and which needn't import any forces as it has no dynamics)";
  WrenchPort     InPortF   "imports forces from the platform or verticality constraint";
  KinematicPort  OutPortK  "exports kinematics to the platform or verticality constraint";

  Real[3]        w; // fixme: why do we need this variable ?

initial algorithm
  AssertInitializedS(name,  name,     "name");
  AssertInitializedI(name,  n,        "n");
  AssertInitialized (name,  { psi },  "psi");
  AssertInitialized (name,  { rollerMass },            "rollerMass");
  AssertInitialized (name,  { rollerAxialMoi },        "rollerAxialMoi");
  AssertInitialized (name,  { rollerOrthogonalMoi },   "rollerOrthogonalMoi");
  AssertInitialized (name,  { wheelHubMass },          "wheelHubMass");
  AssertInitialized (name,  { wheelHubAxialMoi },      "wheelHubAxialMoi");
  AssertInitialized (name,  { wheelHubOrthogonalMoi }, "wheelHubOrthogonalMoi");
  AssertInitialized (name,  { R },    "R");
  AssertInitialized (name,  r0,       "r0");
  AssertInitialized (name,  q0,       "q0");
  AssertInitialized (name,  v0,       "v0");
  AssertInitialized (name,  omega0,   "omega0");
  AssertInitialized (name,  Gravity,  "Gravity");

equation

  for i in 1 : nOne loop
    Contacts[i].n1k = Wheel.T * { 0, 0, 1 };
    Contacts[i].rho = (Wheel.r - Rollers[i].r) / sqrt((Wheel.r - Rollers[i].r) * (Wheel.r - Rollers[i].r));

    // interactions between Rollers and base (the floor) -- via Contacts
    connect(InPortK,                 Contacts[i].InPortA);  // kinematics from base (floor) goes in via InPortK and is being passed into Contacts via their InPortA
    connect(Rollers[i].OutPort,      Contacts[i].InPortB);  // similarly, kinematics from Rollers is fed in into Contacts via InPortB
    connect(Rollers[i].InPorts[1],   Contacts[i].OutPortB); // then what forces are calculated within Contacts, are being exported to Rollers from the Contacts' OutPortB and via the Rollers' first InPorts

    // interactions between Rollers and the Wheel hub -- via Joints
    connect(Rollers[i].InPorts[2],   Joints[i].OutPortA); // as everywhere, forces come from inside the Joints into Rollers (via OutPortA and right into second InPorts, respectively)
    connect(Rollers[i].OutPort,      Joints[i].InPortA);  // conversely, kinematics of the Rollers is being exported into Joints
    connect(Wheel.InPorts[1 + i],    Joints[i].OutPortB); // forces are also exported to the Wheel hub...
    connect(Wheel.OutPort,           Joints[i].InPortB);  // ... and kinematics is taken therefrom into the Joints
  end for;

  // interactions between the Wheel hub and outside world (most likely, the platform of the vehicle or a verticality constraint)
  connect(Wheel.InPorts[1],  InPortF);  // forces come in
  connect(Wheel.OutPort,     OutPortK); // kinematics are exported

  w = transpose(Wheel.OutPort.T) * (Rollers[1].r - Wheel.r);

end OmniWheel;
