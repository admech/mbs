within MBS.Examples.OmniVehicle.PointContact;

model Debug

  parameter Real    pi                   = Modelica.Constants.pi;

  // for runtime assertions
  parameter Integer precisionLevel_v     = 6;
  parameter Integer precisionLevel_omega = 6;

  // mass and geometry
  parameter Real    platformMass         = 10;
  parameter Integer n                    = 4                           "Number of rollers";
  parameter Real    alpha                = Modelica.Constants.pi / n   "Max angle of the half-sector";
  parameter Real    R                    = 1                           "Omni wheel outer radius";
  parameter Real    d                    = 2                           "Vehicle size";
  parameter Real    R1                   = R*cos(alpha)                "Omni wheel inner radius";

  // initial conditions
  parameter Real[3] r0                   = { 0, R, 0 }                 "Initial mass center position";
  parameter Real[3] v0                   = { 0, 0, 0 }                 "Initial mass center velocity";
  parameter Real[4] q0                   = { 1, 0, 0, 0 }              "Initial orientation quaternion";
  parameter Real    om0                  = 0                           "Initial angular velocity magnitude";
  parameter Real[3] omega0               = { 0, om0, 0 }               "Initial angular velocity vector";

  parameter Real[3] omega1_0             = omega0 + (-d)*om0/R * { 0, 0, 1 } "Initial angular velocity of wheel 1";
  parameter Real[3] omega2_0             = omega0 + (-d)*om0/R * { 0, 0, 1 } "Initial angular velocity of wheel 2";
  parameter Real[3] omega3_0             = omega0 + (-d)*om0/R * { 0, 0, 1 } "Initial angular velocity of wheel 3";

  // for plotting
  Real _angle   (start = 0);
  Real _omega0  (start = om0);
  Real _omega1  (start = sqrt(omega1_0 * omega1_0));
  Real _omega2  (start = sqrt(omega2_0 * omega2_0));
  Real _omega3  (start = sqrt(omega3_0 * omega3_0));
  Real _r1      (start = r0[1]);
  Real _r3      (start = r0[3]);
  Real _v1      (start = v0[1]);
  Real _v3      (start = v0[3]);

  annotation (
    experiment(
      StopTime=6,
      NumberOfIntervals=30000,
      Tolerance=1e-009,
      Algorithm="Dassl"
    ),
    experimentSetupOutput
  );

  // the actual components

  Base Floor;

  ThreePortsHeavyBody Platform
    ( m       = platformMass
    , Gravity = { 0, -1, 0 }
    , r       (start = r0)
    , v       (start = v0)
    , q       (start = q0)
    , omega   (start = omega0)
    );
 
  FixedJoint Joint1
    ( nA = { 0, 0, 1 }
    , nB = { 1, 0, 0 }
    , rA = { 0, 0, 0 }
    , rB = { d, 0, 0 }
    );
  FixedJoint Joint2
    ( nA = { 0, 0, 1 }
    , nB = { -cos(pi/3), 0, -cos(pi/6) }
    , rA = { 0, 0, 0 }
    , rB = { -d*cos(pi/3), 0, -d*cos(pi/6) }
    );
  FixedJoint Joint3
    ( nA = { 0, 0, 1 }
    , nB = { -cos(pi/3), 0, cos(pi/6) }
    , rA = { 0, 0, 0 }
    , rB = { -d*cos(pi/3), 0, d*cos(pi/6) }
    );

  OmniWheelGeneral Wheel1
    ( Gravity = { 0,-1,0 }
    , T0      =
       [ 0, 0, -1
       ; 0, 1, 0
       ; 1, 0, 0
       ]
    , r0      = r0 + { d, 0, 0 }
    , v0      = v0 + cross(omega0, { d, 0, 0 })
    , q0      = QMult(q0, { cos(pi/4), 0, sin(pi/4), 0 })
    , omega0  = omega1_0
    );
  OmniWheelGeneral Wheel2
    ( Gravity = { 0,-1,0 }
    , T0      =
      [ -cos(pi/6), 0, cos(pi/3)
      ; 0,          1, 0
      ; -cos(pi/3), 0, -cos(pi/6)
      ]
    , r0      = r0 + { -d*cos(pi/3), 0, -d*cos(pi/6) }
    , v0      = v0 + cross(omega0, { -d*cos(pi/3), 0, -d*cos(pi/6) })
    , q0      = QMult(q0, { cos(7*pi/12),0,sin(7*pi/12),0 })
    , omega0 = omega2_0
    );
  OmniWheelGeneral Wheel3
    ( Gravity = { 0,-1,0 }
    , T0      =
      [ cos(pi/6),  0, cos(pi/3)
      ; 0,          1, 0
      ; -cos(pi/3), 0, cos(pi/6)
      ]
    , r0      = r0 + { -d*cos(pi/3), 0,d *cos(pi/6) }
    , v0      = v0 + cross(omega0, { -d*cos(pi/3), 0, d*cos(pi/6) })
    , q0      = QMult(q0, { cos(-pi/12), 0, sin(-pi/12), 0 })
    , omega0  = omega3_0
    );

equation

  // for plotting
  der(_angle) = _omega0;
  _omega0     = Platform.omega[2];
  _omega1     = sqrt( Wheel1.Wheel.omega * Wheel1.Wheel.omega );
  _omega2     = sqrt( Wheel2.Wheel.omega * Wheel2.Wheel.omega );
  _omega3     = sqrt( Wheel3.Wheel.omega * Wheel3.Wheel.omega );
  _r1         = Platform.r[1];
  _r3         = Platform.r[3];
  _v1         = Platform.v[1];
  _v3         = Platform.v[3];

  assert(noEvent(Platform.v[2]*Platform.v[2] < 10^(-precisionLevel_v)),  "Platform has vertical speed !!!");
  assert(noEvent(Platform.omega[1]*Platform.omega[1] + Platform.omega[3]*Platform.omega[3] < 10^(-precisionLevel_omega)),  "Platform.omega is not all [2] !!!");

  connect( Floor.OutPort,          Wheel2.InPortK     );
  connect( Floor.OutPort,          Wheel3.InPortK     );
  connect( Wheel1.InPortK,         Floor.OutPort      );

  connect( Joint1.OutPortB,        Platform.InPort    );
  connect( Joint2.OutPortB,        Platform.InPort1   );
  connect( Joint3.OutPortB,        Platform.InPort2   );

  connect( Joint3.InPortB,         Platform.OutPort   );
  connect( Platform.OutPort,       Joint1.InPortB     );
  connect( Platform.OutPort,       Joint2.InPortB     );

  connect( Wheel1.InPortF,         Joint1.OutPortA    );
  connect( Wheel1.OutPortK,        Joint1.InPortA     );
  connect( Wheel2.InPortF,         Joint2.OutPortA    );
  connect( Wheel2.OutPortK,        Joint2.InPortA     );
  connect( Wheel3.InPortF,         Joint3.OutPortA    );
  connect( Wheel3.OutPortK,        Joint3.InPortA     );

end Debug;
